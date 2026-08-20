# target_tracker 视觉追踪库实现文档

> 独立动态库（`libtarget_tracker.so`），实现轻量级多目标追踪过滤器：
> 恒定速度卡尔曼滤波平滑 + Coast 抗短暂丢失 + ID 稳定与主目标锁定。
> 算法行为的唯一权威规格为 `docs/视觉追踪技术路线.md`（下称"技术路线"），
> 本文档不重复抄录算法全文，仅描述实现结构与对照关系。

---

## 一、功能职责与边界

**做什么**：

- 每帧接收 YOLO 检测框列表，输出**一个主目标**的平滑中心点/边界框/状态标志
  （`TrackResult`），供下游速度闭环（PID）消费。
- 平滑：8 维恒定速度卡尔曼滤波（位置/尺寸 + 速度），抑制检测框逐帧抖动。
- Coast 预测：目标短暂消失时用滤波器预测值继续输出（`is_predicted=true`），
  避免控制输出突变；连续丢失超过 `max_lost_frames`（严格大于）才判丢。
- ID 稳定：贪婪最邻近关联（中心距离 + IoU 加权代价）维持轨道 ID；
  主目标锁定后不被高置信度新目标抢占，仅在轨道死亡或 `Reset()` 时解除。

**不做什么**（边界）：

- 不做光流估计、不做激光/惯导等多源融合（属 perception 模块职责）；
- 不做 Topic 订阅/发布接线、不依赖 spdlog/Eigen/OpenCV，零第三方依赖；
- 不做画面边界钳制与负尺寸约束（与技术路线 §4.5 注记保持一致）；
- 不要求检测置信度下限过滤（上游检测器已完成阈值过滤）。

## 二、接口与数据流

命名空间 `drone::tracker`，头文件保护 `#pragma once`。

| 头文件 | 内容 |
|--------|------|
| `include/target_tracker/kalman_box_filter.h` | `BoxCXCYWH`、`KalmanBoxFilter` |
| `include/target_tracker/tracklet.h` | `BoxXYXY`、`ToCenterSize`/`ToXYXY`/`ComputeIoU`、`Tracklet` |
| `include/target_tracker/target_tracker.h` | `Detection`、`FrameShape`、`TrackResult`、`TargetTracker` |

主要接口签名：

```cpp
class KalmanBoxFilter {
    void Initiate(const BoxCXCYWH& z);              // 首次观测初始化（§4.3）
    std::optional<BoxCXCYWH> Predict();             // 未初始化返回 nullopt（§4.4）
    void Update(const BoxCXCYWH& z);                // 观测更新（§4.5）
    bool Initialized() const;
    const std::array<double, 8>& State() const;     // [cx,cy,w,h,vx,vy,vw,vh]
};

struct Tracklet {
    Tracklet(int id, const BoxCXCYWH& measurement, double conf, int cls,
             int max_lost_frames);
    bool IsDead() const;      // lost_count > max_lost（严格大于）
    bool IsCoasting() const;  // lost_count > 0
    void Predict();           // kf.predict 非空时更新 last_center/last_box、age+1
    void Update(const BoxXYXY& box, double conf, int cls);  // 匹配更新
    void MarkMiss();          // lost+1、age+1
    // 字段：track_id / kf / lost_count / max_lost / confidence / cls_id /
    //       age / hits / last_center / last_box / last_raw_box
};

class TargetTracker {
    struct Config {  // §8.1 默认值
        int    max_lost_frames      = 8;
        double max_association_dist = 200.0;
        int    min_hits             = 3;
        double dist_weight          = 0.5;
        double iou_weight           = 0.5;
    };
    TargetTracker();                               // 默认配置
    explicit TargetTracker(const Config& cfg);
    TrackResult Update(const std::vector<Detection>& detections,
                       std::optional<FrameShape> frame_shape = std::nullopt);
    TrackResult LastResult() const;                // 线程安全拷贝
    void Reset();                                  // 清全部状态且 next_id 归零
    int ActiveCount() const;
    std::unordered_map<int, Tracklet> ActiveTracks() const;  // 只读快照
};
```

数据流：调用方每帧将检测列表（像素坐标 `[x1,y1,x2,y2,conf,cls_id]`）传入
`Update()`，得到 `TrackResult`（九字段：tracked / primary_id / center / box /
confidence / is_predicted / lost_frames / n_active / raw）。丢失时
`tracked=false`、定位字段空、`lost_frames=0`（技术路线 §2.3）。

## 三、关键实现点

### 3.1 卡尔曼滤波（对照技术路线 §4）

- 状态 `x[8]=[cx,cy,w,h,vx,vy,vw,vh]`，观测 `z[4]=[cx,cy,w,h]`，dt=1 帧。
- 常量矩阵：`F`（8×8，I4 + 右上 I4）、`H`（4×8，[I4|0]）为 `constexpr`；
  矩阵乘法/转置用 `std::array` 定长手写，全程 `double`（§9.3）。
- 常量：`kStdPos=1/50`、`kStdVel=1/200`、`kIniPos=2.0`、`kIniVel=100.0`、
  `kRScale=2.0`（§4.2/§8.2）。
- **动态噪声**：`Predict()` 时 Q=diag(σ_pos²,σ_vel²)，σ 直接用状态中的 w/h
  缩放；`Update()` 时 R 用 `abs(w)/abs(h)` 缩放并乘 2——两处差异与规格
  §4.2 注记严格保持一致。
- `Initiate()`：速度置零，P=diag(2,2,2,2,100,100,100,100)（§4.3）。
- `Update()` 中 S=HPHᵀ+R 为 4×4，用**高斯-约当消元 + 部分主元**求逆，
  消元前对角加 1e-9 正则化防奇异（§9.3）；随后 K=PHᵀS⁻¹、
  x←x+K(z−Hx)、P←(I−KH)P（§4.5）。

### 3.2 贪婪关联（对照 §5）

- `cost = dist_weight·dist/max(Dmax,1e-3) + iou_weight·(1−IoU)`，
  距离用 `std::hypot`；每条轨道 `best_cost` 初始化为 1.0，**仅 cost 严格
  小于才匹配**（cost=1.0 恰好不匹配，§5.1）。
- 按轨道 confidence 降序（平局按 ID 升序保证确定性）逐一贪心匹配；
  排序时复制 ID 到 `std::vector` 后 `std::sort`，不直接排哈希表（§9.2）。

### 3.3 主目标选举三级回退（对照 §6.3）

1. **锁定延续优先**：`locked_primary_id` 存活则直接输出（Coast 中照样输出
   `is_predicted=true`），不被高置信度目标抢占；
2. **重新选举**（无锁定/锁定已死亡）：measured（lost==0）中 hits≥min_hits
   的已确认池取 confidence 最大；池空则退回全部 measured；
3. 无 measured 时从 coasting 中选离画面中心最近者（欧氏距离平方，
   `frame_shape` 缺省按 640×480 中心 (320,240)），`is_predicted=true`；
   都没有则 `tracked=false`。

锁定仅在轨道死亡删除处或 `Reset()` 时解除。

### 3.4 主流程六步（对照 §7）

`Update()` 严格按：①全体轨道 predict ②检测中心表 + 贪婪关联 ③匹配轨道
update（raw_box 由检测重建）④未匹配轨道 mark_miss、死亡即删、若为锁定
目标同时清锁 ⑤未匹配检测新建轨道（创建当帧赋 last_box/last_raw_box，
保证当帧可参与输出；next_id 从 0 递增永不复用）⑥主目标选举组装输出并
缓存 last_result。

### 3.5 线程模型（对照 §9.5）

单线程写 `Update()`（推理主循环）；库内仅对 `last_result` 缓存加一把
`std::mutex`，`LastResult()` 返回受保护拷贝。内部轨道状态不加锁；
若未来改为多线程调用 `Update()` 需整体加锁。

### 3.6 符号导出说明

当前不加 `-fvisibility=hidden`、不使用导出宏，共享库默认导出全部公共符号。
日后若启用隐藏可见性（如 `CXX_VISIBILITY_PRESET hidden`），需补
`TARGET_TRACKER_API` 宏（导出/导入双态）并标注所有公共类与函数，
否则下游链接将缺失符号。

## 四、日志行为

本库为**纯算法层，不打任何日志**（不依赖 spdlog）。若未来接入主工程，
接线层（订阅/发布适配部件）按主工程日志纪律负责关键路径日志与节流，
本库内部不新增日志。

## 五、测试方式

构建环境：WSL2 Ubuntu（AGENTS.md 规定的开发机构建环境）。

```bash
cd /mnt/d/ProgramData/drone_test/drone_test
cmake -S target_tracker -B target_tracker/build
cmake --build target_tracker/build -j$(nproc)
ctest --test-dir target_tracker/build --output-on-failure
```

测试共 27 条（kalman_box_filter_test 5 条 / tracklet_test 12 条 /
target_tracker_test 10 条），其中 `target_tracker_test.cpp` 逐条覆盖
技术路线 §9.4 六条验收用例：

1. `EmptyFramesCoastThenLost`：空检测帧 Coast 持续 max_lost_frames 帧
   （is_predicted=true），第 max_lost_frames+1 帧 tracked=false；
2. `IdStrictlyIncreasingNoReuse`：ID 严格递增不复用，Reset 后归零；
3. `LockNotStolenAndSwitchAfterDeath`：Coast 期间高置信度新目标不抢占锁定，
   锁定目标死亡后切换；
4. `MinHitsFallbackAllowsNewTrack`：无已确认实测轨道时新轨道（hits<min_hits）
   可当选主目标；
5. `CostExactlyOneDoesNotMatch`：纯距离项构造 cost=1.0 恰好不匹配，
   并附 cost<1.0 对照帧；
6. `HigherConfidenceTrackWinsDetection`：两轨道竞争同一检测，置信度高者获胜。

其余为补充行为测试（丢失输出字段约定、Coast 兜底选举、锁定延续、
默认画面中心、IoU 边界、滤波器收敛性等）。

## 六、排查 / 修改要点

- **主目标"甩头"**：确认是否误改锁定解除逻辑——锁定只能在轨道死亡删除处
  或 `Reset()` 解除；选举分支不得覆盖存活锁定。
- **Coast 时间不对**：死亡判定为 `lost_count > max_lost`（严格大于），
  默认允许 Coast 8 帧、第 9 帧判丢；改成 `>=` 属于行为变更。
- **关联不上 / 乱匹配**：检查 `max_association_dist` 与目标帧间位移是否
  匹配；cost 判据为严格小于 1.0；dist/iou 权重之和建议保持 1.0。
- **滤波发散或跳变**：检查 Q/R 动态噪声是否按状态 w/h 缩放（R 用 abs 且
  乘 2、Q 不用 abs，该差异是规格要求）；4×4 求逆已加 1e-9 正则化。
- **测试链接失败（找不到 so）**：测试可执行文件通过 `CMAKE_BUILD_RPATH`
  定位库；若把测试挪到别处运行需设置 `LD_LIBRARY_PATH`。
- 修改任何行为后必须同步更新本文档与 `docs/视觉追踪技术路线.md` 的对照注释。

## 七、未来接线指引（接入主工程视频链路）

本库以独立动态库交付，主工程接入建议采用：

1. **订阅解码帧主题作为每帧节拍**：接线部件订阅解码帧 Topic，以解码帧
   到达为帧节拍驱动追踪（保证与推理节奏一致）；
2. **按 frame_sequence 聚合 DetectionResult**：YOLO 每目标发布一条
   DetectionResult（同帧共享 frame_sequence），接线部件按 frame_sequence
   聚合成 `std::vector<Detection>` 后调用 `Update()`；
3. **主工程 `include/common/types.h` 新增 `TrackedTarget` 消息与
   `kTrackedTarget` 主题**：由接线部件把 `TrackResult` 转换为
   `TrackedTarget` 发布，供控制线程（20Hz PID）订阅消费；Coast
   （is_predicted=true）时调用方清 PID 积分、tracked=false 时零速刹车
   （技术路线 §1.2 上游行为约定）。

主工程链接该 so 的 CMake 片段示例：

```cmake
# 主工程 CMakeLists.txt 中引入视觉追踪库（独立子工程）
add_subdirectory(target_tracker)

target_link_libraries(drone_control PRIVATE target_tracker)
# 部署时需将 libtarget_tracker.so.1 一并打包到可执行文件目录
# （或设置 INSTALL_RPATH=$ORIGIN，与 docs/部署与打包.md 目录式部署约定一致）
```
