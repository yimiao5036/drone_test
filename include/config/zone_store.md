# 围栏区域缓存（zone_store）

## 功能职责

把地面站下发的 `common::FenceZone` 落盘成 `config/zone.json`，并在启动时读回，
使飞机在**丢失地面站 65018 的情况下仍能按最近一次合法区域执行围栏**。

- 不做：不校验区域是否包含当前 Home（几何判定在状态机）、不主动发送、不做版本协商。
- 边界：文件只是**缓存快照**；权威来源是地面站 65018。读到内容也要经
  `MissionStateMachine::ApplyFenceZone()` 做同一套合法性检查。

## 接口与数据流

```cpp
class ZoneStore {
public:
    static std::optional<common::FenceZone> Load(const std::string& path);
    static bool Save(const std::string& path, const common::FenceZone& zone);
};
```

JSON 键（`schema=1`）：

| 键 | 类型 | 语义 |
|---|---|---|
| `schema` | int | 文件结构版本，当前 1 |
| `protocol_version` | int | 65018 协议版本（`kFenceZoneProtocolVersion`） |
| `zone_id` / `version` | int | 区域编号与版本（两者都必须非 0） |
| `type` | int | 0=多边形 |
| `action` | string | `hold` / `return` / `land` |
| `altitude_reference` | int | 0=AMSL、1=Home 相对 |
| `alt_min_m` / `alt_max_m` | double | 高度带（`alt_max_m=0` 表示不限） |
| `vertices[]` | array | `{latitude_deg, longitude_deg}`，至少 3 个 |

数据流：`GroundStationLink` 65018 → `FenceZone` Topic → `MissionStateMachine::ApplyFenceZone()`
→ 内容变化时 `ZoneStore::Save()`；启动时 `MissionStateMachine::Start()` →
`ZoneStore::Load()` → 作为初始 `fence_zone_`（`fence_dirty_=true`，首拍重建多边形）。

## 关键实现点

- **原子写**：先写 `path + ".tmp"`，`std::filesystem::rename()` 覆盖目标文件；
  同分区 rename 是原子操作，掉电不会留下半个文件。写失败记 ERROR 但不中断任务。
- **读取宽松**：文件不存在 → 静默返回 `nullopt`（首次部署正常情况）；解析异常、
  字段缺失/类型错、`zone_id==0`、`version==0`、`vertices<3` → WARN 并返回 `nullopt`，
  让状态机按 `kNoZone` 处理（不越界、不拦起飞）。缓存只做**结构**校验（字段/类型/
  顶点数）；几何合法性（自交、零面积、重复点、高度带倒挂、非法 `altitude_reference`）
  由 `MissionStateMachine::ApplyFenceZone()` 调 `control::fence::ValidateZone()` 拒收
  （2026-10-30 审查修复：此前缓存里的退化区域会被直接采用）。
- **写盘时机以内容为准**：`ApplyFenceZone()` 用 `FenceZoneContentChanged()` 比较
  `zone_id/version/type/action/altitude_reference/alt_min/alt_max/顶点集`，同
  `zone_id+version` 但内容变化也会重写缓存（旧实现只比 id/version，地面站修
  顶点却不升版本时缓存与内存不一致，重启后回滚旧几何）。
- 路径由 `fence.zone_cache_path` 配置；`config.cpp` 里用 `ResolveAdjacentPath(ConfigDirectoryOf(path), ...)` 
  解析，与 `config/` 同级部署，不受当前工作目录影响。
- 不保存 `received_ms` 的真实时间语义：`Load()` 返回的 `received_ms` 用“加载时刻”填充，
  仅用于日志展示，不参与有效期判定（围栏区域本身就是持久数据，`valid_for_ms=0`）。

## 日志行为

- INFO `围栏区域缓存已装载: path=... zone_id=.. version=.. vertices=..`（启动成功）。
- WARN `围栏区域缓存不可用: path=... （收到 65018 之前按 no_zone 处理）`。
- INFO `围栏区域缓存已保存: path=...`；失败 ERROR `围栏区域缓存写入失败`。
- 读文件本身不节流：只在启动一次，写只在区域内容变化时发生。

## 测试方式

```bash
./build/config_test --gtest_filter='ZoneStoreTest.*'   # 3 例：往返、缺文件、非法内容
```

状态机侧的缓存行为（首拍重建、缓存非法时不越界）在
`tests/state_machine/mission_state_machine_test.cpp` 的 `Fence*` 用例里验证；
临界场景（解析失败、目录不可写）只能靠日志确认，属硬件/文件系统相关。

## 排查/修改要点

- 上电后 65019 一直 `no_zone` 且没有 65018：检查 `config/zone.json` 是否存在、
  `fence.enabled` 是否为 true、地面站是否发过区域。
- 改了 JSON 键名必须同步 `ZoneStore::Load/Save`，否则老文件会在升级后被判非法
  （读侧会 WARN 并忽略，不会崩，但围栏会退化成 `no_zone`）。
- 该文件是**运行时缓存**，由程序写；不建议手工编辑后提交到仓库
  （`config/zone.json` 默认可不入库，见 `docs/部署与打包.md`）。
