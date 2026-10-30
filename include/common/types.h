/**
 * @file types.h
 * @brief 模块间公共消息类型与 Topic 名称常量
 *
 * 属于 drone/common 模块。所有跨模块传递的数据类型集中在此定义，
 * 各部件接口（ICameraReceiver、IVideoDecoder 等）只依赖本文件中的类型，
 * 保证并行开发时数据接口一致。
 *
 * 与 docs/数据接口文档.md、docs/archive/早期设计/通信与数据定义.md 对应：
 * - 数据接口文档：代码级契约（本文件类型 + 接口签名 + Topic 一览）
 * - 通信与数据定义：物理链路与字段语义
 *
 * 字段语义约定：
 * - 时间戳统一使用单调时钟毫秒（receive_time_ms / valid_until_ms）。
 * - 无效数值使用 NaN（float 距离、精度等）；无效标志用布尔或位标志显式表达。
 * - frame_id 坐标系编码：0=未知 1=WGS84 经纬度 2=机体(前右下) 3=局部NED 4=局部ENU。
 * - 消息以 shared_ptr<const T> 经 Topic 分发，结构体内不持有大块像素数据
 *   （视频帧使用 video::FrameHandle，见 include/video/video_frame.h）。
 */
#pragma once

#include <cstdint>
#include <limits>
#include <vector>

namespace drone::common {

/// 通用消息头。所有内部控制相关消息必须包含。
struct MessageHeader {
    uint64_t sequence = 0;         ///< 来源内单调递增序号
    uint64_t source_time_ms = 0;   ///< 设备或地面站产生数据的时间；不可用时为 0
    uint64_t receive_time_ms = 0;  ///< 算力板收到数据的单调时钟时间
    uint64_t valid_for_ms = 0;     ///< 数据允许使用的最长时间（超时判断用 receive_time_ms）
    uint8_t source_id = 0;         ///< 数据来源标识
    uint8_t health = 0;            ///< 来源健康与数据质量（0=未知 1=良好 2=降级 3=故障）
    uint8_t frame_id = 0;          ///< 坐标系标识（空间数据），编码见文件头注释
};

/// 视频编码类型（码流块语义）。
enum class VideoCodec : uint8_t {
    kUnknown = 0,  ///< 未知编码
    kH264,         ///< H.264 / AVC
    kH265,         ///< H.265 / HEVC
    kMjpeg,        ///< MJPEG（USB UVC 双目相机，一帧一个访问单元）
};

/// H.265/H.264/MJPEG 码流块（摄像头采集 → 视频解码器）。
/// 骨架期使用字节容器；大数据零拷贝优化在实现期按性能实测决定。
/// MJPEG 语义：一帧=一个访问单元、is_key_frame=true、parameter_sets 为空。
struct EncodedFrame {
    MessageHeader header;
    VideoCodec codec = VideoCodec::kUnknown;  ///< 编码类型
    uint32_t stream_sequence = 0;  ///< 码流序号（解码侧用于乱序/丢包检测）
    uint64_t capture_time_ms = 0;  ///< 摄像头时间；不可用时为 0（不得伪造）
    bool is_key_frame = false;     ///< 是否关键帧（I 帧）
    /// 流级参数集（H.264 SPS/PPS 或 H.265 VPS/SPS/PPS，annexb 或 avcC/hvcC
    /// 原始字节）。RTSP/容器流的参数集在流级（SDP/容器头）而非逐帧码流中，
    /// 由 CameraReceiver 在连接建立后单独发布一条仅含参数集的消息（data 为空）；
    /// 裸码流无独立参数集时留空，解码器从关键帧内嵌参数集解析。
    std::vector<uint8_t> parameter_sets;
    std::vector<uint8_t> data;     ///< 码流字节（一个可送入解码器的访问单元，含帧边界）
};

/// 检测结果（YOLO → 感知融合 / 目标估计）。
struct DetectionResult {
    MessageHeader header;
    uint64_t frame_sequence = 0;  ///< 对应图像帧序号
    uint32_t class_id = 0;        ///< 类别：0=无人机 1=障碍物（后续按模型扩展）
    float confidence = 0.f;       ///< 检测置信度 [0,1]
    float bbox_x = 0.f;           ///< 像素坐标框左上角 x
    float bbox_y = 0.f;           ///< 像素坐标框左上角 y
    float bbox_w = 0.f;           ///< 像素坐标框宽
    float bbox_h = 0.f;           ///< 像素坐标框高
    float center_pixel_x = 0.f;   ///< 目标中心像素 x
    float center_pixel_y = 0.f;   ///< 目标中心像素 y
    uint32_t track_id = 0;        ///< 跟踪器目标编号；0=未跟踪（V1 可选）
    float inference_time_ms = 0.f;///< 推理耗时
};

/// 单目标视觉观测（YOLO → 视觉稳定性判定/后续视觉跟踪）。
/// YoloDetector 对每一个成功完成推理的帧发布且仅发布一条：
/// - 本帧检测到目标：detected=true，几何字段有效，取置信度最高者作为唯一目标；
/// - 本帧没有目标：detected=false，几何字段无效。
/// 这是视觉链路的帧级节拍。解码帧会被实时队列丢弃（YOLO输入队列容量为1），
/// 因此不能拿解码帧序号判断“本帧推理完成”，只有本消息能表达“该帧确实推理完成且结果为空”。
struct VisualTargetObservation {
    MessageHeader header;
    uint64_t frame_sequence = 0;  ///< 对应已完成推理的图像帧序号
    bool detected = false;        ///< 本帧是否检测到目标；false时后续几何字段无效
    uint32_t class_id = 0;        ///< 类别：0=无人机
    float confidence = 0.f;       ///< 唯一目标置信度 [0,1]
    float bbox_x = 0.f;           ///< 像素坐标框左上角 x
    float bbox_y = 0.f;           ///< 像素坐标框左上角 y
    float bbox_w = 0.f;           ///< 像素坐标框宽
    float bbox_h = 0.f;           ///< 像素坐标框高
    float center_pixel_x = 0.f;   ///< 目标中心像素 x
    float center_pixel_y = 0.f;   ///< 目标中心像素 y
    uint32_t candidate_count = 0; ///< 本帧过滤后候选目标数（用于观测多目标出现）
    uint32_t frame_width = 0;     ///< 图像有效宽，供归一化和后续标定使用
    uint32_t frame_height = 0;    ///< 图像有效高
    float inference_time_ms = 0.f;///< 本帧推理耗时
};

/// 单目标视觉稳定性状态。当前仅影子发布，不驱动飞行控制。
enum class VisualTargetState : uint8_t {
    kSearching = 0,  ///< 尚无稳定检测（从未检测到或短时丢失后未重新确认）
    kAcquiring = 1,  ///< 连续检测但未达锁定门限
    kLocked = 2,     ///< 连续稳定检测，可用于后续视觉跟踪门禁
    kLost = 3,       ///< 连续丢失或观测超时
};

/// 单目标视觉稳定性状态（视觉稳定性判定 → 状态机/诊断影子消费者）。
struct VisualTargetStatus {
    MessageHeader header;
    VisualTargetState state = VisualTargetState::kSearching;
    uint64_t frame_sequence = 0;        ///< 最近一次观测对应的图像帧序号
    uint64_t last_seen_ms = 0;          ///< 最近一次有效检测的单调时钟时间；从未检测为0
    int consecutive_detected_frames = 0;///< 连续检测到目标的帧数
    int consecutive_missed_frames = 0;  ///< 连续未检测到目标的帧数
    float confidence = 0.f;             ///< 平滑后置信度
    float center_pixel_x = 0.f;         ///< 平滑后目标中心像素 x
    float center_pixel_y = 0.f;         ///< 平滑后目标中心像素 y
    uint32_t frame_width = 0;           ///< 最近观测的图像宽
    uint32_t frame_height = 0;          ///< 最近观测的图像高
    bool valid = false;                 ///< 是否处于可用锁定状态（当前仅诊断用途）
};

/// 双目测距目标距离（StereoRanger → 视觉跟踪影子距离通道）。
/// 对应设计 docs/superpowers/specs/2026-09-23-双目测距v1.0-design.md。
/// v1.0 单目标约定：track_id 恒 0；valid=false 时距离字段为 NaN 不可用，
/// 方位字段（左目框、中心）照常有效。
struct StereoTargetDistance {
    MessageHeader header;
    uint64_t frame_sequence = 0;    ///< 左目帧序号
    uint32_t class_id = 0;          ///< 类别：0=无人机
    uint32_t track_id = 0;          ///< v1.0 恒 0（多目标关联留 v1.1）
    float forward_distance_m = std::numeric_limits<float>::quiet_NaN();  ///< 前向距离Z（米，滤波后）
    float slant_range_m = std::numeric_limits<float>::quiet_NaN();       ///< 斜距R（米）
    float disparity_px = std::numeric_limits<float>::quiet_NaN();  ///< 归一化视差折算左目像素视差
    float confidence = 0.f;         ///< 左右目置信度取小；未匹配时为左目置信度
    bool valid = false;             ///< 匹配成功且在有效域内（disparity≥最小视差且距离在域内）
    float center_pixel_x_mid = 0.f; ///< 双目中心x中值；未匹配时为左目中心x
    // 目标在基准相机系（SBS 右半=物理左目校正域：x 右 / y 下 / z 前）的三维
    // 位置（米，EMA 平滑后）；z 分量即 forward_distance_m。valid=false 时不可用。
    // 供视觉观测 → 局部 NED 转换（TargetEstimator 视觉融合）使用。
    float camera_x_m = std::numeric_limits<float>::quiet_NaN();
    float camera_y_m = std::numeric_limits<float>::quiet_NaN();
    float bbox_x = 0.f;             ///< 左目框（像素）
    float bbox_y = 0.f;
    float bbox_w = 0.f;
    float bbox_h = 0.f;
};

/// 光流结果（光流估计 → 感知融合）。
struct OpticalFlowResult {
    MessageHeader header;
    uint64_t frame_sequence = 0;      ///< 当前帧序号
    uint64_t ref_frame_sequence = 0;  ///< 参考（上一）帧序号
    float flow_x = 0.f;               ///< 平均像素位移 x
    float flow_y = 0.f;               ///< 平均像素位移 y
    float quality = 0.f;              ///< 质量 [0,1]；低纹理/强压缩/运动模糊/光照异常时降低
    bool valid = false;               ///< 本次估计是否有效
};

/// 距离样本预留类型。当前产品已取消独立单点激光雷达，计划由双目摄像头深度模块输出；
/// 不得自动解释为任意视觉目标的距离。
struct LaserRangeSample {
    MessageHeader header;
    float distance_m = 0.f;         ///< 有效距离（米）；无效时 valid=false
    bool valid = false;             ///< 距离是否有效
    uint8_t quality = 0;            ///< 信号质量 [0,255]
    uint8_t error_code = 0;         ///< 厂商错误码；0=无错误
    float update_hz = 0.f;          ///< 当前更新频率
    uint64_t consecutive_error_count = 0;  ///< 连续异常计数
};

/// 地面站目标状态（地面站 → 状态机/目标估计）。动态目标引导用，
/// 不得按静态航点永久保存。
struct GroundStationTarget {
    MessageHeader header;
    uint32_t ground_station_boot_id = 0;
    uint32_t update_seq = 0;
    uint32_t target_id = 0;
    int32_t latitude_1e7 = 0;     ///< WGS-84 纬度 ×1e7
    int32_t longitude_1e7 = 0;    ///< WGS-84 经度 ×1e7
    int32_t altitude_mm = 0;      ///< 高度（毫米），基准见 alt_reference
    uint8_t alt_reference = 0;    ///< 0=未知 1=MSL 2=AGL 3=WGS84 椭球
    uint8_t protocol_version = 0; ///< TRACK_TARGET_UPDATE协议版本
    uint64_t transport_age_ms = 0;///< 基于TIMESYNC扣除得到的传输年龄
    float velocity_north_mps = 0.f;
    float velocity_east_mps = 0.f;
    float velocity_down_mps = 0.f;
    float heading_deg = 0.f;      ///< 相对真北；无效为 NaN
    float horizontal_accuracy_m = 0.f;  ///< 水平定位精度
    float vertical_accuracy_m = 0.f;    ///< 垂直定位精度
    /// 可选字段有效标志：bit0=altitude bit1=vN bit2=vE bit3=vD
    /// bit4=heading bit5=horizontal_accuracy bit6=vertical_accuracy
    uint32_t validity_flags = 0;
};

/// 任务状态（状态机，S0~S16，见 docs/状态机设计.md）。
enum class MissionState : uint8_t {
    kBoot = 0,          ///< S0 进程启动和基础资源初始化
    kSelfCheck,         ///< S1 检查飞控、地面站、摄像头、雷达、模型和配置
    kReady,             ///< S2 等待地面站下发任务
    kArming,            ///< S3 请求 PX4 解锁并验证
    kTakeoff,           ///< S4 起飞到任务高度
    kGpsApproach,       ///< S5 按地面站目标位置远距离位置引导
    kVisualHandover,    ///< S6 验证视觉稳定性并切换位置→速度控制
    kVisualTracking,    ///< S7 视觉/光流/飞行状态速度与方向跟踪
    kObstacleHold,      ///< S8 前方危险减速、刹停并悬停
    kInterceptReady,    ///< S9 检查锁定、执行机构和可选授权
    kInterceptExecuting,///< S10 调用抽象拦截接口并等待结果
    kPostIntercept,     ///< S11 评估目标与系统状态；V1 默认返航
    kReturnHome,        ///< S12 返回 PX4 上报的 Home 点
    kLanding,           ///< S13 降落并确认落地
    kDisarmed,          ///< S14 确认上锁，任务结束
    kManualOverride,    ///< S15 遥控器人工接管
    kFailsafe,          ///< S16 降级执行；完全失效依赖 PX4 保护
    kCount,
};

/// PX4 飞行状态快照（PX4 通信 → 状态机/控制/感知融合）。
/// 状态机使用不可变快照，避免一次决策读取到不同时间的零散字段。
struct FlightStateSnapshot {
    MessageHeader header;
    bool connected = false;       ///< PX4 心跳与连接状态
    bool armed = false;
    bool landed = false;          ///< 仅 landed_state_valid=true 时可使用
    bool landed_state_valid = false;
    uint8_t base_mode = 0;        ///< MAVLink HEARTBEAT.base_mode
    uint32_t custom_mode = 0;     ///< PX4 HEARTBEAT.custom_mode 原始值
    uint8_t flight_mode = 0;      ///< PX4 1.17.0 custom main mode
    uint8_t flight_sub_mode = 0;  ///< PX4 1.17.0 custom sub mode
    uint8_t system_status = 0;    ///< MAV_STATE
    bool gps_state_valid = false;
    bool gps_fix = false;
    uint8_t gps_fix_type = 0;     ///< MAVLink GPS_FIX_TYPE
    uint8_t gps_satellites_visible = 0;  ///< 可见卫星数（gps_satellites_valid=true 时有效）
    bool gps_satellites_valid = false;
    float gps_eph_m = 0.f;        ///< 水平定位精度（米，gps_accuracy_valid=true 时有效）
    float gps_epv_m = 0.f;        ///< 垂直定位精度（米，同上）
    bool gps_accuracy_valid = false;
    bool global_position_valid = false;
    int32_t latitude_1e7 = 0;     ///< 全局位置 WGS-84 ×1e7
    int32_t longitude_1e7 = 0;
    int32_t altitude_mm = 0;      ///< MSL 高度（毫米）
    bool local_position_valid = false;
    float local_x_m = 0.f;        ///< 局部 NED 位置
    float local_y_m = 0.f;
    float local_z_m = 0.f;
    float vx_mps = 0.f;           ///< 速度（与局部位置同坐标系）
    float vy_mps = 0.f;
    float vz_mps = 0.f;
    bool attitude_valid = false;
    float roll_rad = 0.f;         ///< 姿态（机体欧拉角）
    float pitch_rad = 0.f;
    float yaw_rad = 0.f;
    float baro_alt_m = 0.f;       ///< 气压高度及其参考
    bool home_valid = false;
    int32_t home_lat_1e7 = 0;
    int32_t home_lon_1e7 = 0;
    int32_t home_altitude_mm = 0;
    /// EKF 投影参考锚点（MAVLink GPS_GLOBAL_ORIGIN = LOCAL_POSITION_NED.ref_* 的
    /// MAVLink 出口）：局部坐标系的 WGS84 锚，与持续刷新的 Home 不同源。该消息为
    /// 变化触发式（锚点变化 >1.1cm 才发），`ned_ref_received_count` 用于判读锚点
    /// 稳定性（稳定则全程 ≈1 条）——Home 漂移处置方案裁决的现场观测依据。
    bool ned_ref_valid = false;
    int32_t ned_ref_lat_1e7 = 0;
    int32_t ned_ref_lon_1e7 = 0;
    int32_t ned_ref_alt_mm = 0;
    uint64_t ned_ref_received_count = 0;
    bool battery_valid = false;         ///< 电压有效且数据新鲜
    float battery_voltage_v = 0.f;      ///< 动力电池电压
    bool battery_current_valid = false;
    float battery_current_a = 0.f;      ///< 动力电池电流
    bool battery_remaining_valid = false;
    float battery_remaining_pct = 0.f;  ///< 剩余电量百分比
    /// PX4 电池保护参数（PARAM_REQUEST_READ/PARAM_VALUE 读取，静态配置、非遥测流）：
    /// 断连或 Stop 立即失效；连接期间读取成功后保持，不参与 telemetry_timeout。
    bool battery_low_threshold_valid = false;
    float battery_low_threshold_pct = 0.f;  ///< PX4 BAT_LOW_THR（百分比）
    bool low_battery_action_valid = false;
    uint8_t low_battery_action = 0;         ///< PX4 COM_LOW_BAT_ACT 枚举值（四舍五入）
    uint64_t battery_parameter_time_ms = 0; ///< 最近一次参数回包的单调时钟毫秒
    bool rc_state_valid = false;
    bool rc_connected = false;          ///< 遥控器链路在线，不等于人工接管
    uint8_t rc_rssi = 0;                ///< MAVLink RC_CHANNELS.rssi
    bool manual_override_active = false;///< 后续按模式开关/控制权规则判断
    bool autopilot_version_valid = false;
    uint32_t flight_sw_version = 0;      ///< AUTOPILOT_VERSION 原始版本编码
    uint8_t firmware_major = 0;
    uint8_t firmware_minor = 0;
    uint8_t firmware_patch = 0;
    uint64_t autopilot_capabilities = 0;
    uint16_t last_ack_command = 0;      ///< 最近 MAVLink 命令 ACK 对应的命令
    uint8_t last_ack_result = 0;        ///< 最近 ACK 结果
    uint64_t last_ack_time_ms = 0;
};

/// 控制意图类型（状态机/策略 → 飞行控制）。
enum class ControlIntentType : uint8_t {
    kNone = 0,
    kPositionGuide,     ///< 位置引导意图
    kVelocityHeading,   ///< 速度与航向意图
    kBrakeHover,        ///< 刹停/悬停意图
    kTakeoff,           ///< 起飞意图
    kLand,              ///< 降落意图
    kRtl,               ///< 返航意图
    kManualOverride,    ///< 禁止自动输出/人工接管意图
};

/// 逐轴速度有效位（TargetState / ControlIntent / Px4Setpoint 共用）。
/// 位号约定：bit0=N、bit1=E、bit2=D；与 GroundStationTarget.validity_flags
/// 的 bit1/2/3（vN/vE/vD）不同，估计器内负责显式映射。
constexpr uint8_t kVelocityValidNorth = 1u << 0;
constexpr uint8_t kVelocityValidEast = 1u << 1;
constexpr uint8_t kVelocityValidDown = 1u << 2;
constexpr uint8_t kVelocityValidAll =
    kVelocityValidNorth | kVelocityValidEast | kVelocityValidDown;

/// 控制意图（状态机/策略 → 飞行控制器）。
/// 生成状态、优先级、有效期、坐标系、限制条件和产生原因齐备；
/// PX4 通信线程发送前再次检查控制权和有效期。
struct ControlIntent {
    MessageHeader header;
    ControlIntentType type = ControlIntentType::kNone;
    uint8_t priority = 0;        ///< 优先级，数值越大越优先
    uint8_t control_source = 0;  ///< 0=位置 1=视觉 2=安全 3=遥控器 4=PX4 保护
    float target_x = 0.f;        ///< 目标位置或速度分量（坐标系见 frame_id）
    float target_y = 0.f;
    float target_z = 0.f;
    float yaw_deg = 0.f;         ///< 绝对航向（度，NED；位置式航向通道）
    float yaw_rate_dps = 0.f;    ///< 偏航角速度指令（度/秒；速率式航向通道，
                                 ///< 2026-09-23 视觉跟踪移植新增，默认 0=不使用）
    /// 航向角通道是否有效（2026-10-29 返航机头朝飞行方向）。历史约定用
    /// `yaw_deg != 0` 判“是否指定航向”，无法表达正北 0°，故新增显式有效位。
    /// false：不指定航向，PX4 保持原机头平移；true：按 yaw_deg 控制机头。
    bool yaw_valid = false;
    float max_speed_mps = 0.f;   ///< 速度限制；0=不限制
    /// 位置型意图的速度前馈（局部 NED，m/s；2026-10-19 P3）。
    float feedforward_vx_mps = 0.f;
    float feedforward_vy_mps = 0.f;
    float feedforward_vz_mps = 0.f;
    /// 前馈逐轴有效位（kVelocityValid*）；0=无前馈。仅 kPositionGuide 使用。
    uint8_t feedforward_valid_mask = 0;
    uint32_t reason_code = 0;    ///< 产生原因编码（见 docs/数据接口文档.md 附录）
    uint64_t valid_until_ms = 0; ///< 有效期截止（单调时钟）
};

/// ControlIntent::reason_code 高位标志（低位段沿用 docs/数据接口文档.md §3.5
/// 枚举：0~7 规划语义、100+ 视觉跟踪影子段，高位为标志位段）：
/// bit31 = 状态机显式请求控制权。FlightController 授权链必要条件之一——
/// 仅当最新通过入口门禁的意图携带该位时才允许 IDLE→ACTIVE；进入 IDLE 时
/// 锁存清零，回 ACTIVE 必须重新显式请求。状态机侧置位在阶段四 Task 7 落地。
constexpr uint32_t kControlIntentReasonRequestControl = 1u << 31;

/// PX4 设定值类型（飞行控制器 → PX4 通信）。
enum class SetpointType : uint8_t {
    kNone = 0,
    kPosition,  ///< 位置设定值（m）
    kVelocity,  ///< 速度设定值（m/s）
    kAttitude,  ///< 姿态设定值
};

/// PX4 设定值（飞行控制器 → PX4 通信线程 → 飞控）。
struct Px4Setpoint {
    MessageHeader header;
    SetpointType type = SetpointType::kNone;
    float x = 0.f;         ///< 位置或速度分量
    float y = 0.f;
    float z = 0.f;
    /// 位置型设定值的速度前馈（局部 NED，m/s；2026-10-19 P3）。
    float feedforward_vx_mps = 0.f;
    float feedforward_vy_mps = 0.f;
    float feedforward_vz_mps = 0.f;
    /// 前馈逐轴有效位（kVelocityValid*）；0=无前馈。速度型必须为 0。
    uint8_t feedforward_valid_mask = 0;
    float yaw_deg = 0.f;       ///< 目标航向；仅 use_yaw=true 时使用
    float yaw_rate_dps = 0.f;  ///< 目标航向角速度；仅 use_yaw_rate=true 时使用
    bool use_yaw = false;      ///< 是否启用航向角控制
    bool use_yaw_rate = false; ///< 是否启用航向角速度控制；不得与 use_yaw 同时为 true
    bool valid = false;        ///< false 时停止发送；刹停/悬停必须生成有效零速度或位置保持值
};

/// PX4 命令 ACK 状态快照（命令窄接口 → 状态机/执行器）。
/// matched_count/timeout_count 单调递增；last_* 为最近一次匹配 ACK/超时。
/// do_set_actuator_* 为 MAV_CMD_DO_SET_ACTUATOR(187) 专用关联字段：捕网复位
/// 等待横跨 S12~S14，期间可能插入 AUTO.LAND(176)/DISARM(400) 的 ACK；若只看
/// 全局 last_*，同一决策拍内后到的 ACK 会覆盖 187，执行器将静默挂死。
/// 执行器必须用本节字段判断 187 的推进/故障；全局字段保留给通用诊断。
struct CommandAckStatus {
    uint64_t matched_count = 0;
    uint64_t timeout_count = 0;
    uint16_t last_command = 0;
    uint8_t last_result = 0;
    uint64_t last_time_ms = 0;
    uint16_t last_timeout_command = 0;
    /// MAV_CMD_DO_SET_ACTUATOR(187) 专用最近终态：避免 176/400 等 ACK 覆盖。
    uint64_t do_set_actuator_ack_count = 0;
    uint8_t do_set_actuator_last_result = 0;
    uint64_t do_set_actuator_last_time_ms = 0;
    uint64_t do_set_actuator_timeout_count = 0;
};

/// 健康状态（健康管理 → 状态机/地面站回传）。
struct HealthStatus {
    MessageHeader header;
    /// 链路健康位：bit0=双目摄像头 bit1=PX4 bit2=地面站电台 bit3=预留（原激光雷达）bit4=图传；1=已注册且健康
    uint32_t link_health_bits = 0;
    /// 设备健康位：bit0=解码器 bit1=NPU/YOLO bit2~3=预留（取消独立电源管理）；1=已注册且健康
    uint32_t device_health_bits = 0;
    /// 数据新鲜度位：与 link_health_bits 同序，0=新鲜 1=超时
    uint32_t data_freshness_bits = 0;
    /// 活动错误位：bit0摄像头 bit1解码器 bit2YOLO bit3PX4 bit4地面站 bit5图传 bit6~7预留
    uint32_t error_bits = 0;
    float cpu_load_pct = std::numeric_limits<float>::quiet_NaN();  ///< 算力板负载；无有效采样时为NaN
    uint32_t timeout_event_count = 0;  ///< 累计进入超时状态的事件次数
};

/// 任务分配的可用状态（65013 v2 回传与 65015 回执共用）。
/// 0~5 与地面站协议一致；只有 ready 表示可被调度。
enum class MissionAvailability : uint8_t {
    kUnavailable = 0,  ///< 未就绪/故障/状态不确定
    kReady = 1,        ///< S2，未执行任务，可分配
    kAssigned = 2,     ///< 已接受分配，S3/S4
    kEngaged = 3,      ///< S5~S10 任务中
    kReturning = 4,    ///< S11~S13 返航/降落
    kTerminal = 5,     ///< S14/S16 或空中任务已结束
};

/// 65015 任务分配回执结果。
enum class MissionAssignmentResult : uint8_t {
    kAccepted = 0,
    kRejected = 1,
    kCancelledOrAborted = 2,
};

/// 65015 任务分配回执原因码（与设计规格 §5.4 一致）。
enum class MissionAssignmentReason : uint8_t {
    kNone = 0,
    kInvalidMessage = 1,
    kUnsupportedVersion = 2,
    kUnsupportedInterceptorType = 3,
    kNotReady = 4,
    kAlreadyAssigned = 5,
    kPreflightFailed = 6,
    kControlDisabled = 7,
    kModeTimeout = 8,
    kArmTimeout = 9,
    kBootstrapTimeout = 10,
    kTakeoffTimeout = 11,
    kCancelledByGcs = 12,
    kTargetWaitTimeout = 13,
    kTargetLost = 14,
    kBatteryLow = 15,
    kBatteryTelemetryInvalid = 16,
    kBatteryConfigUnknown = 17,
    kPx4Disconnected = 18,
    kMissionFault = 19,
    kInternalError = 20,
    kZoneMismatch = 21,  ///< 机载围栏区域版本与分配声明不一致（v3，2026-10-29）
    kFenceViolation = 22,  ///< 围栏越界触发的中止/降落（return/land 动作，2026-10-30）
};

/// 任务分配（地面站 65014 → 状态机）。
/// valid_for_ms 只约束起飞开始，不约束空中任务时长；目标位置仍由 65010 提供。
struct MissionAssignment {
    MessageHeader header;
    uint8_t protocol_version = 1;  ///< 分配协议版本，当前固定 1
    uint32_t assignment_id = 0;    ///< 地面站生成的分配编号
    uint32_t target_id = 0;        ///< 指定目标编号；0 非法
    uint8_t interceptor_type = 0;  ///< 25=捕网 26=火箭，必须等于本机 component
    uint8_t flags = 0;             ///< bit0=授权自动起飞 bit1=取消当前分配
    uint32_t valid_for_ms = 0;     ///< 分配有效期（毫秒）
    /// 65014 v3 扩展（2026-10-29 多边形围栏）：分配时地面站声明的区域编号/版本。
    /// 与机载缓存不一致时按 kZoneMismatch 拒绝（起飞前必须确认同一份围栏）。
    uint16_t zone_id = 0;
    uint16_t zone_version = 0;
    uint64_t received_ms = 0;      ///< 本机收到消息的单调时钟时间
};

/// 任务分配回执（状态机 → 地面站 65015）。
struct MissionAssignmentAck {
    MessageHeader header;
    uint32_t assignment_id = 0;
    uint32_t target_id = 0;
    MissionAssignmentResult result = MissionAssignmentResult::kRejected;
    MissionAssignmentReason reason = MissionAssignmentReason::kNone;
    MissionState mission_state = MissionState::kBoot;
    MissionAvailability availability = MissionAvailability::kUnavailable;
};

/// 任务与拦截状态回传（状态机 → 地面站）。
struct MissionStatus {
    MessageHeader header;
    MissionState state = MissionState::kBoot;  ///< 当前任务状态
    uint64_t state_entered_ms = 0;             ///< 进入当前状态的时间
    uint8_t control_source = 0;                ///< 当前控制来源（与 ControlIntent 一致）
    uint8_t task_phase = 0;                    ///< 任务阶段（实现期定义）
    uint32_t active_warning_bits = 0;          ///< 激活的告警位
    float front_distance_m = std::numeric_limits<float>::quiet_NaN();  ///< 前向障碍距离；无效为 NaN
    bool interception_authorized = false;      ///< 拦截授权
    uint8_t power_status_bits = 0;             ///< 兼容保留；当前电量统一使用PX4动力电池状态
    /// 65013 v2 扩展字段（2026-10-22，多机任务分配）：无分配时 id 为 0。
    uint32_t assignment_id = 0;                ///< 当前分配编号
    uint32_t target_id = 0;                    ///< 当前目标编号
    MissionAvailability availability = MissionAvailability::kUnavailable;  ///< 可用状态
    uint8_t interceptor_type = 0;              ///< 25=捕网 26=火箭；未知为 0
    /// 65013 v3 扩展字段（2026-10-29，多边形围栏）：未装载区域时 id/version 为 0。
    uint16_t zone_id = 0;                      ///< 当前生效围栏区域编号
    uint16_t zone_version = 0;                 ///< 当前生效围栏区域版本
    uint8_t fence_state = 0;                   ///< 见 FenceState；0=正常
};

/// 围栏区域类型（65018 下发）。
enum class FenceZoneType : uint8_t {
    kPolygon = 0,  ///< 多边形（当前唯一支持类型）
};

/// 围栏越界动作（地面站定义，机载执行）。
/// 越界判定成立后由状态机执行对应中止动作，与电池守卫共用中止入口。
enum class FenceAction : uint8_t {
    kHold = 0,    ///< 原地悬停（不返航、不降落，等人工）
    kReturn = 1,  ///< 中止任务并返航 Home（默认）
    kLand = 2,    ///< 原地降落
};

/// 围栏几何状态（65019 回传，供地面站显示）。
enum class FenceState : uint8_t {
    kNormal = 0,      ///< 在区域内（含高度带内）
    kNearBoundary = 1,///< 在区域内但接近边界（告警）
    kViolated = 2,    ///< 已越界（判定成立，按 action 处置）
    kNoZone = 3,      ///< 未装载区域（围栏未启用或缓存缺失）
    kZoneInvalid = 4, ///< 区域数据非法（顶点不足/版本冲突）
};

/// 围栏多边形顶点（WGS84，65018 线格式与 zone.json 缓存共用）。
struct FenceVertex {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
};

/// 围栏区域（地面站 65018 → 状态机；机载缓存为 config/zone.json）。
/// 顶点为 WGS84 经纬度，机载按当前 EKF 锚点转换为局部 NED 后做几何判定。
struct FenceZone {
    MessageHeader header;
    uint8_t protocol_version = 1;    ///< 区域协议版本，当前固定 1
    uint16_t zone_id = 0;            ///< 区域编号（0=非法）
    uint16_t version = 0;            ///< 区域内容版本；内容变更即递增
    FenceZoneType type = FenceZoneType::kPolygon;
    FenceAction action = FenceAction::kReturn;  ///< 越界动作
    uint8_t altitude_reference = 0;  ///< 0=AMSL 绝对高度 1=Home 相对高度
    double alt_min_m = 0.0;          ///< 高度带下限（m）
    double alt_max_m = 0.0;          ///< 高度带上限（m）；0=未限制，用配置默认
    std::vector<FenceVertex> vertices;  ///< 多边形顶点（WGS84，≥3 个）
    uint64_t received_ms = 0;        ///< 本机收到的单调时钟时间
    /// 顶点量化参考点（1e7 度）。机载原样保存并随 65022 回传，使地面站能与
    /// 自己下发的定义逐字节比对，避免"反投影再投影"带来的漂移。
    int32_t ref_lat_1e7 = 0;
    int32_t ref_lon_1e7 = 0;
};

/// 局部 NED 二维点（围栏几何用）。
struct Vec2Ned {
    double north_m = 0.0;
    double east_m = 0.0;
};

/// 已转换为局部 NED 的围栏多边形快照（状态机 → 飞行控制器钳制）。
/// 顶点顺序不限自交（不做凸性检查）；坐标系原点与位置意图一致（局部 NED）。
struct FencePolygonNed {
    uint16_t zone_id = 0;
    uint16_t version = 0;
    FenceAction action = FenceAction::kHold;
    double alt_min_m = 0.0;  ///< 相对 Home 高度下限（m）
    double alt_max_m = 0.0;  ///< 相对 Home 高度上限（m）；0=用配置默认
    std::vector<Vec2Ned> vertices;
    bool valid() const { return vertices.size() >= 3; }
};

/// 机载围栏多边形只读来源（状态机实现，飞行控制器注入消费）。
/// 供飞行控制器把位置意图钳制到区域内；返回空/无效快照时回退圆形围栏。
class IFencePolygonSource {
public:
    virtual ~IFencePolygonSource() = default;
    /// 线程安全：可被控制周期线程与决策线程并发调用。
    virtual FencePolygonNed LatestFencePolygon() const = 0;
};

/// 围栏状态回传（状态机 → 地面站 65019）。
struct FenceStatus {
    MessageHeader header;
    uint16_t zone_id = 0;
    uint16_t version = 0;
    FenceState state = FenceState::kNoZone;
    double distance_to_boundary_m = 0.0;  ///< 距最近边界距离；区域内为正，区域外为负
    uint32_t violation_duration_ms = 0;   ///< 连续越界持续时长；0=未越界
    double local_north_m = 0.0;           ///< 飞机当前局部 NED 位置
    double local_east_m = 0.0;
    double local_down_m = 0.0;
    bool action_pending = false;          ///< 越界动作是否待执行（已中止任务）
};

/// 地面站捕网发射指令（65016，地面站 → 指定飞机；台架/测试用）。
/// 只在 `actuator.gcs_fire_enabled=true` 且“已落地 + 未解锁”时执行，
/// 与 `interception.enabled` 的自动拦截发射互斥（见 GcsNetFire）。
struct NetFireCommand {
    MessageHeader header;
    uint8_t protocol_version = 1;   ///< 发射协议版本，当前固定 1
    uint32_t command_id = 0;        ///< 地面站生成的指令编号；65017 原样带回
    uint8_t flags = 0;              ///< bit0=确认发射（必须置位）
    uint64_t received_ms = 0;       ///< 本机收到消息的单调时钟时间
};

/// 65017 捕网发射回执结果。
enum class NetFireResult : uint8_t {
    kAccepted = 0,   ///< 已入队，执行器开始触发/复位时序
    kCompleted = 1,  ///< 触发-复位周期完成，执行器回到待机
    kRejected = 2,   ///< 未执行，具体原因见 reason
};

/// 65017 捕网发射回执原因码。
enum class NetFireReason : uint8_t {
    kNone = 0,
    kInvalidMessage = 1,        ///< 帧长/字段非法
    kUnsupportedVersion = 2,    ///< 协议版本不支持
    kConfirmationMissing = 3,   ///< flags 未置确认位
    kDuplicateCommand = 4,      ///< 同一 command_id 已处理（回执重放）
    kCommandStale = 5,          ///< 指令在队列中滞留过久
    kDisabled = 6,              ///< actuator.gcs_fire_enabled=false
    kFlightStateStale = 7,      ///< 无新鲜飞行状态，无法确认落地/未解锁
    kArmed = 8,                 ///< 已解锁，拒绝发射
    kNotLanded = 9,             ///< 未落地，拒绝发射
    kActuatorUnavailable = 10,  ///< 未注入命令接口 / 执行器未创建
    kActuatorFaulted = 11,      ///< 执行器终态故障
    kActuatorBusy = 12,         ///< 执行器未回到 Ready
    kCooldown = 13,             ///< 距上一次成功触发的间隔小于冷却时间
    kNoRemainingShots = 14,     ///< 发数已耗尽（单发为 1）
    kSendFailed = 15,           ///< 触发命令入队失败
    kInternalError = 100,
};

/// 地面站捕网发射回执（飞机 → 地面站 65017）。
struct NetFireAck {
    MessageHeader header;
    uint32_t command_id = 0;
    NetFireResult result = NetFireResult::kRejected;
    NetFireReason reason = NetFireReason::kNone;
    /// 执行器状态快照：0=Ready 1=Busy（触发/复位时序中） 2=Faulted。
    uint8_t actuator_state = 0;
};

/// 地面站任务指令类型（65020 MISSION_COMMAND）。
/// 与 65014 的区别：65014 是“分配任务”，65020 是“对已起飞任务的即时指令”。
enum class MissionCommandType : uint8_t {
    kReturnHome = 1,  ///< 主动返航：中止当前任务并返回 Home（当前唯一指令）
};

/// 65021 任务指令回执结果。
enum class MissionCommandResult : uint8_t {
    kAccepted = 0,   ///< 已受理，已开始执行（如已切到 S12 返航）
    kCompleted = 1,  ///< 指令目标状态已达成（如本来就在返航/降落）
    kRejected = 2,   ///< 未执行，原因见 reason
};

/// 65021 任务指令回执原因码。
enum class MissionCommandReason : uint8_t {
    kNone = 0,
    kInvalidMessage = 1,       ///< 帧长/字段非法
    kUnsupportedVersion = 2,   ///< 协议版本不支持
    kUnsupportedCommand = 3,   ///< 指令类型不支持
    kDuplicateCommand = 4,     ///< 同一 command_id 已处理（回执重放，不重复执行）
    kNotAirborne = 5,          ///< 不在可执行的空中任务状态（未起飞/已落地/故障）
    kAlreadyReturning = 6,     ///< 已在返航或降落流程
    kMissionFault = 7,         ///< 执行器故障等任务级故障，交 PX4/人工处置
    kInternalError = 20,
};

/// 地面站任务指令（65020，地面站 → 指定飞机）。
/// 只对指定飞机点对点发送；机载按状态判定，空中任务态才会真正中止返航。
struct MissionCommand {
    MessageHeader header;
    uint8_t protocol_version = 1;   ///< 指令协议版本，当前固定 1
    uint32_t command_id = 0;        ///< 地面站生成的指令编号；65021 原样带回
    MissionCommandType type = MissionCommandType::kReturnHome;  ///< 指令类型
    uint64_t received_ms = 0;       ///< 本机收到消息的单调时钟时间
};

/// 任务指令回执（飞机 → 地面站 65021）。
struct MissionCommandAck {
    MessageHeader header;
    uint8_t protocol_version = 1;
    uint32_t command_id = 0;                                   ///< 回执对应指令
    MissionCommandType type = MissionCommandType::kReturnHome; ///< 回执对应类型
    MissionCommandResult result = MissionCommandResult::kRejected;
    MissionCommandReason reason = MissionCommandReason::kNone;
    MissionState mission_state = MissionState::kBoot;  ///< 处理后的任务状态
};

/// 目标估计状态（感知融合/目标估计 → 状态机与控制）。
struct TargetState {
    MessageHeader header;
    uint32_t target_id = 0;
    float pos_x_m = 0.f;       ///< 目标位置（坐标系见 frame_id）
    float pos_y_m = 0.f;
    float pos_z_m = 0.f;       ///< 垂直状态无效时为NaN
    float vel_x_mps = 0.f;     ///< 目标速度
    float vel_y_mps = 0.f;
    float vel_z_mps = 0.f;     ///< 垂直状态无效时为NaN
    /// 逐轴速度可用位（kVelocityValid*）；0=不可用于前馈。目标过期时清零。
    uint8_t velocity_valid_mask = 0;
    float confidence = 0.f;    ///< 估计置信度 [0,1]
    uint64_t last_observed_ms = 0;  ///< 最近有效观测时间
    bool valid = false;        ///< 水平估计是否新鲜有效；需要三维的消费者还必须检查Z/VZ有限
};

/// Topic 名称常量。全工程统一引用，禁止字符串散落。
/// 队列容量与满队列策略见 docs/数据接口文档.md §5。
namespace topics {
inline constexpr char kCameraStream[] = "camera_stream";      ///< EncodedFrame
inline constexpr char kDecodedFrame[] = "decoded_frame";      ///< video::FrameHandle
inline constexpr char kDecodedFrameLeft[] = "decoded_frame_left";    ///< video::FrameHandle（双目拆分左目=SBS右半=物理左目=测距基准图）
inline constexpr char kDecodedFrameRight[] = "decoded_frame_right";  ///< video::FrameHandle（双目拆分右目=SBS左半=物理右目=测距辅助图）
inline constexpr char kDetection[] = "detection";             ///< DetectionResult（基准图检测框）
inline constexpr char kStereoTargetDistance[] = "stereo_target_distance";  ///< StereoTargetDistance
inline constexpr char kVisualTarget[] = "visual_target";      ///< VisualTargetObservation
inline constexpr char kVisualTargetStatus[] = "visual_target_status";  ///< VisualTargetStatus
inline constexpr char kOpticalFlow[] = "optical_flow";        ///< OpticalFlowResult
inline constexpr char kLaserRange[] = "laser_range";          ///< LaserRangeSample
inline constexpr char kFusedTarget[] = "fused_target";        ///< TargetState（融合输出）
inline constexpr char kEstimatedTarget[] = "estimated_target";///< TargetState（估计输出）
inline constexpr char kGroundTarget[] = "ground_target";      ///< GroundStationTarget
inline constexpr char kFlightState[] = "flight_state";        ///< FlightStateSnapshot
inline constexpr char kControlIntent[] = "control_intent";    ///< ControlIntent
inline constexpr char kPx4Setpoint[] = "px4_setpoint";        ///< Px4Setpoint
inline constexpr char kMissionStatus[] = "mission_status";    ///< MissionStatus
inline constexpr char kMissionAssignment[] = "mission_assignment";        ///< MissionAssignment（65014）
inline constexpr char kMissionAssignmentAck[] = "mission_assignment_ack"; ///< MissionAssignmentAck（65015）
inline constexpr char kNetFireCommand[] = "net_fire_command";             ///< NetFireCommand（65016）
inline constexpr char kNetFireAck[] = "net_fire_ack";                     ///< NetFireAck（65017）
inline constexpr char kFenceZone[] = "fence_zone";                        ///< FenceZone（65018/缓存）
inline constexpr char kFenceStatus[] = "fence_status";
/// 围栏区域回读（状态机 → 地面站链接，编码为 65022）。
inline constexpr char kFenceZoneReport[] = "fence_zone_report";                    ///< FenceStatus（65019）
inline constexpr char kMissionCommand[] = "mission_command";              ///< MissionCommand（65020）
inline constexpr char kMissionCommandAck[] = "mission_command_ack";       ///< MissionCommandAck（65021）
inline constexpr char kHealthStatus[] = "health_status";      ///< HealthStatus
inline constexpr char kAnnotatedFrame[] = "annotated_frame";  ///< video::FrameHandle
}  // namespace topics

}  // namespace drone::common
