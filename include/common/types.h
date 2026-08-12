/**
 * @file types.h
 * @brief 模块间公共消息类型与 Topic 名称常量
 *
 * 属于 drone/common 模块。所有跨模块传递的数据类型集中在此定义，
 * 各部件接口（ICameraReceiver、IVideoDecoder 等）只依赖本文件中的类型，
 * 保证并行开发时数据接口一致。
 *
 * 与 docs/数据接口文档.md、docs/通信与数据定义.md 对应：
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

/// H.265 码流块（摄像头采集 → 视频解码器）。
/// 骨架期使用字节容器；大数据零拷贝优化在实现期按性能实测决定。
struct EncodedFrame {
    MessageHeader header;
    uint32_t stream_sequence = 0;  ///< 码流序号（解码侧用于乱序/丢包检测）
    uint64_t capture_time_ms = 0;  ///< 摄像头时间；不可用时为 0（不得伪造）
    bool is_key_frame = false;     ///< 是否关键帧（I 帧）
    std::vector<uint8_t> data;     ///< 码流字节（含帧边界，分包处理见实现协议）
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

/// 激光雷达单点距离（雷达 → 感知融合）。表达机头前向单点距离，
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
    uint32_t target_id = 0;
    int32_t latitude_1e7 = 0;     ///< WGS-84 纬度 ×1e7
    int32_t longitude_1e7 = 0;    ///< WGS-84 经度 ×1e7
    int32_t altitude_mm = 0;      ///< 高度（毫米），基准见 alt_reference
    uint8_t alt_reference = 0;    ///< 0=未知 1=MSL 2=AGL 3=WGS84 椭球
    float velocity_north_mps = 0.f;
    float velocity_east_mps = 0.f;
    float velocity_down_mps = 0.f;
    float heading_deg = 0.f;      ///< 相对真北；无效为 NaN
    float horizontal_accuracy_m = 0.f;  ///< 水平定位精度
    float vertical_accuracy_m = 0.f;    ///< 垂直定位精度
    /// 可选字段有效标志：bit0=vN bit1=vE bit2=vD bit3=heading
    /// bit4=horizontal_accuracy bit5=vertical_accuracy
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
    bool landed = true;
    uint8_t flight_mode = 0;      ///< PX4 模式枚举（实现期按 1.11.3 映射）
    bool gps_fix = false;
    int32_t latitude_1e7 = 0;     ///< 全局位置 WGS-84 ×1e7
    int32_t longitude_1e7 = 0;
    int32_t altitude_mm = 0;
    float local_x_m = 0.f;        ///< 局部位置（坐标系见 frame_id）
    float local_y_m = 0.f;
    float local_z_m = 0.f;
    float vx_mps = 0.f;           ///< 速度（与局部位置同坐标系）
    float vy_mps = 0.f;
    float vz_mps = 0.f;
    float roll_rad = 0.f;         ///< 姿态（机体欧拉角）
    float pitch_rad = 0.f;
    float yaw_rad = 0.f;
    float baro_alt_m = 0.f;       ///< 气压高度及其参考
    bool home_valid = false;
    int32_t home_lat_1e7 = 0;
    int32_t home_lon_1e7 = 0;
    float battery_voltage_v = 0.f;      ///< 动力电池电压
    float battery_current_a = 0.f;      ///< 动力电池电流
    float battery_remaining_pct = 0.f;  ///< 剩余电量百分比
    bool rc_connected = false;          ///< 遥控器链路和人工接管状态
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
    float yaw_deg = 0.f;
    float max_speed_mps = 0.f;   ///< 速度限制；0=不限制
    uint32_t reason_code = 0;    ///< 产生原因编码（见 docs/数据接口文档.md 附录）
    uint64_t valid_until_ms = 0; ///< 有效期截止（单调时钟）
};

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
    float yaw_deg = 0.f;   ///< 目标航向
    float yaw_rate_dps = 0.f;  ///< 目标航向角速度
    bool valid = false;    ///< 无效设定值（如刹停完成）也需发送，避免过期指令
};

/// 健康状态（健康管理 → 状态机/地面站回传）。
struct HealthStatus {
    MessageHeader header;
    /// 链路健康位：bit0=摄像头 bit1=PX4 bit2=地面站电台 bit3=激光雷达 bit4=图传
    uint32_t link_health_bits = 0;
    /// 设备健康位：bit0=解码器 bit1=NPU/YOLO bit2=电源A bit3=电源B
    uint32_t device_health_bits = 0;
    /// 数据新鲜度位：与 link_health_bits 同序，0=新鲜 1=超时
    uint32_t data_freshness_bits = 0;
    uint32_t error_bits = 0;      ///< 错误位（实现期按模块定义）
    float cpu_load_pct = 0.f;     ///< 算力板负载
};

/// 任务与拦截状态回传（状态机 → 地面站）。
struct MissionStatus {
    MessageHeader header;
    MissionState state = MissionState::kBoot;  ///< 当前任务状态
    uint64_t state_entered_ms = 0;             ///< 进入当前状态的时间
    uint8_t control_source = 0;                ///< 当前控制来源（与 ControlIntent 一致）
    uint8_t task_phase = 0;                    ///< 任务阶段（实现期定义）
    uint32_t active_warning_bits = 0;          ///< 激活的告警位
    float front_distance_m = 0.f;              ///< 前向障碍距离；无效为 NaN
    bool interception_authorized = false;      ///< 拦截授权
    uint8_t power_status_bits = 0;             ///< bit0=电源A可用 bit1=电源B可用
};

/// 目标估计状态（感知融合/目标估计 → 状态机与控制）。
struct TargetState {
    MessageHeader header;
    uint32_t target_id = 0;
    float pos_x_m = 0.f;       ///< 目标位置（坐标系见 frame_id）
    float pos_y_m = 0.f;
    float pos_z_m = 0.f;
    float vel_x_mps = 0.f;     ///< 目标速度
    float vel_y_mps = 0.f;
    float vel_z_mps = 0.f;
    float confidence = 0.f;    ///< 估计置信度 [0,1]
    uint64_t last_observed_ms = 0;  ///< 最近有效观测时间
    bool valid = false;        ///< 估计是否可被控制使用
};

/// Topic 名称常量。全工程统一引用，禁止字符串散落。
/// 队列容量与满队列策略见 docs/数据接口文档.md §5。
namespace topics {
inline constexpr char kCameraStream[] = "camera_stream";      ///< EncodedFrame
inline constexpr char kDecodedFrame[] = "decoded_frame";      ///< video::FrameHandle
inline constexpr char kDetection[] = "detection";             ///< DetectionResult
inline constexpr char kOpticalFlow[] = "optical_flow";        ///< OpticalFlowResult
inline constexpr char kLaserRange[] = "laser_range";          ///< LaserRangeSample
inline constexpr char kFusedTarget[] = "fused_target";        ///< TargetState（融合输出）
inline constexpr char kEstimatedTarget[] = "estimated_target";///< TargetState（估计输出）
inline constexpr char kGroundTarget[] = "ground_target";      ///< GroundStationTarget
inline constexpr char kFlightState[] = "flight_state";        ///< FlightStateSnapshot
inline constexpr char kControlIntent[] = "control_intent";    ///< ControlIntent
inline constexpr char kPx4Setpoint[] = "px4_setpoint";        ///< Px4Setpoint
inline constexpr char kMissionStatus[] = "mission_status";    ///< MissionStatus
inline constexpr char kHealthStatus[] = "health_status";      ///< HealthStatus
inline constexpr char kAnnotatedFrame[] = "annotated_frame";  ///< video::FrameHandle
}  // namespace topics

}  // namespace drone::common
