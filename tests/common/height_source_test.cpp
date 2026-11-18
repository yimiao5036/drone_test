#include "common/types.h"

#include <gtest/gtest.h>

#include <optional>

namespace drone {
namespace {

using common::AboveTakeoffM;
using common::EkfAboveTakeoffM;
using common::FlightStateSnapshot;
using common::HeightSource;
using common::RadarAlive;
using common::RadarAglM;
using common::RadarGateParams;
using common::RadarRejectReason;

// ---------------------------------------------------------------------------
// P5「雷达接入」纯类型层（2026-11-18）：单一高度出口 `AboveTakeoffM()` 的退化链
// （有效雷达 AGL → 气压 AGL → EKF 差分）与 `RadarAglM()` 的各类拒绝原因。
//
// 全部是头文件里的纯函数，不启线程、不依赖 Topic/状态机，因此可以逐条钉死语义：
// - gate.enabled=false 时必须按 P5 之前的行为只用 EKF 差分（气压只做交叉校验）；
// - 雷达不可用时必须返回 nullopt 并给出原因，**绝不能用 0 / max_distance 冒充**；
// - signal_quality 的 0/1/2~100 三种编码按 PX4 出口实现区分（0=没有质量指标）。
// ---------------------------------------------------------------------------

/// 基线快照：EKF 相对高度 5.0m（Home 局部 z=0、local_z=-5）、气压 AGL 7.0m、
/// 雷达新鲜有效 3.0m。用例只改自己关心的那一路，其余保持"三路都可用"，
/// 这样任何退化链错误都能立刻暴露（例如 disabled 时错误地选了气压）。
FlightStateSnapshot MakeFlight() {
    FlightStateSnapshot state;
    state.local_position_valid = true;
    state.home_local_z_valid = true;
    state.home_local_z_m = 0.f;
    state.local_z_m = -5.f;
    state.baro_altitude_valid = true;
    state.baro_takeoff_ref_valid = true;
    state.baro_takeoff_ref_amsl_m = 100.f;
    state.baro_amsl_m = 107.f;  // 气压 AGL = 107 - 100 = 7.0m
    state.distance_valid = true;
    state.distance_time_valid = true;
    state.distance_time_ms = 1000;
    state.distance_m = 3.f;
    state.distance_min_m = 0.1f;
    state.distance_max_m = 8.f;
    state.distance_orientation = common::kDistanceOrientationDownward;
    state.distance_signal_quality = 0;
    return state;
}

RadarGateParams MakeGate() {
    RadarGateParams gate;
    gate.enabled = true;
    gate.min_signal_quality = 30;
    gate.stale_ms = 200;
    return gate;
}

// 回滚开关：radar.enabled=false 时必须与 P5 之前逐位一致——决策高度 = EKF 差分，
// 气压只做交叉校验（不能悄悄改成"baro 优先"）。
TEST(RadarHeightSourceTest, DisabledGateUsesEkfDifferentialNotBaro) {
    FlightStateSnapshot state = MakeFlight();
    ASSERT_TRUE(common::BaroAboveTakeoffM(state).has_value()) << "前提：气压 AGL 可用";
    ASSERT_TRUE(EkfAboveTakeoffM(state).has_value()) << "前提：EKF 差分可用";

    RadarGateParams gate = MakeGate();
    gate.enabled = false;  // 回滚开关
    const auto height = AboveTakeoffM(state, 1000, gate);

    ASSERT_TRUE(height.has_value());
    EXPECT_EQ(height->source, HeightSource::kEkf);
    EXPECT_DOUBLE_EQ(height->agl_m, 5.0) << "必须取 EKF 差分 5.0m";
    EXPECT_NE(height->agl_m, 7.0) << "不得取气压 AGL 7.0m";
    EXPECT_EQ(height->radar_reject, RadarRejectReason::kDisabled);
}

// 同一回滚语义的另一半：EKF 不可用时返回 nullopt，**不得**退到气压
// （否则 gate.enabled=false 的行为就不再是 P5 之前那一条路径了）。
TEST(RadarHeightSourceTest, DisabledGateDoesNotFallBackToBaro) {
    FlightStateSnapshot state = MakeFlight();
    state.local_position_valid = false;  // EKF 差分不可用
    ASSERT_TRUE(common::BaroAboveTakeoffM(state).has_value()) << "前提：气压 AGL 仍可用";

    RadarGateParams gate = MakeGate();
    gate.enabled = false;
    EXPECT_FALSE(AboveTakeoffM(state, 1000, gate).has_value())
        << "回滚路径只认 EKF 差分，气压只做交叉校验，不得当选高度源";
}

// 雷达新鲜有效（orientation=25、读数在 (min,max) 内、quality=0）时优先于气压/EKF。
TEST(RadarHeightSourceTest, FreshRadarWinsOverBaroAndEkf) {
    const FlightStateSnapshot state = MakeFlight();
    const auto height = AboveTakeoffM(state, 1000, MakeGate());

    ASSERT_TRUE(height.has_value());
    EXPECT_EQ(height->source, HeightSource::kRadar);
    EXPECT_DOUBLE_EQ(height->agl_m, 3.0) << "agl_m 必须等于 distance_m";
    EXPECT_EQ(height->radar_reject, RadarRejectReason::kNone)
        << "雷达被用上时 reason 必须是 kNone";
}

// 过期（now - distance_time_ms > stale_ms）→ 退到气压 AGL，并如实给出 kStale。
TEST(RadarHeightSourceTest, StaleRadarFallsBackToBaro) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_time_ms = 1000;

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1201, MakeGate(), &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kStale);

    const auto height = AboveTakeoffM(state, 1201, MakeGate());
    ASSERT_TRUE(height.has_value());
    EXPECT_EQ(height->source, HeightSource::kBaro);
    EXPECT_DOUBLE_EQ(height->agl_m, 7.0);
    EXPECT_EQ(height->radar_reject, RadarRejectReason::kStale);
}

// stale_ms 边界：age == stale_ms 仍可用；age == stale_ms + 1 才是过期。
// 时钟未初始化（now < 到达时刻）的负差值按 0 处理，同样可用。
TEST(RadarHeightSourceTest, RadarAgeBoundaryIsInclusive) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_time_ms = 1000;

    EXPECT_TRUE(RadarAglM(state, 1200, MakeGate()).has_value())
        << "age==stale_ms(200ms) 仍可用";
    EXPECT_DOUBLE_EQ(*RadarAglM(state, 1200, MakeGate()), 3.0);

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1201, MakeGate(), &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kStale) << "age==stale_ms+1 必须过期";

    EXPECT_TRUE(RadarAglM(state, 900, MakeGate()).has_value())
        << "now 早于到达时刻时按 age=0，不应吃负差值（时钟未初始化的一拍）";
}

// signal_quality==1 是 MAVLink 的 invalid 专用值（PX4 把驱动 quality==0 映射成 1），
// 必须拒绝；随后退化到气压。
TEST(RadarHeightSourceTest, SignalQualityInvalidIsRejected) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_signal_quality = common::kDistanceSignalQualityInvalid;

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, MakeGate(), &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kSignalInvalid);

    const auto height = AboveTakeoffM(state, 1000, MakeGate());
    ASSERT_TRUE(height.has_value());
    EXPECT_EQ(height->source, HeightSource::kBaro);
    EXPECT_EQ(height->radar_reject, RadarRejectReason::kSignalInvalid);
}

// 量程两端都算无效：distance_m <= min 或 >= max（边界值本身不可用），
// 这正是"超量程当落地"必须被挡住的那条。
TEST(RadarHeightSourceTest, OutOfRangeRejectedAtBothEnds) {
    FlightStateSnapshot state = MakeFlight();
    RadarGateParams gate = MakeGate();
    RadarRejectReason reason = RadarRejectReason::kNone;

    state.distance_m = state.distance_max_m;  // == max（8.0m）
    EXPECT_FALSE(RadarAglM(state, 1000, gate, &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kOutOfRange);

    state.distance_m = state.distance_min_m;  // == min（0.1m）
    reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, gate, &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kOutOfRange);

    state.distance_m = 0.05f;  // < min（地面盲区读数，PX4 可能原样透传）
    reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, gate, &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kOutOfRange);

    // 边界内侧最近处仍可用（确认上面的失败不是"所有读数都拒"）。
    // distance_m 是 float，7.9f 转 double 与字面量 7.9 有 ~1e-7 的表示差，
    // 故用 EXPECT_NEAR 而不是 EXPECT_DOUBLE_EQ。
    state.distance_m = 7.9f;
    EXPECT_NEAR(*RadarAglM(state, 1000, gate), 7.9, 1e-6);
}

// 驱动没给 max（max<=0）时跳过量程判据：不能把"没有量程信息"当成"超量程"。
TEST(RadarHeightSourceTest, RangeCheckSkippedWhenDriverReportsNoMax) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_min_m = 0.f;
    state.distance_max_m = 0.f;  // 驱动未上报量程
    state.distance_m = 100.f;
    EXPECT_DOUBLE_EQ(*RadarAglM(state, 1000, MakeGate()), 100.0);
}

// orientation 不是向下（25）→ 拒绝；P5 只用向下安装的测距。
TEST(RadarHeightSourceTest, WrongOrientationIsRejected) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_orientation = 0;  // ROTATION_NONE

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, MakeGate(), &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kWrongOrientation);
}

// 从未收到过 DISTANCE_SENSOR（未装雷达/驱动没起来）→ kNoData，退到气压。
TEST(RadarHeightSourceTest, NoDataIsRejectedWhenNoDistanceSensorEverSeen) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_time_valid = false;

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, MakeGate(), &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kNoData);

    const auto height = AboveTakeoffM(state, 1000, MakeGate());
    ASSERT_TRUE(height.has_value());
    EXPECT_EQ(height->source, HeightSource::kBaro);
    EXPECT_EQ(height->radar_reject, RadarRejectReason::kNoData);
}

// 时间戳新鲜但本帧明确无测量（current_distance=0 / UINT16_MAX 等）→ kNoMeasurement。
TEST(RadarHeightSourceTest, NoMeasurementIsRejectedWhenFrameInvalid) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_valid = false;

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, MakeGate(), &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kNoMeasurement);
}

// signal_quality 的三种编码必须分开处理：
// 0 = 驱动没有质量指标（可用，不受门槛约束）；1 = invalid；2~100 才受门槛约束。
TEST(RadarHeightSourceTest, QualityThresholdOnlyAppliesToMeasuredQuality) {
    FlightStateSnapshot state = MakeFlight();
    RadarGateParams gate = MakeGate();
    gate.min_signal_quality = 50;  // 门槛 50
    RadarRejectReason reason = RadarRejectReason::kNone;

    state.distance_signal_quality = 0;  // 没有质量指标
    EXPECT_DOUBLE_EQ(*RadarAglM(state, 1000, gate), 3.0)
        << "quality==0 是『驱动没有质量指标』，不能被门槛判差";

    state.distance_signal_quality = 40;  // 实测 40 < 50
    EXPECT_FALSE(RadarAglM(state, 1000, gate, &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kQualityLow);

    state.distance_signal_quality = 2;  // 2~100 的实测最小值也受门槛约束
    reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, gate, &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kQualityLow);

    state.distance_signal_quality = 50;  // == 门槛可用
    EXPECT_DOUBLE_EQ(*RadarAglM(state, 1000, gate), 3.0);

    // 门槛置 0 = 关闭质量判据：50 以下的实测质量也放行（但 1 仍是 invalid）。
    gate.min_signal_quality = 0;
    state.distance_signal_quality = 2;
    EXPECT_DOUBLE_EQ(*RadarAglM(state, 1000, gate), 3.0);
    state.distance_signal_quality = common::kDistanceSignalQualityInvalid;
    reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, gate, &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kSignalInvalid);
}

// 三条高度源全部不可用 → nullopt。**绝不能返回 0**（0 会被上层当成"已落地"）。
TEST(RadarHeightSourceTest, AllSourcesUnavailableReturnsNulloptNotZero) {
    FlightStateSnapshot state;  // 全空：无局部位置/基准/气压/测距
    const RadarGateParams gate = MakeGate();

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, gate, &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kNoData) << "没有时间戳时先判 kNoData";

    const auto height = AboveTakeoffM(state, 1000, gate);
    EXPECT_FALSE(height.has_value())
        << "三条全不可用必须返回 nullopt，调用方按『高度不可用』处理";
}

// ekf_fallback 参数：传了就用调用方的基准，不传才用 EkfAboveTakeoffM(state)。
// 这样降落段可以传入用 HomeLocalNed() 兜底算出的 base_z，避免两处基准不一致。
TEST(RadarHeightSourceTest, EkfFallbackParameterTakesPrecedence) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_time_valid = false;  // 雷达无数据
    state.baro_altitude_valid = false;  // 气压无数据

    // 不传 fallback：用 EkfAboveTakeoffM(state) = 0 - (-5) = 5.0m。
    const auto from_state = AboveTakeoffM(state, 1000, MakeGate());
    ASSERT_TRUE(from_state.has_value());
    EXPECT_EQ(from_state->source, HeightSource::kEkf);
    EXPECT_DOUBLE_EQ(from_state->agl_m, 5.0);
    EXPECT_EQ(from_state->radar_reject, RadarRejectReason::kNoData);

    // 传 fallback=2.5：必须用它，而不是 state 里的 5.0m。
    const auto from_fallback = AboveTakeoffM(state, 1000, MakeGate(), 2.5);
    ASSERT_TRUE(from_fallback.has_value());
    EXPECT_EQ(from_fallback->source, HeightSource::kEkf);
    EXPECT_DOUBLE_EQ(from_fallback->agl_m, 2.5);

    // 回滚路径同样认这个参数：EKF 快照本身不可用时也能用调用方基准。
    state.local_position_valid = false;
    RadarGateParams disabled = MakeGate();
    disabled.enabled = false;
    const auto rollback = AboveTakeoffM(state, 1000, disabled, 2.5);
    ASSERT_TRUE(rollback.has_value());
    EXPECT_EQ(rollback->source, HeightSource::kEkf);
    EXPECT_DOUBLE_EQ(rollback->agl_m, 2.5);
    EXPECT_EQ(rollback->radar_reject, RadarRejectReason::kDisabled);
}

// 雷达不可用且气压缺失时继续退化到 EKF，并保留雷达拒绝原因（供 S13a 日志）。
TEST(RadarHeightSourceTest, FallsThroughToEkfWhenBaroMissing) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_time_ms = 0;  // age = 1000ms > 200ms
    state.baro_altitude_valid = false;

    const auto height = AboveTakeoffM(state, 1000, MakeGate());
    ASSERT_TRUE(height.has_value());
    EXPECT_EQ(height->source, HeightSource::kEkf);
    EXPECT_DOUBLE_EQ(height->agl_m, 5.0);
    EXPECT_EQ(height->radar_reject, RadarRejectReason::kStale);
}

// ---------------------------------------------------------------------------
// RadarAlive：起飞前门禁的判据（2026-11-18 语义修正）。
// 语义 = "收到过数据且时效内"，**不判读数是否在量程内**：落在地上的下视测距
// 必然在盲区（TFmini-S 最小 0.4m，装机离地 0.1~0.2m），所以预检只能问
// "链路活着没有"。
// ---------------------------------------------------------------------------

TEST(RadarAliveTest, AgeAtLimitIsStillAlive) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_time_ms = 1000;

    EXPECT_TRUE(RadarAlive(state, 1200, 200)) << "age==alive_ms 仍算活着";
    EXPECT_FALSE(RadarAlive(state, 1201, 200)) << "age==alive_ms+1 算断流";
    EXPECT_TRUE(RadarAlive(state, 900, 200))
        << "now 早于到达时刻按 age=0（时钟未初始化的一拍）";
}

TEST(RadarAliveTest, NeverSeenSensorIsNotAlive) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_time_valid = false;
    EXPECT_FALSE(RadarAlive(state, 1000, 200));
}

// 与 RadarAglM 的关键区别：本帧"无测量"（盲区/驱动判无效）不代表链路断——
// 预检门禁必须仍判活着，否则地面上永远无法起飞；高度出口则必须拒绝它。
TEST(RadarAliveTest, InvalidMeasurementDoesNotHideAliveLink) {
    FlightStateSnapshot state = MakeFlight();
    state.distance_valid = false;  // 本帧无有效测量（地面上就在盲区）

    EXPECT_TRUE(RadarAlive(state, 1000, 200))
        << "时间戳新鲜即算雷达在报数，不判 distance_valid";

    RadarRejectReason reason = RadarRejectReason::kNone;
    EXPECT_FALSE(RadarAglM(state, 1000, MakeGate(), &reason).has_value());
    EXPECT_EQ(reason, RadarRejectReason::kNoMeasurement)
        << "同一帧在高度出口必须因『无测量』被拒（两种口径的分工）";
}

}  // namespace
}  // namespace drone
