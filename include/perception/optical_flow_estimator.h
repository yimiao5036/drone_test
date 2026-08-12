/**
 * @file optical_flow_estimator.h
 * @brief 光流估计部件接口（IOpticalFlowEstimator）
 *
 * 属于 drone/perception 模块。职责：订阅相邻解码帧，估计光流
 * （关联相邻帧序号与时间间隔），输出质量指标；低纹理、强压缩、
 * 运动模糊或光照异常时降低质量状态。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，光流算法在实现期接入（可选用现成库）。
 * - OpticalFlowEstimatorStub 为骨架占位实现：生命周期可运行，
 *   业务方法记录"未实现"节流日志并返回默认值。
 *
 * 数据流：common::Topic<FrameHandle> ──► IOpticalFlowEstimator ──► common::Topic<OpticalFlowResult>
 * 可替换边界：光流算法。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"
#include "video/video_frame.h"

namespace drone::perception {

/// 光流估计部件抽象接口。
class IOpticalFlowEstimator {
public:
    virtual ~IOpticalFlowEstimator() = default;

    // ---- 生命周期 ----
    /// 启动光流估计（启动消费线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止光流估计；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;

    // ---- 输入 ----
    /// 绑定解码帧输入主题（IVideoDecoder::FrameOutput()）。
    virtual void SetInput(common::Topic<video::FrameHandle>& input) = 0;

    // ---- 输出 ----
    /// 光流结果输出主题：common::OpticalFlowResult。
    virtual common::Topic<common::OpticalFlowResult>& FlowOutput() = 0;

    // ---- 状态查询 ----
    /// 累计处理帧对数。
    virtual uint64_t ProcessedFrameCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class OpticalFlowEstimatorStub final : public IOpticalFlowEstimator {
public:
    OpticalFlowEstimatorStub();
    ~OpticalFlowEstimatorStub() override;

    OpticalFlowEstimatorStub(const OpticalFlowEstimatorStub&) = delete;
    OpticalFlowEstimatorStub& operator=(const OpticalFlowEstimatorStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;

    void SetInput(common::Topic<video::FrameHandle>& input) override;

    common::Topic<common::OpticalFlowResult>& FlowOutput() override;

    uint64_t ProcessedFrameCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t processed_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::OpticalFlowResult> flow_output_;
};

}  // namespace drone::perception
