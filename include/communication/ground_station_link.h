/**
 * @file ground_station_link.h
 * @brief 地面站通信部件接口（IGroundStationLink）
 *
 * 属于 drone/communication 模块。职责：与地面站经电台 MAVLink 双向通信——
 * 上行接收任务输入与目标状态（GroundStationTarget），下行回传任务与
 * 拦截状态（MissionStatus）。
 *
 * 骨架期说明：
 * - 本接口为纯虚抽象，地面站 dialect/电台协议适配在实现期接入。
 * - GroundStationLinkStub 为骨架占位实现：生命周期可运行，业务方法
 *   记录"未实现"节流日志并返回默认值。
 *
 * 数据流：
 *   电台串口 ──► IGroundStationLink ──► common::Topic<GroundStationTarget>
 *   common::Topic<MissionStatus> ──► IGroundStationLink ──► 电台串口
 * 可替换边界：MAVLink dialect/电台适配器。
 */
#pragma once

#include <cstdint>

#include "common/topic.h"
#include "common/types.h"

namespace drone::communication {

/// 地面站通信部件抽象接口。
class IGroundStationLink {
public:
    virtual ~IGroundStationLink() = default;

    // ---- 生命周期 ----
    /// 启动通信（打开串口、启动收发线程）。返回是否成功启动。
    virtual bool Start() = 0;
    /// 停止通信并关闭串口；幂等。
    virtual void Stop() = 0;
    /// 是否已启动。
    virtual bool IsRunning() const = 0;
    /// 当前是否收到地面站心跳（连接状态）。
    virtual bool IsConnected() const = 0;

    // ---- 上行输出 ----
    /// 地面站目标状态输出主题：common::GroundStationTarget。
    virtual common::Topic<common::GroundStationTarget>& TargetOutput() = 0;

    // ---- 下行输入 ----
    /// 绑定任务状态回传输入主题（IMissionStateMachine::StatusOutput()）。
    virtual void SetInput(common::Topic<common::MissionStatus>& status) = 0;

    // ---- 状态查询 ----
    /// 累计发送消息数。
    virtual uint64_t SendCount() const = 0;
    /// 累计接收消息数。
    virtual uint64_t ReceiveCount() const = 0;
    /// 累计错误次数。
    virtual uint64_t ErrorCount() const = 0;
};

/// 骨架占位实现：生命周期完整，业务方法打印"未实现"节流日志并返回默认值。
class GroundStationLinkStub final : public IGroundStationLink {
public:
    GroundStationLinkStub();
    ~GroundStationLinkStub() override;

    GroundStationLinkStub(const GroundStationLinkStub&) = delete;
    GroundStationLinkStub& operator=(const GroundStationLinkStub&) = delete;

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    bool IsConnected() const override;

    common::Topic<common::GroundStationTarget>& TargetOutput() override;

    void SetInput(common::Topic<common::MissionStatus>& status) override;

    uint64_t SendCount() const override;
    uint64_t ReceiveCount() const override;
    uint64_t ErrorCount() const override;

private:
    bool running_ = false;
    uint64_t send_count_ = 0;
    uint64_t receive_count_ = 0;
    uint64_t error_count_ = 0;
    common::Topic<common::GroundStationTarget> target_output_;
};

}  // namespace drone::communication
