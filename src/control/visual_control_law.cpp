// =============================================================================
// visual_control_law.cpp —— 视觉控制段接口工厂
//
// 当前只有 TrackingControlLaw 一个实现：捕网与干扰共用（参数按本机配置），
// 撞击剖面预留、调用即抛异常。详见 include/control/visual_control_law.h。
// =============================================================================

#include "control/visual_control_law.h"

#include <stdexcept>

#include "control/tracking_control_law.h"

namespace drone::control {

const char* VisualProfileName(VisualProfile profile) {
    switch (profile) {
        case VisualProfile::kNetCapture:
            return "net_capture";
        case VisualProfile::kJammer:
            return "jammer";
        case VisualProfile::kImpact:
            return "impact";
    }
    return "unknown";
}

std::unique_ptr<IVisualControlLaw> CreateVisualControlLaw(
    VisualProfile profile, const TrackingConfig& config) {
    switch (profile) {
        case VisualProfile::kNetCapture:
        case VisualProfile::kJammer:
            // 干扰与捕网的视觉控制段同构：都是“到距离保持 + 瞄准”，
            // 差异只在 R/速度上限/末端动作，参数由本机 visual_tracking 注入。
            return std::make_unique<TrackingControlLaw>(config, profile);
        case VisualProfile::kImpact:
            // 撞击需要“不保持距离、全速穿越”的纵向律，尚未实现。这里必须
            // fail-fast：不能让撞击机静默套用捕网律飞到目标前方悬停。
            throw std::invalid_argument(
                "视觉剖面 impact 尚未实现（撞击机预留）：当前仅支持 "
                "net_capture/jammer");
    }
    throw std::invalid_argument("未知的视觉剖面枚举值");
}

}  // namespace drone::control
