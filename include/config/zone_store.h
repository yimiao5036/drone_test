/**
 * @file zone_store.h
 * @brief 围栏区域缓存（config/zone.json）读写
 *
 * 属于 drone/config 模块。职责：把地面站 65018 下发的围栏区域落盘为
 * `config/zone.json`，重启后由机载重新装载——保证"地面站掉线/飞机重启后
 * 仍持有同一份围栏"，不必依赖地面站在线。
 *
 * 边界：只做纯文件读写与 JSON 编解码，不做 WGS84→NED 转换、不做越界判定
 * （分别在 MissionStateMachine 的围栏守卫与 control::fence 中）。
 *
 * 实现文档见同目录 配置管理.md「围栏区域缓存」一节。
 */
#pragma once

#include <optional>
#include <string>

#include "common/types.h"

namespace drone::config {

/// 围栏区域缓存文件读写（原子写：临时文件 + rename，避免半截文件）。
class ZoneStore {
public:
    /// 区域缓存文件结构版本（与 65018 线格式版本无关）。
    static constexpr int kSchemaVersion = 1;

    /// 读取缓存。文件不存在返回 std::nullopt（正常情形：从未下发过区域）；
    /// 内容非法（JSON 解析失败/顶点不足 3 个/zone_id 或 version 为 0）同样
    /// 返回 std::nullopt，并记一条 WARN 便于排查。
    static std::optional<common::FenceZone> Load(const std::string& path);

    /// 原子写入缓存。父目录不存在时尝试创建（仅一层）；成功返回 true。
    static bool Save(const std::string& path, const common::FenceZone& zone);
};

}  // namespace drone::config
