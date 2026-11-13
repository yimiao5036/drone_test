/**
 * @file zone_store.cpp
 * @brief 围栏区域缓存（config/zone.json）读写实现
 */
#include "config/zone_store.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include "common/logger.h"
#include "nlohmann/json.hpp"

namespace drone::config {
namespace {

using nlohmann::json;

const char* ActionToText(common::FenceAction action) {
    switch (action) {
        case common::FenceAction::kHold:
            return "hold";
        case common::FenceAction::kReturn:
            return "return";
        case common::FenceAction::kLand:
            return "land";
    }
    return "return";
}

common::FenceAction ActionFromText(const std::string& text,
                                   common::FenceAction fallback) {
    if (text == "hold") {
        return common::FenceAction::kHold;
    }
    if (text == "return") {
        return common::FenceAction::kReturn;
    }
    if (text == "land") {
        return common::FenceAction::kLand;
    }
    return fallback;
}

}  // namespace

std::optional<common::FenceZone> ZoneStore::Load(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return std::nullopt;  // 从未下发过区域属正常情形，不打日志
    }
    json root;
    try {
        input >> root;
    } catch (const std::exception& error) {
        SPDLOG_WARN("围栏区域缓存解析失败: path={} error={}", path, error.what());
        return std::nullopt;
    }
    common::FenceZone zone;
    try {
        zone.protocol_version =
            static_cast<uint8_t>(root.value("protocol_version", 1));
        zone.zone_id = static_cast<uint16_t>(root.value("zone_id", 0));
        zone.version = static_cast<uint16_t>(root.value("version", 0));
        zone.type = static_cast<common::FenceZoneType>(root.value("type", 0));
        zone.action = ActionFromText(root.value("action", std::string("return")),
                                     common::FenceAction::kReturn);
        zone.altitude_reference =
            static_cast<uint8_t>(root.value("altitude_reference", 1));
        zone.alt_min_m = root.value("alt_min_m", 0.0);
        zone.alt_max_m = root.value("alt_max_m", 0.0);
        const json vertices = root.value("vertices", json::array());
        for (const auto& vertex : vertices) {
            common::FenceVertex point;
            point.latitude_deg = vertex.value("latitude_deg", 0.0);
            point.longitude_deg = vertex.value("longitude_deg", 0.0);
            zone.vertices.push_back(point);
        }
    } catch (const std::exception& error) {
        SPDLOG_WARN("围栏区域缓存字段非法: path={} error={}", path, error.what());
        return std::nullopt;
    }
    if (zone.zone_id == 0 || zone.version == 0 || zone.vertices.size() < 3) {
        SPDLOG_WARN("围栏区域缓存内容不完整，已忽略: path={} zone_id={} version={} "
                    "vertices={}",
                    path, zone.zone_id, zone.version, zone.vertices.size());
        return std::nullopt;
    }
    SPDLOG_INFO("围栏区域缓存已加载: path={} zone_id={} version={} vertices={} "
                "action={} alt=[{:.1f},{:.1f}]m",
                path, zone.zone_id, zone.version, zone.vertices.size(),
                ActionToText(zone.action), zone.alt_min_m, zone.alt_max_m);
    return zone;
}

bool ZoneStore::Save(const std::string& path, const common::FenceZone& zone) {
    json root;
    root["schema"] = kSchemaVersion;
    root["protocol_version"] = zone.protocol_version;
    root["zone_id"] = zone.zone_id;
    root["version"] = zone.version;
    root["type"] = static_cast<int>(zone.type);
    root["action"] = ActionToText(zone.action);
    root["altitude_reference"] = zone.altitude_reference;
    root["alt_min_m"] = zone.alt_min_m;
    root["alt_max_m"] = zone.alt_max_m;
    json vertices = json::array();
    for (const auto& vertex : zone.vertices) {
        json item;
        item["latitude_deg"] = vertex.latitude_deg;
        item["longitude_deg"] = vertex.longitude_deg;
        vertices.push_back(item);
    }
    root["vertices"] = vertices;

    // 原子写：先写临时文件再 rename，避免掉电留下半截 JSON。
    const std::string temporary = path + ".tmp";
    {
        std::ofstream output(temporary, std::ios::trunc);
        if (!output.is_open()) {
            SPDLOG_ERROR("围栏区域缓存写入失败，无法打开临时文件: path={}", temporary);
            return false;
        }
        output << root.dump(2) << '\n';
        if (!output.good()) {
            SPDLOG_ERROR("围栏区域缓存写入失败: path={}", temporary);
            return false;
        }
    }
    std::error_code error;
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::filesystem::remove(temporary, error);
        SPDLOG_ERROR("围栏区域缓存落盘失败: path={} error={}", path, error.message());
        return false;
    }
    SPDLOG_INFO("围栏区域缓存已保存: path={} zone_id={} version={} vertices={}",
                path, zone.zone_id, zone.version, zone.vertices.size());
    return true;
}

}  // namespace drone::config
