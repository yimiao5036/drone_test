#pragma once

#include <cmath>
#include <array>
#include <type_traits>

namespace geo {

// ==================== WGS84 常量 ====================
inline constexpr double WGS84_A  = 6378137.0;                    // 长半轴 (m)
inline constexpr double WGS84_F  = 1.0 / 298.257223563;          // 扁率
inline constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);    // e²

inline constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
inline constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;

// ==================== 数据结构 ====================
struct LLA {
    double lat = 0.0;   // 纬度 (度)
    double lon = 0.0;   // 经度 (度)
    double alt = 0.0;   // 高度 (m)
};

struct ECEF {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct NED {
    double n = 0.0;     // 北 (m)
    double e = 0.0;     // 东 (m)
    double d = 0.0;     // 地 (m)
};

struct ENU {
    double e = 0.0;     // 东 (m)
    double n = 0.0;     // 北 (m)
    double u = 0.0;     // 天 (m)
};

// ==================== 内部辅助 ====================
namespace detail {

inline void ecef_to_enu_matrix(double lat0_deg, double lon0_deg, double R[3][3]) noexcept {
    const double lat0 = lat0_deg * DEG2RAD;
    const double lon0 = lon0_deg * DEG2RAD;
    const double sin_lat = std::sin(lat0);
    const double cos_lat = std::cos(lat0);
    const double sin_lon = std::sin(lon0);
    const double cos_lon = std::cos(lon0);

    // 东
    R[0][0] = -sin_lon;
    R[0][1] =  cos_lon;
    R[0][2] =  0.0;

    // 北
    R[1][0] = -sin_lat * cos_lon;
    R[1][1] = -sin_lat * sin_lon;
    R[1][2] =  cos_lat;

    // 天
    R[2][0] =  cos_lat * cos_lon;
    R[2][1] =  cos_lat * sin_lon;
    R[2][2] =  sin_lat;
}

} // namespace detail

// ==================== 核心转换 ====================

/** LLA → ECEF */
[[nodiscard]] inline ECEF lla_to_ecef(const LLA& lla) noexcept {
    const double lat = lla.lat * DEG2RAD;
    const double lon = lla.lon * DEG2RAD;
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double cos_lon = std::cos(lon);
    const double sin_lon = std::sin(lon);

    const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    return ECEF{
        (N + lla.alt) * cos_lat * cos_lon,
        (N + lla.alt) * cos_lat * sin_lon,
        (N * (1.0 - WGS84_E2) + lla.alt) * sin_lat
    };
}

/** ECEF → LLA（迭代法，亚毫米级精度） */
[[nodiscard]] inline LLA ecef_to_lla(const ECEF& ecef) noexcept {
    const double lon = std::atan2(ecef.y, ecef.x);
    const double p   = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);

    double lat = std::atan2(ecef.z, p * (1.0 - WGS84_E2));
    double h   = 0.0;

    for (int i = 0; i < 10; ++i) {
        const double sin_lat = std::sin(lat);
        const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
        h = p / std::cos(lat) - N;
        const double lat_new = std::atan2(ecef.z, p * (1.0 - WGS84_E2 * N / (N + h)));
        if (std::abs(lat_new - lat) < 1e-12) {
            lat = lat_new;
            break;
        }
        lat = lat_new;
    }

    return LLA{lat * RAD2DEG, lon * RAD2DEG, h};
}

/** ECEF → ENU */
[[nodiscard]] inline ENU ecef_to_enu(const ECEF& ecef, const ECEF& ref_ecef,
                                     double lat0_deg, double lon0_deg) noexcept {
    double R[3][3];
    detail::ecef_to_enu_matrix(lat0_deg, lon0_deg, R);

    const double dx = ecef.x - ref_ecef.x;
    const double dy = ecef.y - ref_ecef.y;
    const double dz = ecef.z - ref_ecef.z;

    return ENU{
        R[0][0]*dx + R[0][1]*dy + R[0][2]*dz,  // E
        R[1][0]*dx + R[1][1]*dy + R[1][2]*dz,  // N
        R[2][0]*dx + R[2][1]*dy + R[2][2]*dz   // U
    };
}

/** ECEF → NED */
[[nodiscard]] inline NED ecef_to_ned(const ECEF& ecef, const ECEF& ref_ecef,
                                     double lat0_deg, double lon0_deg) noexcept {
    const ENU enu = ecef_to_enu(ecef, ref_ecef, lat0_deg, lon0_deg);
    return NED{enu.n, enu.e, -enu.u};
}

// ==================== 高层便捷接口 ====================

/** LLA → NED（最常用） */
[[nodiscard]] inline NED lla_to_ned(const LLA& target, const LLA& ref) noexcept {
    const ECEF ecef     = lla_to_ecef(target);
    const ECEF ref_ecef = lla_to_ecef(ref);
    return ecef_to_ned(ecef, ref_ecef, ref.lat, ref.lon);
}

/** NED → LLA */
[[nodiscard]] inline LLA ned_to_lla(const NED& ned, const LLA& ref) noexcept {
    const ECEF ref_ecef = lla_to_ecef(ref);

    // NED → ENU
    const ENU enu{ned.e, ned.n, -ned.d};

    double R[3][3];
    detail::ecef_to_enu_matrix(ref.lat, ref.lon, R);

    // R 是正交矩阵，转置 = 逆
    const ECEF ecef{
        ref_ecef.x + R[0][0]*enu.e + R[1][0]*enu.n + R[2][0]*enu.u,
        ref_ecef.y + R[0][1]*enu.e + R[1][1]*enu.n + R[2][1]*enu.u,
        ref_ecef.z + R[0][2]*enu.e + R[1][2]*enu.n + R[2][2]*enu.u
    };

    return ecef_to_lla(ecef);
}

/** LLA → ENU */
[[nodiscard]] inline ENU lla_to_enu(const LLA& target, const LLA& ref) noexcept {
    const ECEF ecef     = lla_to_ecef(target);
    const ECEF ref_ecef = lla_to_ecef(ref);
    return ecef_to_enu(ecef, ref_ecef, ref.lat, ref.lon);
}

/** ENU → LLA */
[[nodiscard]] inline LLA enu_to_lla(const ENU& enu, const LLA& ref) noexcept {
    return ned_to_lla(NED{enu.n, enu.e, -enu.u}, ref);
}

} // namespace geo