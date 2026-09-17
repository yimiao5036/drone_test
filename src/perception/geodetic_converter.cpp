#include "perception/geodetic_converter.h"

#include <cmath>

namespace drone::perception {
namespace {

constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

bool IsValid(const GeodeticCoordinate& value) {
    return std::isfinite(value.latitude_deg) &&
           std::isfinite(value.longitude_deg) &&
           std::isfinite(value.altitude_m) &&
           value.latitude_deg >= -90.0 && value.latitude_deg <= 90.0 &&
           value.longitude_deg >= -180.0 && value.longitude_deg <= 180.0;
}

// 经度差归一化到[-pi, pi]，避免跨越180度经线时走长路径。
double NormalizeLongitudeDelta(double value_rad) {
    while (value_rad > kPi) {
        value_rad -= 2.0 * kPi;
    }
    while (value_rad < -kPi) {
        value_rad += 2.0 * kPi;
    }
    return value_rad;
}

}  // namespace

std::optional<LocalNedCoordinate> GeodeticToLocalNed(
    const GeodeticCoordinate& reference,
    const GeodeticCoordinate& target) {
    if (!IsValid(reference) || !IsValid(target)) {
        return std::nullopt;
    }

    const double latitude_rad = reference.latitude_deg * kDegreesToRadians;
    const double delta_latitude_rad =
        (target.latitude_deg - reference.latitude_deg) * kDegreesToRadians;
    const double delta_longitude_rad = NormalizeLongitudeDelta(
        (target.longitude_deg - reference.longitude_deg) * kDegreesToRadians);

    const double eccentricity_squared =
        kWgs84Flattening * (2.0 - kWgs84Flattening);
    const double sin_latitude = std::sin(latitude_rad);
    const double denominator =
        std::sqrt(1.0 - eccentricity_squared * sin_latitude * sin_latitude);
    const double prime_vertical_radius_m =
        kWgs84SemiMajorAxisM / denominator;
    const double meridian_radius_m =
        kWgs84SemiMajorAxisM * (1.0 - eccentricity_squared) /
        (denominator * denominator * denominator);

    LocalNedCoordinate result;
    result.north_m = delta_latitude_rad *
                     (meridian_radius_m + reference.altitude_m);
    result.east_m = delta_longitude_rad *
                    (prime_vertical_radius_m + reference.altitude_m) *
                    std::cos(latitude_rad);
    // NED向下为正；目标高度高于参考点时down为负。
    result.down_m = reference.altitude_m - target.altitude_m;
    if (!std::isfinite(result.north_m) || !std::isfinite(result.east_m) ||
        !std::isfinite(result.down_m)) {
        return std::nullopt;
    }
    return result;
}

}  // namespace drone::perception
