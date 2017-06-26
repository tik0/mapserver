#ifndef CONSTANTS_ODOMETRY_HPP_
#define CONSTANTS_ODOMETRY_HPP_

#include <math.h>
#include "common.hpp"

namespace constants {
namespace odometry {
/** \brief Sampling time of the odometry*/
const double sampleTime = 0.05;  // seconds

/** \brief Corvature offset*/
const double curvatureOffset = 0.0226;  // 1 / kilometer

/** \brief Initial machine orientation in degree*/
const double orientation_deg = 0;  // degree

/** \brief Initial machine orientation in rad*/
const double orientation_rad = orientation_deg * M_PI / 180.0;  // Radiant
}
}
#endif // CONSTANTS_ODOMETRY_HPP_
