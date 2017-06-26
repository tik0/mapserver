#ifndef CONSTANTS_GEOMETRY_HPP_
#define CONSTANTS_GEOMETRY_HPP_

#include <math.h>
#include "common.hpp"

namespace constants {
namespace geometry {
namespace si {
/** \brief Distance between wheels in meter */
const double wheelBaseDistance = 0.069f;  // TODO Correct value

/** \brief Wheel diameter in meter */
const double wheelDiameter = 0.05571f;  // TODO Correct value

/** \brief Wheel circumference in meter */
const double wheelCircumference = M_PI * wheelDiameter;
}

/** \brief Distance between wheels in millimeter */
const int32_t wheelBaseDistance = si::wheelBaseDistance * 1e3;

/** \brief Wheel diameter */
const int32_t wheelDiameter = si::wheelDiameter * 1e3;

/** \brief Wheel circumference in millimeter */
const int32_t wheelCircumference = si::wheelCircumference * 1e3;

/** \brief Millimeter per meter */
const int32_t millimeterPerMeter = 1e3;

/** \brief Meter per millimeter */
const double meterPerMillimeter = 1e-3;
}
}

#endif // CONSTANTS_GEOMETRY_HPP_
