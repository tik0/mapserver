#ifndef CONSTANTS_TIME_HPP_
#define CONSTANTS_TIME_HPP_

#include "common.hpp"

namespace constants {
namespace time {
/** \brief Hours per day */
const int32_t hoursPerDay = 24;

/** \brief Amount of seconds per minute */
const int32_t secondsPerMinute = 60;

/** \brief Amount of minutes per hour */
const int32_t minutesPerHour = 60;

/** \brief Amount of seconds per hour */
const int32_t secondsPerHour = minutesPerHour * secondsPerMinute;

/** \brief Amount of seconds per day */
const int32_t minutesPerDay = minutesPerHour * hoursPerDay;

/** \brief Amount of milliseconds per second */
const int32_t millisecondsPerSecond = 1e3;

/** \brief Amount of microseconds per second */
const int32_t microscondsPerSecond = 1e6;
}
}

#endif // CONSTANTS_TIME_HPP_

