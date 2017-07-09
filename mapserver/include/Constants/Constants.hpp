#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

/*! \brief Constants regarding the mapserver project
 *
 *  This header contains constant variables
 *  regarding the mapserver project, which means that
 *  these values do not change during runtime.
 *  Constants are e.g. physical ones like seconds per minute
 *  or geometrical ones like the circumference of wheel.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly in milli iff the variable
 *  is of type integer, unless it is explicitly named in
 *  the variable.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly without prefix (e.g. milli)
 *  iff the variable is of type float, unless it is
 *  explicitly named in the variable. The SI prefix is
 *  used, iff the variable is of type float and therefor
 *  in SI units.
 */

#include <math.h>
#include <float.h>
#include <stdint.h>
#include <Eigen/Dense>


#include "scopes.hpp"
#include "geometry.hpp"
#include "numeric.hpp"
#include "odometry.hpp"
#include "sensor.hpp"
#include "time.hpp"
#include "mapping.hpp"
#include "machine.hpp"
#include "tf.hpp"

#endif /* CONSTANTS_HPP_ */
