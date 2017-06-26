#ifndef CONSTANTS_SENSOR_HPP_
#define CONSTANTS_SENSOR_HPP_

#include <math.h>
#include "common.hpp"

namespace constants {
namespace sensors {

namespace LASE_2000D_226 {
const double angleMin_deg = -45.0;
const double angleMax_deg = 45.0;
const double angleIncrement_deg = 0.09;
const double freqzenzy_hz = 16;
const double measurementDelta_s = 1e-5;  // Time between to measurement points
const double rangeMax_m = 80.0;
const double rangeMin_m = 0.2;
const double rangeMeasurementToMeter_m = 1e-4;  // 1/10 mm
const int numMeasurements = 1e3;
const int32_t indicatorDistanceNoisy = 0x7FFFFFFF;  // 2147483647
const int32_t indicatorDistanceClose = 0x80000000;  // -2147483648
const int32_t pulsewidth100percent_ps = 10000;  // Pulsewidth in ps for 100 % signal strenght
const int32_t pulsewidth3percent_ps = 4096;  // Pulsewidth in ps for 3 % signal strenght
// Converts pulsewidth to signal strenght in percent
int calcStrenght(int32_t pulsewidth_ps) {
  return (100.0f - 3.0f)
      / float(pulsewidth100percent_ps - pulsewidth3percent_ps)
      * float(pulsewidth_ps - pulsewidth3percent_ps) + 3.0f;
}
}

namespace SICK_LDMRS {
const int numLayers = 4;
const double angleMinLayer1and2_rad = -50.0 * deg2rad;
const double angleMaxLayer1and2_rad = 50.0 * deg2rad;
const double angleMinLayer3and4_rad = -35.0 * deg2rad;
const double angleMaxLayer3and4_rad = 60.0 * deg2rad;
const double layer1Inclination_rad = -1.2 * deg2rad;
const double layer2Inclination_rad = -0.4 * deg2rad;
const double layer3Inclination_rad = 0.4 * deg2rad;
const double layer4Inclination_rad = 1.2 * deg2rad;
const double rangeMax_m = 100.0;
const double rangeMin_m = 0.2;
// TODO Is a constant increment used, or the variable one, which is described in the manual
const double angleIncrementBetweenLayers_rad = 0.25 * deg2rad;
const double angleIncrementPerLayers_rad = 2.0
    * angleIncrementBetweenLayers_rad;
const double freqzenzy_hz = 25;
const double rangeMeasurementToMeter_m = 1e-2;  // cm
}

namespace PF {
const double angleMin_rad = -44.0 * deg2rad;
const double angleMax_rad = 44.0 * deg2rad;
const double angleIncrement_rad = 8.0 * deg2rad;
const double freqzenzy_hz = 50;
const double measurementDelta_s = 0;  // ? Time between to measurement points
const double rangeMax_m = 10.0;  // actually 10 meter
const double rangeMin_m = 0.2;
const int numMeasurements = 11;
const double rangeMeasurementToMeter_m = 1e-3;  // mm
}

namespace LEUZ {
// TODO
}

}
}
#endif // CONSTANTS_SENSOR_HPP_
