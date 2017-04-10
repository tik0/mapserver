#ifndef MS_CONSTANTS_H_
#define MS_CONSTANTS_H_

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

namespace ms {
namespace constants {

const double rad2deg = 180.0 / M_PI;
const double deg2rad = M_PI / 180.0;

namespace numeric {
  const int8_t  invalidValue_int8   = 0x80;
  const int16_t invalidValue_int16  = 0x8000;
  const int32_t invalidValue_int32  = 0x80000000;
  const int64_t invalidValue_int64  = 0x8000000000000000;
  const float   invalidValue_float  = -FLT_MAX;
  const double  invalidValue_double = -DBL_MAX;
  const int8_t  invalidValueP_int8   = 0x7F;
  const int16_t invalidValueP_int16  = 0x7FFF;
  const int32_t invalidValueP_int32  = 0x7FFFFFFF;
  const int64_t invalidValueP_int64  = 0x7FFFFFFFFFFFFFFF;
  const float   invalidValueP_float  = FLT_MAX;
  const double  invalidValueP_double = DBL_MAX;
}

namespace mapping {
  /** \brief Map resolution */
  const double discreteResolution = 0.1; // meter per cell
  
  namespace roi {
    /** \brief Map width */
    const double width = 35; // meter
    
    /** \brief Map height */
    const double height = 35; // meter

    /** \brief Map altitude */
    const double altitude = 35; // meter
    
    namespace boundingbox {
      /** \brief Boundingbox width */
      const double width = 8; // meter
      
      /** \brief Boundingbox height */
      const double height = 8; // meter

      /** \brief Boundingbox altitude */
      // WARNING is not checked in mapserver/tile_publisher
      const double altitude = 0; // meter
    }
    
    /** \brief Coordinate system origin in lateral (x) direction */
    const double originWidth = width / 2.0; // meter
    
    /** \brief Coordinate system origin in vertical (y) direction */
    const double originHeight = height / 2.0; // meter

    /** \brief Coordinate system origin in horizontal (z) direction */
    const double originAltitude = altitude / 2.0; // meter
    
    /** \brief Minimum lateral coordinate (x) */
    const double xMin = originWidth - width; // meter
    
    /** \brief Maximum lateral coordinate (x) */
    const double xMax = width - originWidth; // meter
    
    /** \brief Minimum vertical coordinate (y) */
    const double yMin = originHeight - height; // meter
    
    /** \brief Maximum vertical coordinate (y) */
    const double yMax = height - originHeight; // meter
    
    namespace discrete {
      /** \brief Map width */
      const size_t width = roi::width / discreteResolution; // cells
      
      /** \brief Map height */
      const size_t height = roi::height / discreteResolution; // cells
      
      /** \brief Amount of cells in the map */
      const size_t numCells = width * height;
      
      /** \brief Coordinate system origin in lateral (x) direction */
      const size_t originWidth = width / 2.0; // meter
      
      /** \brief Coordinate system origin in vertical (y) direction */
      const size_t originHeight = height / 2.0; // meter
      
      /** \brief Minimum lateral coordinate (x) */
      const size_t xMin = originWidth - width;
      
      /** \brief Maximum lateral coordinate (x) */
      const size_t xMax = width - originWidth;
      
      /** \brief Minimum vertical coordinate (y) */
      const size_t yMin = originHeight - height;
      
      /** \brief Maximum vertical coordinate (y) */
      const size_t yMax = height - originHeight;      
    }
  }
  
  namespace ogm {
    
    /** \brief Maximum allowed certainty value in the OGM server */
    const double maxOccupancyUpdateCertainty = 0.8;
    
    /** \brief Minimum allowed certainty value in the OGM server */
    const double minOccupancyUpdateCertainty = 0.2;
    
    /** \brief OGM value for unknown */
    const double unknownOccupancyUpdateCertainty = 0.5;
    
    /** \brief Minimum certainty value, to be drawn into the colored image */
    const double minDrawOccupancyUpdateCertainty = 0.6;
    
    /** \brief Standard storage location */
    const std::string mapStorageLocation("/tmp/");
  }
}

namespace odometry {
  /** \brief Sampling time of the odometry*/
  const double sampleTime = 0.05; // seconds

  /** \brief Corvature offset*/
  const double curvatureOffset = 0.0226; // 1 / kilometer

  /** \brief Initial machine orientation in degree*/
  const double orientation_deg = 0; // degree
  
  /** \brief Initial machine orientation in rad*/
  const double orientation_rad = orientation_deg * M_PI / 180.0; // Radiant
}

namespace sensors {
  namespace LASE_2000D_226 {
    const double angleMin_deg = -45.0;
    const double angleMax_deg = 45.0;
    const double angleIncrement_deg = 0.09;
    const double freqzenzy_hz = 16;
    const double measurementDelta_s = 1e-5; // Time between to measurement points
    const double rangeMax_m = 80.0;
    const double rangeMin_m = 0.2;
    const double rangeMeasurementToMeter_m = 1e-4; // 1/10 mm
    const int numMeasurements = 1e3;
    const int32_t indicatorDistanceNoisy = 0x7FFFFFFF; // 2147483647
    const int32_t indicatorDistanceClose = 0x80000000; // -2147483648
    const int32_t pulsewidth100percent_ps = 10000; // Pulsewidth in ps for 100 % signal strenght
    const int32_t pulsewidth3percent_ps = 4096;  // Pulsewidth in ps for 3 % signal strenght
    // Converts pulsewidth to signal strenght in percent
    int calcStrenght(int32_t pulsewidth_ps) {
      return (100.0f - 3.0f) / float(pulsewidth100percent_ps - pulsewidth3percent_ps) * float(pulsewidth_ps - pulsewidth3percent_ps) + 3.0f;
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
    const double angleIncrementPerLayers_rad = 2.0 * angleIncrementBetweenLayers_rad;
    const double freqzenzy_hz = 25;
    const double rangeMeasurementToMeter_m = 1e-2; // cm
  }
  
  namespace PF {
    const double angleMin_rad = -44.0 * deg2rad;
    const double angleMax_rad = 44.0 * deg2rad;
    const double angleIncrement_rad = 8.0 * deg2rad;
    const double freqzenzy_hz = 50;
    const double measurementDelta_s = 0; // ? Time between to measurement points
    const double rangeMax_m = 10.0;  // actually 10 meter
    const double rangeMin_m = 0.2;
    const int numMeasurements = 11;
    const double rangeMeasurementToMeter_m = 1e-3; // mm
  }
  
  namespace LEUZ {
    // TODO
  }
}

namespace machine {
  namespace tf {
    template <typename T>
    constexpr Eigen::Matrix<T, 4, 4> trans( T x, T y, T z) {
      Eigen::Matrix<T, 4, 4> returnMatrix;
      returnMatrix << 1, 0, 0, x,
                      0, 1, 0, y,
                      0, 0, 1, z,
                      0, 0, 0, 1;
      return returnMatrix;
    }
    template <typename T>
    constexpr Eigen::Matrix<T, 4, 4> rotX( T angle) {
      Eigen::Matrix<T, 4, 4> returnMatrix;
      returnMatrix << 1, 0,          0,           0,
                      0, cos(angle), -sin(angle), 0,
                      0, sin(angle), cos(angle),  0,
                      0, 0,          0,           1;
      return returnMatrix;
    }
    template <typename T>
    constexpr Eigen::Matrix<T, 4, 4> rotY( T angle) {
      Eigen::Matrix<T, 4, 4> returnMatrix;
      returnMatrix << cos(angle),  0, sin(angle),  0,
                      0,           1, 0,           0,
                      -sin(angle), 0, cos(angle),  0,
                      0,           0, 0,           1;
      return returnMatrix;
    }
    template <typename T>
    constexpr Eigen::Matrix<T, 4, 4> rotZ( T angle) {
      Eigen::Matrix<T, 4, 4> returnMatrix;
      returnMatrix << cos(angle), -sin(angle), 0, 0,
                      sin(angle),  cos(angle), 0, 0,
                      0,          0,           1, 0,
                      0,          0,           0, 1;
      return returnMatrix;
    }
    template <typename T>
    constexpr Eigen::Matrix<T, 3, 3> getRot(Eigen::Matrix<T, 4, 4> M) {
      Eigen::Matrix<T, 3, 3> returnMatrix;
        return M.template block<3,3 >(0,0);
    }
    template <typename T>
    constexpr Eigen::Matrix<T, 3, 1> getTrans(Eigen::Matrix<T, 4, 4> M) {
      return M.template block<3,1>(0,3);
    }
  }
  namespace frames {
    namespace names {
      const std::string WORLD("world");
      const std::string ROI_ORIGIN("ROI_ORIGIN");
      const std::string ROI("ROI");
      // const std::string GLOBAL("GLOBAL");
      const std::string MACHINE("MACHINE");
      const std::string COILER("COILER");
      const std::string MACHINE_ROI("MACHINE_ROI");
      const std::string LEUZE("LEUZ");
      const std::string LASE1_BASE("LASE1_BASE");
      const std::string LASE1("LASE1");
      const std::string LASE2_BASE("LASE2_BASE");
      const std::string LASE2("LASE2");
      const std::string HORNS("HORNS");
      const std::string SICK1_BASE("SICK1_BASE");
      const std::string SICK1("SICK1");
      const std::string SICK1_LAYER1("SICK1_LAYER1");
      const std::string SICK1_LAYER2("SICK1_LAYER2");
      const std::string SICK1_LAYER3("SICK1_LAYER3");
      const std::string SICK1_LAYER4("SICK1_LAYER4");
      const std::string SICK2_BASE("SICK2_BASE");
      const std::string SICK2_LAYER1("SICK2_LAYER1");
      const std::string SICK2_LAYER2("SICK2_LAYER2");
      const std::string SICK2_LAYER3("SICK2_LAYER3");
      const std::string SICK2_LAYER4("SICK2_LAYER4");
      const std::string SICK2("SICK2");
      const std::string PF("PF");
      const std::string BACK_BASE("BACK_BASE");
      const std::string SICK3("SICK3");
      const std::string SICK3_LAYER1("SICK3_LAYER1");
      const std::string SICK3_LAYER2("SICK3_LAYER2");
      const std::string SICK3_LAYER3("SICK3_LAYER3");
      const std::string SICK3_LAYER4("SICK3_LAYER4");
      const std::string IBEO_BASE("IBEO_BASE");
      const std::string IBEO("IBEO");
      const std::string IMU("IMU");
      const std::string INCLINATION("INCLINATION");
      const std::string DRIVEN_AXIS_FOOTPRINT("DRIVEN_AXIS_FOOTPRINT");
      const std::string WHEEL_ODOMETRY("WHEEL_ODOMETRY");
      const std::string GPS("GPS");
      const std::string HOKUYO_BASE("HOKUYO_BASE");
      const std::string HOKUYO("HOKUYO");
      const std::string MCAM_BASE("MCAM_BASE");
      const std::string MCAM("MCAM");
      const std::string CCAM_BASE("CCAM_BASE");
      const std::string CCAM("CCAM");
      const std::string IRCAM_BASE("IRCAM_BASE");
      const std::string IRCAM("IRCAM");
      const std::string BASE_LINK("base_link");
    }
   
    /** \brief Transformation from the ROI (middle of ROI) to the ROI Origin, which is the origin for all image based processing frameworks like MATLAB and OpenCV*/
    Eigen::Matrix4d roi_roiOrigin = tf::trans<double>(-mapping::roi::originWidth, -mapping::roi::originHeight, 0.0);
    
    /** \brief Transformation from the machine frame to the coiler frame*/
    // TODO This is static for now, but has to become of course dynamic, with respect to the coiler signals
    Eigen::Matrix4d machine_coiler = tf::trans<double>(-2.1, 1.0, 0.0) * tf::rotY<double>(M_PI) * tf::rotX<double>(M_PI/2.0);
    
    /** \brief Transformation from the Leuze sensor to the machine frame*/
    Eigen::Matrix4d leuze_machine = tf::trans<double>(-2.0*0.852*1.0/4.0, 0.0, 0.0) * tf::rotY<double>(M_PI/2.0) * tf::rotX<double>(M_PI/2.0);

    /** \brief Transformation from the machine frame to the Leuze sensor*/
    Eigen::Matrix4d machine_leuze = leuze_machine.inverse();
    
    /** \brief Transformation from the machine frame to the IMU on the cabine*/
    Eigen::Matrix4d machine_imu = tf::trans<double>(-2.593, 2.222, 0.0) * tf::rotX<double>(-M_PI/2.0);
    
    /** \brief Transformation from the machine frame to the INCLINATION frame*/
    Eigen::Matrix4d machine_inclination = tf::trans<double>(-0.436691, -0.703319, -0.0665) * tf::rotX<double>(-M_PI/2.0);
    
    /** \brief Transformation from the machine frame to the footprint of the driven axis frame*/
    Eigen::Matrix4d machine_drivenAxisFootprint = tf::trans<double>(-0.436691, -1.7456, 0.0);
    
    /** \brief Transformation from the machine frame to the footprint of the driven axis frame*/
    Eigen::Matrix4d machine_wheelOdometry = machine_drivenAxisFootprint;
    
    /** \brief Transformation from the machine frame to the footprint of the driven axis frame*/
    Eigen::Matrix4d machine_gps = machine_drivenAxisFootprint;    
    
    /** \brief mcs-dachhalter_c.png: Transformation from the machine frame to the horns*/
    Eigen::Matrix4d machine_horns = tf::trans<double>(-2.633343, 2.1994, 0.0);

//     /** \brief Transformation from the horns to the LASE1 base plate (lower one without the thickness (4mm) of the mounting plate for both LASE sensors)*/
//     Eigen::Matrix4d horns_lase1base = tf::trans<double>(-0.124451 + (0.004*cos(29.758*M_PI/180.0)), -0.081424 - (0.004*sin(29.758*M_PI/180.0)), 0.0545) * tf::rotY<double>(M_PI) * tf::rotX<double>(M_PI / 2.0) * tf::rotY<double>(-29.758 * M_PI / 180.0);

    /** \brief dachhalter-lase_u_basis_c.png: Transformation from the horns to the LASE1 base plate (lower oneincluding the mounting plate for both LASE sensors)*/
    Eigen::Matrix4d horns_lase1base = tf::trans<double>(-0.124451, -0.081424, 0.0545) * tf::rotY<double>(M_PI) * tf::rotX<double>(M_PI / 2.0) * tf::rotY<double>(-29.758 * M_PI / 180.0);
    
    /** \brief lase-lase_orig_c.png: Transformation from the LASE1 base plate to the LASE1 sensor frame*/
    Eigen::Matrix4d lase1base_lase1 = tf::trans<double>(0.075, 0.0545, 0.0);
    
    /** \brief dachhalter-lase_o_basis_c.png: Transformation from the horns to the LASE2 base plate (upper one)*/
    Eigen::Matrix4d horns_lase2base = tf::trans<double>(-0.206489, 0.062068, 0.0545) * tf::rotY<double>(M_PI) * tf::rotX<double>(M_PI / 2.0) * tf::rotY<double>(-29.758 * M_PI / 180.0);
    
    /** \brief Transformation from the LASE2 base plate to the LASE2 sensor frame*/
    Eigen::Matrix4d lase2base_lase2 = lase1base_lase1;
    
    /** \brief dachhalter-sick_c.png: Transformation from the horns to the SICK1 base plate*/
    Eigen::Matrix4d horns_sick1base = tf::trans<double>(-0.143544, -0.105507, -0.230293) * tf::rotY<double>(-M_PI / 2.0) * tf::rotX<double>(14.613 * M_PI / 180.0);
    
    /** \brief sick-sick_orig-c.png: Transformation from the SICK1 base plate to the SICK1 sensor frame*/
    Eigen::Matrix4d sick1base_sick1 = tf::trans<double>(0.024, 0.0, 0.044) * tf::rotY<double>(-M_PI / 2.0) * tf::rotX<double>(M_PI / 2.0) * tf::rotX<double>(M_PI);
    
    /** \brief dachhalter-sick_c.png: Transformation from the horns to the SICK2 base plate*/
    Eigen::Matrix4d horns_sick2base = tf::trans<double>(-0.149531, -0.092189, 0.284711) * tf::rotY<double>(-M_PI / 2.0) * tf::rotX<double>(33.901 * M_PI / 180.0);
    
    /** \brief Transformation from the SICK2 base plate to the SICK2 sensor frame*/
    Eigen::Matrix4d sick2base_sick2 = sick1base_sick1;
    
    // HACK: This is a direct transformation from MCU to the sensor frame
    /** \brief Transformation from the machine frame to the origin of SICK3*/
    Eigen::Matrix4d machine_sick3 = tf::trans<double>(5.970, 1.601521, 0.166) * tf::rotX<double>(M_PI / 2.0) * tf::rotY<double>((-45-13.5) * M_PI / 180.0);

    /** \brief Transformation from the horns to the IBEO base plate*/
    Eigen::Matrix4d horns_ibeoBase = tf::trans<double>(0.4, 0.2, -0.1);
    
    /** \brief Transformation from the IBEO base plate to the IBEO sensor frame*/
    Eigen::Matrix4d ibeoBase_ibeo = tf::trans<double>(0.1, 0.0, 0.0) * tf::rotY<double>(30.0*M_PI/180.0);
    
    /** \brief Transformation from the horns to the SICK3 base plate*/
    Eigen::Matrix4d horns_sick3base = tf::trans<double>(0.1, 0.2, -0.1);
    
    /** \brief Transformation from the horns to the monochrome camera base plate*/
    Eigen::Matrix4d horns_mCamBase = tf::trans<double>(0.1, 0.0, -0.1);
    
    /** \brief Transformation from the monochrome camera base plate to the monochrome camera frame*/
    Eigen::Matrix4d mCamBase_mCam = tf::trans<double>(0.1, 0.0, 0.0) * tf::rotY<double>(30.0*M_PI/180.0) * tf::rotY<double>(M_PI/2.0);
    
    /** \brief Transformation from the footprint of the machine coordinate system to the machine coordinate system itself */
//    Eigen::Matrix4d machineRoi_machine = tf::trans<double>(0.0, 0.0, 1.7456) * tf::rotX<double>(-M_PI/2.0) * tf::rotZ<double>(M_PI);
    Eigen::Matrix4d machineRoi_machine = tf::trans<double>(0.0, 0.0, 1.7456) * tf::rotX<double>(M_PI/2.0) * tf::rotY<double>(M_PI);
    Eigen::Matrix4d baseLink_machine = machineRoi_machine;

    /** \brief Transformation from the horns to the IR camera base plate*/
    Eigen::Matrix4d horns_irCamBase = tf::trans<double>(0.1, 0.0, -0.4);
    
    /** \brief Transformation from the IR camera base plate to the IR camera frame*/
    Eigen::Matrix4d irCamBase_irCam = tf::trans<double>(0.1, 0.0, 0.0) * tf::rotY<double>(30.0*M_PI/180.0) * tf::rotY<double>(M_PI/2.0);
    
    /** \brief Transformation from the horns to the color camera base plate*/
    Eigen::Matrix4d horns_cCamBase = tf::trans<double>(0.1, -0.4, -0.1);
    
    /** \brief Transformation from the color camera base plate to the color camera frame*/
    Eigen::Matrix4d cCamBase_cCam = tf::trans<double>(0.1, 0.0, 0.0) * tf::rotY<double>(30.0*M_PI/180.0) * tf::rotY<double>(M_PI/2.0);
    
    /** \brief Transformation from the machine frame to the PF sensor*/
    Eigen::Matrix4d machine_pf = tf::trans<double>(2, -0.1, 2.5) * tf::rotX<double>(M_PI) * tf::rotY<double>(M_PI/2.0);
    
    /** \brief Transformation from the machine frame to the back base plate*/
    Eigen::Matrix4d machine_backBase = tf::trans<double>(5, -2.1, 0.0) * tf::rotX<double>(M_PI/2.0);
    
    /** \brief Transformation from the back base plate to the SICK3 sensor frame*/
    Eigen::Matrix4d backBase_sick3 = tf::trans<double>(0.1, 0.0, 0.0) * tf::rotY<double>(30.0*M_PI/180.0);
    
    /** \brief mcs-hok-c.png: Transformation from the machine frame to the Hokuyo base plate*/
    Eigen::Matrix4d machine_hokuyoBase = tf::trans<double>(6.492219, -0.319418, 0.0) * tf::rotZ<double>(M_PI/2.0) * tf::rotX<double>(-M_PI/2.0);
    
    /** \brief Transformation from the Hokuyo base plate Hokuyo laser scanner*/
    // HACK The sensor is not correctly oriented by its messages, thus we doing some nasty stuff here with the rotation
    Eigen::Matrix4d hokuyoBase_hokuyo = tf::trans<double>(0.0, 0.0, 0.1) * tf::rotZ<double>(M_PI/2.0 + M_PI/8.0 + M_PI/16.0 - M_PI/64.0);
    
    /** \brief Transformation for the different Sick LDMRS layers*/
    Eigen::Matrix4d sick_layer1 = tf::rotY<double>(-sensors::SICK_LDMRS::layer1Inclination_rad);
    Eigen::Matrix4d sick_layer2 = tf::rotY<double>(-sensors::SICK_LDMRS::layer2Inclination_rad);
    Eigen::Matrix4d sick_layer3 = tf::rotY<double>(-sensors::SICK_LDMRS::layer3Inclination_rad);
    Eigen::Matrix4d sick_layer4 = tf::rotY<double>(-sensors::SICK_LDMRS::layer4Inclination_rad);
  }
  namespace coiler {
    /** \brief Width of the coiler*/
    const double width = 9.30;  // meter
    
    /** \brief Depth of the coiler*/
    const double depth = 1.2;  // meter
  }
}

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

namespace scopes {
  const std::string root("");

  namespace map {
    namespace displayRoi {
      const std::string displayRoi("/displayRoi/compressed");
    }

    namespace rawServer {
      const std::string parent("/rawServer");
        namespace requests {
          const std::string height("height");
          const std::string meanHeight("meanHeight");
          const std::string meanHeightTest("meanHeightTest");
          const std::string varianceHeight("varianceHeight");
          const std::string varianceHeightTest("varianceHeightTest");
          const std::string quantil90Height("quantil90Height");
          const std::string quantil90HeightTest("quantil90HeightTest");
          const std::string quantil95Height("quantil95Height");
          const std::string quantil95HeightTest("quantil95HeightTest");
          const std::string pulsewidth("pulsewidth");
          const std::string meanPulsewidth("meanPulsewidth");
          const std::string meanPulsewidthTest("meanPulsewidthTest");
          const std::string variancePulsewidth("variancePulsewidth");
          const std::string variancePulsewidthTest("variancePulsewidthTest");
          const std::string quantil90Pulsewidth("quantil90Pulsewidth");
          const std::string quantil90PulsewidthTest("quantil90PulsewidthTest");
          const std::string quantil95Pulsewidth("quantil95Pulsewidth");
          const std::string quantil95PulsewidthTest("quantil95PulsewidthTest");

        }
    }

    namespace ogmServer {
      const std::string parent("/mapServer");
      namespace requests {
        const std::string compressedMapImage("compressedMapImageReq");
        const std::string singleLayerOgm("singleLayerOgmReq");
        // These are the request strings in a singleLayerOgm-server request
        const std::string stockDensity1("stockDensity1");
        const std::string stockDensity2("stockDensity2");
        const std::string stockDensity3("stockDensity3");
        const std::string obstacleNonProcNonAcc("obstacleNonProcNonAcc");
        const std::string obstacleProcNonAcc("obstacleProcNonAcc");
        const std::string obstacleNonProcAcc("obstacleNonProcAcc");
        const std::string stockFlattened("stockFlattened");
        const std::string processed("processed");
        const std::string stockEdge("stockEdge");
      }
    }
    namespace super {
      const std::string ogm("/ism");
      const std::string raw("/map/raw");
    }
    namespace sub {
      const std::string stockDensity1(std::string("/") + ogmServer::requests::stockDensity1);
      const std::string stockDensity2(std::string("/") + ogmServer::requests::stockDensity2);
      const std::string stockDensity3(std::string("/") + ogmServer::requests::stockDensity3);
      const std::string obstacleNonProcNonAcc(std::string("/") + ogmServer::requests::obstacleNonProcNonAcc);
      const std::string obstacleProcNonAcc(std::string("/") + ogmServer::requests::obstacleProcNonAcc);
      const std::string obstacleNonProcAcc(std::string("/") + ogmServer::requests::obstacleNonProcAcc);
      const std::string stockFlattened(std::string("/") + ogmServer::requests::stockFlattened);
      const std::string processed(std::string("/") + ogmServer::requests::processed);
      const std::string stockEdge(std::string("/") + ogmServer::requests::stockEdge);
    }
    const std::string stockDensity1(super::ogm + sub::stockDensity1);
    const std::string stockDensity2(super::ogm + sub::stockDensity2);
    const std::string stockDensity3(super::ogm + sub::stockDensity3);
    const std::string obstacleNonProcNonAcc(super::ogm + sub::obstacleNonProcNonAcc);
    const std::string obstacleProcNonAcc(super::ogm + sub::obstacleProcNonAcc);
    const std::string obstacleNonProcAcc(super::ogm + sub::obstacleNonProcAcc);
    const std::string stockFlattened(super::ogm + sub::stockFlattened);
    const std::string processed(super::ogm + sub::processed);
    const std::string stockEdge(super::ogm + sub::stockEdge);
  }
}

namespace mappingLayers {
const uint NUM_MAPS = 9;

enum maps {
  stockDensity1 = 0,
  stockDensity2,
  stockDensity3,
  obstacleNonProcNonAcc,
  obstacleProcNonAcc,
  obstacleNonProcAcc,
  stockFlattened,
  stockEdge,
  processed  // Overwrites all other properties for visualization and other reasoning
};

static std::string mapSubScopes[NUM_MAPS] {
  scopes::map::sub::stockDensity1,
  scopes::map::sub::stockDensity2,
  scopes::map::sub::stockDensity3,
  scopes::map::sub::obstacleNonProcNonAcc,
  scopes::map::sub::obstacleProcNonAcc,
  scopes::map::sub::obstacleNonProcAcc,
  scopes::map::sub::stockFlattened,
  scopes::map::sub::processed,
  scopes::map::sub::stockEdge
};

static std::string mapRequestScopes[NUM_MAPS] {
  scopes::map::ogmServer::requests::stockDensity1,
  scopes::map::ogmServer::requests::stockDensity2,
  scopes::map::ogmServer::requests::stockDensity3,
  scopes::map::ogmServer::requests::obstacleNonProcNonAcc,
  scopes::map::ogmServer::requests::obstacleProcNonAcc,
  scopes::map::ogmServer::requests::obstacleNonProcAcc,
  scopes::map::ogmServer::requests::stockFlattened,
  scopes::map::ogmServer::requests::processed,
  scopes::map::ogmServer::requests::stockEdge
};

static std::string ismScopes[NUM_MAPS] {
  scopes::map::stockDensity1,
  scopes::map::stockDensity2,
  scopes::map::stockDensity3,
  scopes::map::obstacleNonProcNonAcc,
  scopes::map::obstacleProcNonAcc,
  scopes::map::obstacleNonProcAcc,
  scopes::map::stockFlattened,
  scopes::map::processed,
  scopes::map::stockEdge
};

const int NUM_BGR_CHANNELS = 3;

static unsigned char mapColorBGR[NUM_MAPS][NUM_BGR_CHANNELS] {
  {255,   0,   0}, // Light blue
  {200,   0,   0}, // Mid blue
  {127,   0,   0}, // Dark blue
  {  0,   0, 255}, // Dark red
  { 42,  42, 165}, // Brown
  { 30, 105, 210}, // Chocolat
  {200,   0, 200}, // Purple
  {  0, 255,   0},  // Green
  {  0, 255, 255} // Yellow
//  {  0, 0, 0},
//  {  25, 25, 0},
//  {  50, 50, 0},
//  {  75, 75, 0},
//  {  100, 100, 0},
//  {  125, 125, 0}
};

//const uint NUM_MAPS = 9;
//
//enum maps {
//  stockDensity1,
//  stockDensity2,
//  stockDensity3,
//  obstacleNonProcNonAcc,
//  obstacleProcNonAcc,
//  obstacleNonProcAcc,
//  stockFlattened,
//  stockEdge,
//  processed  // Overwrites all other properties for visualization and other reasoning
//};
//
//static std::string mapSubScopes[NUM_MAPS] {
//  scopes::map::sub::stockDensity1,
//  scopes::map::sub::stockDensity2,
//  scopes::map::sub::stockDensity3,
//  scopes::map::sub::obstacleNonProcNonAcc,
//  scopes::map::sub::obstacleProcNonAcc,
//  scopes::map::sub::obstacleNonProcAcc,
//  scopes::map::sub::stockFlattened,
//  scopes::map::sub::processed,
//  scopes::map::sub::stockEdge
//};
//
//static std::string mapRequestScopes[NUM_MAPS] {
//  scopes::map::ogmServer::requests::stockDensity1,
//  scopes::map::ogmServer::requests::stockDensity2,
//  scopes::map::ogmServer::requests::stockDensity3,
//  scopes::map::ogmServer::requests::obstacleNonProcNonAcc,
//  scopes::map::ogmServer::requests::obstacleProcNonAcc,
//  scopes::map::ogmServer::requests::obstacleNonProcAcc,
//  scopes::map::ogmServer::requests::stockFlattened,
//  scopes::map::ogmServer::requests::processed,
//  scopes::map::ogmServer::requests::stockEdge
//};
//
//static std::string ismScopes[NUM_MAPS] {
//  scopes::map::stockDensity1,
//  scopes::map::stockDensity2,
//  scopes::map::stockDensity3,
//  scopes::map::obstacleNonProcNonAcc,
//  scopes::map::obstacleProcNonAcc,
//  scopes::map::obstacleNonProcAcc,
//  scopes::map::stockFlattened,
//  scopes::map::processed,
//  scopes::map::stockEdge
//};
//
//const int NUM_BGR_CHANNELS = 3;
//
//static unsigned char mapColorBGR[NUM_MAPS][NUM_BGR_CHANNELS] {
//  {255,   0,   0}, // Light blue
//  {200,   0,   0}, // Mid blue
//  {127,   0,   0}, // Dark blue
//  {  0,   0, 255}, // Dark red
//  { 42,  42, 165}, // Brown
//  { 30, 105, 210}, // Chocolat
//  {200,   0, 200}, // Purple
//  {  0, 255,   0},  // Green
//  {  0, 255, 255} // Yellow
//};

}
}
}
#endif /* MS_CONSTANTS_H_ */
