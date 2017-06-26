#ifndef CONSTANTS_MACHINE_HPP_
#define CONSTANTS_MACHINE_HPP_

#include <math.h>
#include "common.hpp"
#include "tf.hpp"
#include "sensor.hpp"

namespace constants {
namespace machine {
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
Eigen::Matrix4d roi_roiOrigin = ctf::trans<double>(-mapping::roi::originWidth,
                                                  -mapping::roi::originHeight,
                                                  0.0);

/** \brief Transformation from the machine frame to the coiler frame*/
// TODO This is static for now, but has to become of course dynamic, with respect to the coiler signals
Eigen::Matrix4d machine_coiler = ctf::trans<double>(-2.1, 1.0, 0.0)
    * ctf::rotY<double>(M_PI) * ctf::rotX<double>(M_PI / 2.0);

/** \brief Transformation from the Leuze sensor to the machine frame*/
Eigen::Matrix4d leuze_machine = ctf::trans<double>(-2.0 * 0.852 * 1.0 / 4.0, 0.0,
                                                  0.0)
    * ctf::rotY<double>(M_PI / 2.0) * ctf::rotX<double>(M_PI / 2.0);

/** \brief Transformation from the machine frame to the Leuze sensor*/
Eigen::Matrix4d machine_leuze = leuze_machine.inverse();

/** \brief Transformation from the machine frame to the IMU on the cabine*/
Eigen::Matrix4d machine_imu = ctf::trans<double>(-2.593, 2.222, 0.0)
    * ctf::rotX<double>(-M_PI / 2.0);

/** \brief Transformation from the machine frame to the INCLINATION frame*/
Eigen::Matrix4d machine_inclination = ctf::trans<double>(-0.436691, -0.703319,
                                                        -0.0665)
    * ctf::rotX<double>(-M_PI / 2.0);

/** \brief Transformation from the machine frame to the footprint of the driven axis frame*/
Eigen::Matrix4d machine_drivenAxisFootprint = ctf::trans<double>(-0.436691,
                                                                -1.7456, 0.0);

/** \brief Transformation from the machine frame to the footprint of the driven axis frame*/
Eigen::Matrix4d machine_wheelOdometry = machine_drivenAxisFootprint;

/** \brief Transformation from the machine frame to the footprint of the driven axis frame*/
Eigen::Matrix4d machine_gps = machine_drivenAxisFootprint;

/** \brief mcs-dachhalter_c.png: Transformation from the machine frame to the horns*/
Eigen::Matrix4d machine_horns = ctf::trans<double>(-2.633343, 2.1994, 0.0);

//     /** \brief Transformation from the horns to the LASE1 base plate (lower one without the thickness (4mm) of the mounting plate for both LASE sensors)*/
//     Eigen::Matrix4d horns_lase1base = ctf::trans<double>(-0.124451 + (0.004*cos(29.758*M_PI/180.0)), -0.081424 - (0.004*sin(29.758*M_PI/180.0)), 0.0545) * ctf::rotY<double>(M_PI) * ctf::rotX<double>(M_PI / 2.0) * ctf::rotY<double>(-29.758 * M_PI / 180.0);

/** \brief dachhalter-lase_u_basis_c.png: Transformation from the horns to the LASE1 base plate (lower oneincluding the mounting plate for both LASE sensors)*/
Eigen::Matrix4d horns_lase1base = ctf::trans<double>(-0.124451, -0.081424,
                                                    0.0545)
    * ctf::rotY<double>(M_PI) * ctf::rotX<double>(M_PI / 2.0)
    * ctf::rotY<double>(-29.758 * M_PI / 180.0);

/** \brief lase-lase_orig_c.png: Transformation from the LASE1 base plate to the LASE1 sensor frame*/
Eigen::Matrix4d lase1base_lase1 = ctf::trans<double>(0.075, 0.0545, 0.0);

/** \brief dachhalter-lase_o_basis_c.png: Transformation from the horns to the LASE2 base plate (upper one)*/
Eigen::Matrix4d horns_lase2base = ctf::trans<double>(-0.206489, 0.062068, 0.0545)
    * ctf::rotY<double>(M_PI) * ctf::rotX<double>(M_PI / 2.0)
    * ctf::rotY<double>(-29.758 * M_PI / 180.0);

/** \brief Transformation from the LASE2 base plate to the LASE2 sensor frame*/
Eigen::Matrix4d lase2base_lase2 = lase1base_lase1;

/** \brief dachhalter-sick_c.png: Transformation from the horns to the SICK1 base plate*/
Eigen::Matrix4d horns_sick1base = ctf::trans<double>(-0.143544, -0.105507,
                                                    -0.230293)
    * ctf::rotY<double>(-M_PI / 2.0) * ctf::rotX<double>(14.613 * M_PI / 180.0);

/** \brief sick-sick_orig-c.png: Transformation from the SICK1 base plate to the SICK1 sensor frame*/
Eigen::Matrix4d sick1base_sick1 = ctf::trans<double>(0.024, 0.0, 0.044)
    * ctf::rotY<double>(-M_PI / 2.0) * ctf::rotX<double>(M_PI / 2.0)
    * ctf::rotX<double>(M_PI);

/** \brief dachhalter-sick_c.png: Transformation from the horns to the SICK2 base plate*/
Eigen::Matrix4d horns_sick2base = ctf::trans<double>(-0.149531, -0.092189,
                                                    0.284711)
    * ctf::rotY<double>(-M_PI / 2.0) * ctf::rotX<double>(33.901 * M_PI / 180.0);

/** \brief Transformation from the SICK2 base plate to the SICK2 sensor frame*/
Eigen::Matrix4d sick2base_sick2 = sick1base_sick1;

// HACK: This is a direct transformation from MCU to the sensor frame
/** \brief Transformation from the machine frame to the origin of SICK3*/
Eigen::Matrix4d machine_sick3 = ctf::trans<double>(5.970, 1.601521, 0.166)
    * ctf::rotX<double>(M_PI / 2.0)
    * ctf::rotY<double>((-45 - 13.5) * M_PI / 180.0);

/** \brief Transformation from the horns to the IBEO base plate*/
Eigen::Matrix4d horns_ibeoBase = ctf::trans<double>(0.4, 0.2, -0.1);

/** \brief Transformation from the IBEO base plate to the IBEO sensor frame*/
Eigen::Matrix4d ibeoBase_ibeo = ctf::trans<double>(0.1, 0.0, 0.0)
    * ctf::rotY<double>(30.0 * M_PI / 180.0);

/** \brief Transformation from the horns to the SICK3 base plate*/
Eigen::Matrix4d horns_sick3base = ctf::trans<double>(0.1, 0.2, -0.1);

/** \brief Transformation from the horns to the monochrome camera base plate*/
Eigen::Matrix4d horns_mCamBase = ctf::trans<double>(0.1, 0.0, -0.1);

/** \brief Transformation from the monochrome camera base plate to the monochrome camera frame*/
Eigen::Matrix4d mCamBase_mCam = ctf::trans<double>(0.1, 0.0, 0.0)
    * ctf::rotY<double>(30.0 * M_PI / 180.0) * ctf::rotY<double>(M_PI / 2.0);

/** \brief Transformation from the footprint of the machine coordinate system to the machine coordinate system itself */
//    Eigen::Matrix4d machineRoi_machine = ctf::trans<double>(0.0, 0.0, 1.7456) * ctf::rotX<double>(-M_PI/2.0) * ctf::rotZ<double>(M_PI);
Eigen::Matrix4d machineRoi_machine = ctf::trans<double>(0.0, 0.0, 1.7456)
    * ctf::rotX<double>(M_PI / 2.0) * ctf::rotY<double>(M_PI);
Eigen::Matrix4d baseLink_machine = machineRoi_machine;

/** \brief Transformation from the horns to the IR camera base plate*/
Eigen::Matrix4d horns_irCamBase = ctf::trans<double>(0.1, 0.0, -0.4);

/** \brief Transformation from the IR camera base plate to the IR camera frame*/
Eigen::Matrix4d irCamBase_irCam = ctf::trans<double>(0.1, 0.0, 0.0)
    * ctf::rotY<double>(30.0 * M_PI / 180.0) * ctf::rotY<double>(M_PI / 2.0);

/** \brief Transformation from the horns to the color camera base plate*/
Eigen::Matrix4d horns_cCamBase = ctf::trans<double>(0.1, -0.4, -0.1);

/** \brief Transformation from the color camera base plate to the color camera frame*/
Eigen::Matrix4d cCamBase_cCam = ctf::trans<double>(0.1, 0.0, 0.0)
    * ctf::rotY<double>(30.0 * M_PI / 180.0) * ctf::rotY<double>(M_PI / 2.0);

/** \brief Transformation from the machine frame to the PF sensor*/
Eigen::Matrix4d machine_pf = ctf::trans<double>(2, -0.1, 2.5)
    * ctf::rotX<double>(M_PI) * ctf::rotY<double>(M_PI / 2.0);

/** \brief Transformation from the machine frame to the back base plate*/
Eigen::Matrix4d machine_backBase = ctf::trans<double>(5, -2.1, 0.0)
    * ctf::rotX<double>(M_PI / 2.0);

/** \brief Transformation from the back base plate to the SICK3 sensor frame*/
Eigen::Matrix4d backBase_sick3 = ctf::trans<double>(0.1, 0.0, 0.0)
    * ctf::rotY<double>(30.0 * M_PI / 180.0);

/** \brief mcs-hok-c.png: Transformation from the machine frame to the Hokuyo base plate*/
Eigen::Matrix4d machine_hokuyoBase = ctf::trans<double>(6.492219, -0.319418, 0.0)
    * ctf::rotZ<double>(M_PI / 2.0) * ctf::rotX<double>(-M_PI / 2.0);

/** \brief Transformation from the Hokuyo base plate Hokuyo laser scanner*/
// HACK The sensor is not correctly oriented by its messages, thus we doing some nasty stuff here with the rotation
Eigen::Matrix4d hokuyoBase_hokuyo = ctf::trans<double>(0.0, 0.0, 0.1)
    * ctf::rotZ<double>(M_PI / 2.0 + M_PI / 8.0 + M_PI / 16.0 - M_PI / 64.0);

/** \brief Transformation for the different Sick LDMRS layers*/
Eigen::Matrix4d sick_layer1 = ctf::rotY<double>(
    -(sensors::SICK_LDMRS::layer1Inclination_rad));
Eigen::Matrix4d sick_layer2 = ctf::rotY<double>(
    -(sensors::SICK_LDMRS::layer2Inclination_rad));
Eigen::Matrix4d sick_layer3 = ctf::rotY<double>(
    -(sensors::SICK_LDMRS::layer3Inclination_rad));
Eigen::Matrix4d sick_layer4 = ctf::rotY<double>(
    -(sensors::SICK_LDMRS::layer4Inclination_rad));
}
namespace coiler {
/** \brief Width of the coiler*/
const double width = 9.30;  // meter

/** \brief Depth of the coiler*/
const double depth = 1.2;  // meter
}
}
}

#endif // CONSTANTS_MACHINE_HPP_
