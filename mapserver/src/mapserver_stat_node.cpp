
#include <mapserver_stat.hpp>

//#include <opencv2/opencv.hpp>
//#include <boost/math/special_functions/erf.hpp>
//#include <exception>
//#include <algorithm>
//#include <Eigen/Dense>
//#include <mutex>
//#include <thread>
//#include <fstream>
//#include <functional>
//#include <iostream>
//std::mutex mtxSwap, mtxShowRpc, mtxShowIsm;


// ROS
#include <ros/ros.h>
//#include <ros/spinner.h>
//#include <ros/console.h>
//#include <nav_msgs/OccupancyGrid.h>
//#include <std_msgs/String.h>
//#include <mapserver_msgs/pnsTuple.h>
//#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/Point.h>
//#include <sensor_msgs/PointCloud.h>
//#include <rosgraph_msgs/Log.h>
//#include <tf/transform_listener.h>
//#include <mapserver/rsm.h>
//#include <mapserver/ismStackFloat.h>
//#include <tf_conversions/tf_eigen.h>
//#include <rosapi/Topics.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Point.h>
//#include <std_msgs/String.h>
//#include <geometry_msgs/PointStamped.h>
//#include <tf/transform_broadcaster.h>
//#include <ros/duration.h>
//
//
//
//// Stdandard libraries
//#include <mutex>          // std::mutex
//#include <future>
//#include <math.h>
//#include <string>
//#include <iostream>
//#include <sstream>
//#include <algorithm>    // std::min
//#include <exception>
//#include <map>
//#include <memory>
//
//// Boost
//#include <boost/thread.hpp>
//#include <boost/program_options.hpp>
//#include <boost/shared_ptr.hpp>


//#include <Constants.h>
//#include <utils.h>
//using namespace ms::constants;
//using namespace ms::constants::mappingLayers;
//namespace msNumeric = ms::constants::numeric;
//
//// Concatenated map scopes with the correct prefix, which is set by program options
//static std::string mapScopes[mappingLayers::NUM_MAPS];
//
//// Program options
//  // Properties of a single occupancy grid map
//  static std::string currentTileTfName(""), lastTileTfName("");
//  static std::string tileOriginTfPrefix, tileOriginTfSufixForRoiOrigin, currentTfNameTopic, currentTupleTopic, worldLink, storeMapsTopic, reqTopicMapStack, debugIsmTopic;
//  static float maxOccupancyUpdateCertainty = mapping::ogm::maxOccupancyUpdateCertainty;
//  static int debug = 0;
//  static float rate = 1;
//  static std::string ismScopePrefix = scopes::map::super::ogm;
//
//// Global variables
//  // The map stack
//  static std::vector<mrpt::maps::COccupancyGridMap2D> mapStack;  // Current stack for fusion
//  static std::vector<mrpt::maps::COccupancyGridMap2D> mapStackStorageTemp;  // Temp stack for hdd storage
//  static std::vector<mrpt::maps::COccupancyGridMap2D> mapStackShiftTemp; // Temp stack for shifting
//
//  // Discrete resolution of the map
//  static std::string mapStorageLocation = mapping::ogm::mapStorageLocation;


int main(int argc, char **argv){

  // ROS
  ros::init(argc, argv, "mapserver_stat");
  ros::NodeHandle n("~");

  MapserverStat ms(n);
  ms.spin();

  return 0;
}
