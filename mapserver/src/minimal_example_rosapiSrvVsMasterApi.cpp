// Message formating
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// MRPT
//#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
//#include <mrpt/maps.h>
//#include <mrpt/opengl.h>
//#include <mrpt/gui.h>

// CLAAS
#include <opencv2/opencv.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <exception>
#include <algorithm>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <fstream>
#include <functional>
#include <iostream>
std::mutex mtxSwap, mtxShowRpc;

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Log.h>
#include <tf/transform_listener.h>
#include <mapserver/rsm.h>
#include <tf_conversions/tf_eigen.h>
#include <rosapi/Topics.h>

ros::Publisher publisherMap;
ros::Subscriber subscriberIsm, subscriberTfTileName;
tf::TransformListener *listenerTf;

// OpenCV
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>  // resize

// Stdandard libraries
#include <mutex>          // std::mutex
#include <future>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>    // std::min
#include <exception>
#include <map>

// Boost
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <Constants.h>
#include <utils.h>
using namespace claas::constants;
using namespace claas::constants::mappingLayers;
namespace claasNumeric = claas::constants::numeric;

// Concatenated map scopes with the correct prefix, which is set by program options
static std::string mapScopes[mappingLayers::NUM_MAPS];

// Program options
// Properties of a single occupancy grid map
static std::string currentTileTfName(""), lastTileTfName("");
static std::string topicMap, topicLaser, tileOriginTfPrefix, currentTfNameTopic,
    worldLink;
static double idleStartupTime_s;
static float resolution = mapping::discreteResolution;
static float maxOccupancyUpdateCertainty =
    mapping::ogm::maxOccupancyUpdateCertainty;
static float maxDistanceInsertion = std::min(mapping::roi::width,
                                             mapping::roi::height);
static float max_x = mapping::roi::xMax;
static float min_x = mapping::roi::xMin;
static float max_y = mapping::roi::yMax;
static float min_y = mapping::roi::yMin;
static int debug = 0;
static int doTest = 0;
static float rate = 1;
static std::string ismScopePrefix = scopes::map::super::ogm;
static std::string machineModelScope = scopes::odometry;
// Marging for non present information: 0.45 .. 0.55 -> 0.1
static float uncertaintyBoundary = mapping::ogm::minDrawOccupancyUpdateCertainty;
static float mapInitValue = mapping::ogm::unknownOccupancyUpdateCertainty;

// Global variables
// The map stack
static std::vector<mrpt::maps::COccupancyGridMap2D> mapStack;  // Current stack for fusion
static std::vector<mrpt::maps::COccupancyGridMap2D> mapStackStorageTemp;  // Temp stack for hdd storage
static std::vector<mrpt::maps::COccupancyGridMap2D> mapStackShiftTemp;  // Temp stack for shifting
static mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
// Discrete resolution of the map
static int mapSizeX;
static int mapSizeY;
static cv::Mat storageMapBuffer;
static std::string mapStorageLocation = mapping::ogm::mapStorageLocation;
static int shiftMap = 1;  // Shift (true) or clean (false) the map if center_dresch is received

// For sending the localization
// rsb::Informer<rst::geometry::PoseEuler>::Ptr informerOdometry;

// Actual position
// rsb::Informer<rst::geometry::PoseEuler>::DataPtr odomData(new rst::geometry::PoseEuler);

using namespace boost;
using namespace std;

std::mutex mapRefresh;
std::mutex mtxOdom;       // mutex for odometry messages

// Set all map tiles of all maps to the given value
void fillMapStack(std::vector<mrpt::maps::COccupancyGridMap2D> &mapStack,
                  float value = mapping::ogm::unknownOccupancyUpdateCertainty) {
  for (uint idx = 0; idx < mapStack.size(); ++idx) {
    mapStack[idx].fill(value);  // Fill every map with uncertainty
  }
}

// Translate a map
void translateMap(mrpt::maps::COccupancyGridMap2D &map, int offsetx = 0,
                  int offsety = 0, float fillProbability =
                      mapping::ogm::unknownOccupancyUpdateCertainty) {

  // Define a tranformation for the image
  cv::Mat trans_mat =
      (cv::Mat_<double>(2, 3) << 1, 0, static_cast<double>(offsetx), 0, 1, static_cast<double>(offsety));

  // Get the raw data as an image
  cv::Mat mapAsImage(static_cast<int>(map.getSizeY()),
                     static_cast<int>(map.getSizeX()),
                     CV_8SC1,
                     (void*) &map.getRawMap()[0]);

  // Warp the image (which refers to the map)
  cv::warpAffine(
      mapAsImage,
      mapAsImage,
      trans_mat,
      cv::Size(static_cast<int>(map.getSizeX()),
               static_cast<int>(map.getSizeY())),
      cv::INTER_NEAREST,
      cv::BORDER_CONSTANT,
      cv::Scalar(
          static_cast<double>(mrpt::maps::COccupancyGridMap2D::p2l(
              fillProbability))));
}

// Translate a stack of maps
void translateMapStack(std::vector<mrpt::maps::COccupancyGridMap2D> &mapStack,
                       int offsetx = 0, int offsety = 0, float fillProbability =
                           mapping::ogm::unknownOccupancyUpdateCertainty) {
  for (std::size_t mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    translateMap(mapStack[mapIdx], offsetx, offsety, fillProbability);
  }
}

// Add just some picture of each other
boost::shared_ptr<cv::Mat> doColorMapCallback() {

  // TODO Define this as a global variable so that it dows not
  // have to allocate space on every call
  boost::shared_ptr<cv::Mat> dst(new cv::Mat(mapSizeX, mapSizeY, CV_8UC3));
  dst->setTo(cv::Scalar(127, 127, 127));  // to set all values to 127 (aka unknown)

  const float validCellValue = uncertaintyBoundary;
  for (uint mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    for (int idy = 0; idy < mapSizeY; ++idy) {
      for (int idx = 0; idx < mapSizeX; ++idx) {
        if (mapStack[mapIdx].getCell(idx, idy) > validCellValue) {
          dst->at<cv::Vec3b>(idy, idx)[0] =
              mappingLayers::mapColorBGR[mapIdx][0];
          dst->at<cv::Vec3b>(idy, idx)[1] =
              mappingLayers::mapColorBGR[mapIdx][1];
          dst->at<cv::Vec3b>(idy, idx)[2] =
              mappingLayers::mapColorBGR[mapIdx][2];
        }
      }
    }
  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
}
;

//// TODO I need the location of the ROI in the global coordinate system to sore it as a filename
//void mapRefreshAndStorage(rst::claas::MachineModel_Odemetrie machinaModel, rst::claas::MachineModel_Odemetrie lastMachineModelWithCenterRoi, uint64_t timestamp) {
//  // TODO shift it in the current view
//  // Refresh the map
//  mapRefresh.lock();
//  // Make a backup of the map for storage
//  for (uint mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
//    // TODO Replace by memcopy or getRawMap
//    mapStackStorageTemp[mapIdx] = mapStack[mapIdx];
//  }
//
//  if (shiftMap) { // Shift the map instead of resetting
//    // Caclulate the shift indices from [xmin, xmax) and [ymin, ymax)
//    const int xshift = -static_cast<int>(lastMachineModelWithCenterRoi.x_dis_roi());
//    const int yshift = -static_cast<int>(lastMachineModelWithCenterRoi.y_dis_roi());
//
//    // Shift the map
//    if (debug) {
//      DEBUG_MSG("--- Shift the map: x= " << xshift << ", y= " << yshift)
//    }
//    translateMapStack(mapStack, xshift, yshift);
//
//  } else {
//    // Clean the map
//    fillMapStack(mapStack);
//  }
//  mapRefresh.unlock();
//
//  // Clean the temporal storage for shifting the next time
//  fillMapStack(mapStackShiftTemp, mapInitValue);
//
//  // Get the unix timestamp as string for the sorage name
//  std::ostringstream oss;
//  oss << timestamp;
//  std::string timestampString(oss.str());
//
//  // Get the compression parameter
//  vector<int> compression_params;
//  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//  compression_params.push_back(3);
//
//  for (uint mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
//    // Convert it
//    for (int idy = 0; idy < mapSizeY; ++idy) {
//      for (int idx = 0; idx < mapSizeX; ++idx) {
//        // TODO Replace getCell by a faster function
//        storageMapBuffer.at<uchar>(idy,idx) = static_cast<uchar>(mapStackStorageTemp[mapIdx].getCell(idx, idy) * 255.0f);
//      }
//    }
////    // Define the filename and store it
////    std::string filename(mapStorageLocation);
////    filename.append(timestampString).append("_").append(mapSubScopes[mapIdx], 1, std::string::npos).append(".bmp");
////    if (debug) {
////      DEBUG_MSG("Store map to: " << filename)
////    }
////    cv::imwrite(filename, storageMapBuffer);
//
//    // Get the unix timestamp as string for the storage name
//    std::ostringstream oss;
//    oss << timestamp << mapSubScopes[mapIdx].substr(1, std::string::npos) << "_Idx" << mapIdx
//        << "_Grid" << resolution * geometry::millimeterPerMeter <<  "MM"
//        << "_X" << lastMachineModelWithCenterRoi.x_kon_glo() - lastMachineModelWithCenterRoi.x_kon_roi()
//        << "_Y" << lastMachineModelWithCenterRoi.y_kon_glo() - lastMachineModelWithCenterRoi.y_kon_roi();
//    std::string timestampString(oss.str());
//    std::cout << "-------------- timestampString: " << timestampString << std::endl;
//
//    // Define the filename and store it
//    std::string filename(mapStorageLocation);
//    filename.append(timestampString).append(".png");
//    if (debug) {// Get the unix timestamp as string for the sorage name
//      std::ostringstream oss;
//      oss << timestamp << "_" << mapIdx;
//      std::string timestampString(oss.str());
//      DEBUG_MSG("Store map to: " << filename)
//    }
//
//    // Store the layer
//    cv::imwrite(filename, storageMapBuffer, compression_params);
//  }
//
//}

//void doIsmFusion(const ros::MessageEvent<nav_msgs::OccupancyGrid>& event) {
//
//  std::cerr << event.getConnectionHeader().size() << std::endl;
//}

// Get topic name with callback: http://answers.ros.org/question/68434/get-topic-name-in-callback/?answer=68545#post-id-68545
// Using bind function: http://en.cppreference.com/w/cpp/utility/functional/bind
void doIsmFusion(const std_msgs::String::ConstPtr &msg,
                 const std::string &topic) {
  std::cerr << "Content: " << *msg << std::endl;
  std::cerr << "Topic:   " << topic << std::endl;
}

//// RSB Server function for the mapping server which replies with a compressed image map of the environment
//// TODO Add view.proto
//class mapServerCompressedMapImage: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
//public:
//  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
//    INFO_MSG("Color map request received.");
//    return doColorMapCallback();
//  }
//};

//// Unpack the requested layer to a single OGM and return it
//boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> doOgmSingleLayerCallback(std::string requestedLayerName) {
//  boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> requestedLayer(new rst::navigation::OccupancyGrid2DInt);
//
//  // Get the layer name
//  int requestedLayerIdx = -1;
//  for (uint layerIdx = 0; layerIdx < NUM_MAPS; ++layerIdx) {
//    if (requestedLayerName.compare(mapRequestScopes[layerIdx]) == 0) {
//      requestedLayerIdx = layerIdx;
//      break;
//    }
//  }
//  if (requestedLayerIdx < 0) {
//    throw invalid_argument("Requested layer" + requestedLayerName + "does not exist");
//  }
//
//  DEBUG_MSG("Request layer number: " << requestedLayerIdx)
//
//  // Copy the map layer
//  // TODO Is the order of idy and idx correct? (Check also the compressedColorRequest!)
//  requestedLayer->set_height(mapSizeY);
//  requestedLayer->set_width(mapSizeX);
//  requestedLayer->set_resolution(mapping::discreteResolution);
//  requestedLayer->mutable_map()->resize(mapSizeY * mapSizeX, -1);
//  for (int idy = 0; idy < mapSizeY; ++idy) {
//    for (int idx = 0; idx < mapSizeX; ++idx) {
//      // We get values from 0.0 .. 1.0 and have to map it to 0 .. 100;
////      INFO_MSG("idx " << idx << " / " << mapSizeX)
////      INFO_MSG("idy " << idy << " / " << mapSizeY)
//      requestedLayer->mutable_map()->at(idy * mapSizeX + idx) =
//          char(mapStack[requestedLayerIdx].getCell(idx, idy) * 100.0);
//    }
//  }
//
//  // Set the pose of the map
//  // TODO For now it is the ROI Origin, but it will become something different, when we
//  // TODO introduce view.proto
//  rst::geometry::Pose *layerPose = requestedLayer->mutable_origin();
//  layerPose->mutable_rotation()->set_qx(0.0);
//  layerPose->mutable_rotation()->set_qy(0.0);
//  layerPose->mutable_rotation()->set_qz(0.0);
//  layerPose->mutable_rotation()->set_qw(1.0);
//  layerPose->mutable_translation()->set_x(-mapping::roi::originWidth);
//  layerPose->mutable_translation()->set_y(-mapping::roi::originHeight);
//  layerPose->mutable_translation()->set_z(0.0);
//
//  return requestedLayer;
//}

nav_msgs::OccupancyGrid getIsmOutput(
    const mapserver_msgs::mapPrimitive &view,
    const mrpt::maps::COccupancyGridMap2D &input) {

  cv::Mat statisticMat, croppedMat; /*float matrices*/
  nav_msgs::OccupancyGrid output;

  std::stringstream os;
  os << view;
  ROS_INFO("view:\n%s\n", os.str().c_str());

  // Calculate the requested ROI
  const double xView = view.pose.pose.position.x;  // Meter
  const double yView = view.pose.pose.position.y;  // Meter
  const double wView = view.width * view.resolution;  // Tiles * meter/tiles
  const double hView = view.height * view.resolution;  // Tiles * meter/tiles
  const double dView = view.depth * view.resolution;  // Tiles * meter/tiles
  const std::string sourceframe =
      view.frame_id.empty() ? machine::frames::names::BASE_LINK : view.frame_id;
  tf::Quaternion q;
  tf::quaternionMsgToTF(view.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  const double zRotView = yaw;

  // Resize the resolution
  cv::Size size(view.width, view.depth);
  cv::resize(statisticMat, croppedMat, size, 0, 0, cv::INTER_NEAREST);

  // Copy the meta information
  output.header.stamp = ros::Time::now();
  output.header.frame_id = sourceframe;
  output.info.resolution = view.resolution;
  output.info.width = croppedMat.cols;
  output.info.height = croppedMat.rows;
  output.info.origin = view.pose.pose;
  const int arraySize = croppedMat.cols * croppedMat.rows;

  // Copy everything to output
  output.data.resize(arraySize);
  for (int idx = 0; idx < arraySize; ++idx) {
    output.data.at(idx) = croppedMat.at<char>(idx);
  }

  return output;
}

//class mapServerStockEdge: public rsb::patterns::LocalServer::Callback<rst::claas::mapPrimitive, rst::navigation::OccupancyGrid2DInt> {
//  boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> call(const std::string& /*methodName*/,
//              boost::shared_ptr<rst::claas::mapPrimitive> input) {
//        INFO_MSG( "mapServer-stockEdge method called" );
//        const boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> output(getIsmOutput(input, mapStack.at(claas::constants::mappingLayers::maps::stockEdge)));
//        INFO_MSG( "mapServer-stockEdge method called: Finish" );
//        return output;
//    }
//};

// RSB Server function for the mapping server which replies with a OGM map of the environment
// TODO Add view.proto
//class mapServerSingleLayerOgm: public rsb::patterns::LocalServer::Callback<std::string, rst::navigation::OccupancyGrid2DInt> {
//public:
//  boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> call(const std::string& /*methodName*/,
//                                                              boost::shared_ptr<std::string> requestedLayerName) {
//    INFO_MSG("OGM single layer request received.");
//    return doOgmSingleLayerCallback(*requestedLayerName);
//  }
//};

void testFillMap() {

  // Define the map as a sensor update
  mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
  def.resolution = resolution;
  def.insertionOpts.maxOccupancyUpdateCertainty = maxOccupancyUpdateCertainty;
  def.insertionOpts.maxDistanceInsertion = maxDistanceInsertion;
  def.max_x = 4.0f;
  def.min_x = 0.0f;
  def.max_y = 3.0f;
  def.min_y = 0.0f;

  // Create the map from definition
  mrpt::maps::COccupancyGridMap2D sensorUpdate =
      *mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def);

  // Fill it with some occupied value
  sensorUpdate.fill(0.8);

  // Update a sensor scan:
  // The transformation already points to the lower left edge of the sensor scan
  const float transX = 2.0f;
  const float transY = 5.0f;

  int mapOffsetXStart = transX / def.resolution;
  int mapOffsetXEnd = (transX + def.max_x - def.min_x) / def.resolution;
  int mapOffsetYStart = transY / def.resolution;
  int mapOffsetYEnd = (transY + def.max_y - def.min_y) / def.resolution;

  // Iterate through every map layer
  for (uint mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    // Do a pointwise sensor fusion for every layer
    int mapOffsetXStart = transX / def.resolution * (1 + mapIdx);
    int mapOffsetXEnd = (transX + def.max_x - def.min_x) / def.resolution
        * (1 + mapIdx);
    for (int idy = mapOffsetYStart; idy < mapOffsetYEnd; ++idy) {
      for (int idx = mapOffsetXStart; idx < mapOffsetXEnd; ++idx) {
        mapStack[mapIdx].updateCell(
            idx, idy,
            sensorUpdate.getCell(idx - mapOffsetXStart, idy - mapOffsetYStart));
      }
    }
  }
}

///
/// \brief Store the current tf tile name
/// \param nameMsg Name of the current tile tf
///
void tfTileNameHandler(const std_msgs::String nameMsg) {
  bool currentTileTfNameChange = false;
  mtxSwap.lock();
  if ((nameMsg.data.back() != currentTileTfName.back())) {
    if (currentTileTfName.empty()) {
      // First round, we bootstrap
      currentTileTfName = nameMsg.data;
    } else {
//        std::swap(currentMapHeight_mm, lastMapHeight_mm);
//        std::swap(currentPulsewidth_ps, lastPulsewidth_ps);
//        std::swap(currentMapIterator, lastMapIterator);
      lastTileTfName = currentTileTfName;
      currentTileTfName = nameMsg.data;
      currentTileTfNameChange = true;
    }
  }

  if (currentTileTfNameChange) {
    tf::StampedTransform transformRoiInWorld;
    try {
      listenerTf->waitForTransform(lastTileTfName, worldLink, ros::Time(0.0),
                                   ros::Duration(3.0));
      listenerTf->lookupTransform(lastTileTfName, worldLink, ros::Time(0.0),
                                  transformRoiInWorld);
    } catch (const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("tfTileNameHandler: %s", excStr.c_str());
      mtxSwap.unlock();
      return;
    }
    ROS_DEBUG("NEW MAP");

    // Wait until all references are gone
    std::size_t lockCnt = 0;
    const std::size_t lockCntMax = 200000;  // 2 seconds if we sleep for 10 us
//    while(!(lastMapHeight_mm.unique() && currentPulsewidth_ps.unique() && currentMapIterator.unique())) {
//        usleep(10);
//        if (++lockCnt > lockCntMax) {
//            ROS_ERROR("tfTileNameHandler: Locked for to long, skip storage (maybe deadlock or out if resources?)");
//            mtxSwap.unlock();
//            return;
//        }
//    }

//    mapRefreshAndStorage( lastMapHeight_mm,
//                          currentMapHeight_mm,
//                          transformRoiInWorld,
//                          std::string("height"),
//                          std::string(""),
//                          std::string("mm"),
//                          0,
//                          !dontStoreMaps,
//                          bool(shiftMap),
//                          !shiftMap,
//                          claasNumeric::invalidValue_int16);
  }

  mtxSwap.unlock();

}

void listTopics(const rosgraph_msgs::Log::ConstPtr &msg) {
  std::stringstream ss;
  ss << *msg;
  ROS_INFO("\n%s\n", ss.str().c_str());
}

int main(int argc, char **argv) {

  // ROS
  ros::init(argc, argv, "mapserver_stat");
  ros::NodeHandle n("~");

  n.param<std::string>("tile_origin_tf_prefix", tileOriginTfPrefix,
                       "map_base_link_");
  n.param<std::string>("current_tf_name_topic", currentTfNameTopic,
                       "/currentTfTile");
  n.param<std::string>("world_link", worldLink, "odom");
  n.param<double>("idle_startup_time", idleStartupTime_s, -1.0);  // Wait before mapping (< 0 to disable)

  n.param<int>("debug", debug, 0);  // Enable debug outputs
  n.param<int>("test", doTest, 0);  // Enable testing
  n.param<float>("resolution", resolution, mapping::discreteResolution);  // Resolution of map in meter/cell
  n.param<float>("max_occupanc_update_certainty", maxOccupancyUpdateCertainty,
                 0);  // Maximum update uncertainty
  n.param<float>("max_distance_insertion", maxDistanceInsertion, 0);  // Maximum distance insertion
  n.param<float>("max_x", max_x, mapping::roi::xMax);  // Maxmium value of x in meter
  n.param<float>("min_x", min_x, mapping::roi::xMin);  // Minimum value of x in meter
  n.param<float>("max_y", max_y, mapping::roi::yMax);  // Maxmium value of y in meter
  n.param<float>("min_y", min_y, mapping::roi::yMin);  // Minimum value of y in meter
  n.param<float>("uncertainty_boundary", uncertaintyBoundary, 0);  // Uncertainty boundary for displaying a feature (standard: 0.5 (unknown))
  n.param<float>("mapInit_value", mapInitValue, 0);  // Probability (0 .. 1.0) for initializing the map
  n.param<std::string>("machine_model_topic", machineModelScope, "");  // Scope for receiving the machine models (odometry)
  n.param<std::string>("ism_scope_prefix", ismScopePrefix, "");  // Scope prefix for the inverse sensor models
  n.param<std::string>("map_storage_location", mapStorageLocation, "/tmp");  // Location for the maps to store (preceeding / is needed, like /tmp/)
  n.param<int>("shift_map", shiftMap, 0);  // Shift (1) or wipe (0) the map on center_dresch
  n.param<float>("rate", rate, 1);  // Rate for publishing debug information

  INFO_MSG("Enable debug outputs: " << debug)
  INFO_MSG("Enable testing: " << doTest)
  INFO_MSG("Resolution of map in meter/cell: " << resolution)
  INFO_MSG("Maximum update uncertainty: " << maxOccupancyUpdateCertainty)
  INFO_MSG("Maximum distance insertion: " << maxDistanceInsertion)
  INFO_MSG("Maxmium value of x in meter: " << max_x)
  INFO_MSG("Minimum value of x in meter: " << min_x)
  INFO_MSG("Maxmium value of y in meter: " << max_y)
  INFO_MSG("Minimum value of y in meter: " << min_y)
  INFO_MSG(
      "Uncertainty boundary for displaying a feature (standard: 0.5 (unknown)): "
          << uncertaintyBoundary)
  INFO_MSG("Probability (0 .. 1.0) for initializing the map: " << mapInitValue)
  if (mapInitValue > uncertaintyBoundary)
  WARNING_MSG(
      "'uncertaintyBoundary' is less than 'mapInitValue', display might be corrupted")
  INFO_MSG(
      "Scope for receiving the machine models (odometry): "
          << machineModelScope)
  INFO_MSG("Scope prefix for the inverse sensor models: " << ismScopePrefix)
  for (uint idx = 0; idx < NUM_MAPS; ++idx) {
    mapScopes[idx] = std::string(ismScopePrefix).append(mapSubScopes[idx]);
    INFO_MSG("Scope for " << mapSubScopes[idx] << ": " << mapScopes[idx])
  }
  INFO_MSG("Location for the maps to store: " << mapStorageLocation)

  // Define the properties of one occupancy map
  def.resolution = resolution;
  def.insertionOpts.maxOccupancyUpdateCertainty = maxOccupancyUpdateCertainty;
  def.insertionOpts.maxDistanceInsertion = maxDistanceInsertion;
  def.max_x = max_x;
  def.min_x = min_x;
  def.max_y = max_y;
  def.min_y = min_y;

  // Store discrete resolution of the map
  mapSizeX = (def.max_x - def.min_x) / def.resolution;
  mapSizeY = (def.max_y - def.min_y) / def.resolution;

  // Allocate space for the map
  storageMapBuffer = cv::Mat(mapSizeX, mapSizeY, CV_8UC1);

  // Create the maps
//  for (unsigned char mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
//    mapStack.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
//    mapStackStorageTemp.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
//    mapStackShiftTemp.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
//  }
//  fillMapStack(mapStack, mapInitValue);
//  fillMapStack(mapStackStorageTemp, mapInitValue);
//  fillMapStack(mapStackShiftTemp, mapInitValue);

  if (doTest) {
    INFO_MSG(
        "Test the mapstructure by printing rectangulars in every layer an printing them")
    // Fill all the maps with small rectangles which are overlapping a bit
    testFillMap();
    // Get the map as image
    boost::shared_ptr<cv::Mat> image(doColorMapCallback());
    imshow("Display window", *image);               // Show our image inside it.
    cv::waitKey(0);                        // Wait for a keystroke in the window
    INFO_MSG("Exit")
    return 0;
  }

  // Prepare subscriber for all future OGM
//  subscriberIsm = n.subscribe<nav_msgs::OccupancyGrid>("TOPIC", 100, doIsmFusion);
  subscriberIsm = n.subscribe<std_msgs::String>(
      "/processed", 100,
      std::bind(doIsmFusion, std::placeholders::_1, "processed"));
//  ros::Subscriber subscriberLog = n.subscribe<rosgraph_msgs::Log>("/rosout_agg", 100, listTopics);
//  subscriberTfTileName = n.subscribe<std_msgs::String>(currentTfNameTopic, 2, tfTileNameHandler);

  // Prepare ROS service
  // TODO: Do this on demand for given subscribed topics
  const std::string s("/");
//  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::compressedMapImage , mapServerCompressedMap);
//  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::stockEdge , mapServerStockEdge);
//  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::singleLayerOgm , mapServerSingleLayerOgm);

  ros::AsyncSpinner spinner(5);
  spinner.start();
  // Do stuff periodically
  ros::Rate _rate(rate);
  ros::ServiceClient topicfinder = n.serviceClient<rosapi::Topics>(
      "rosapi/topics");
  while (ros::ok()) {
    // Plot map
//      boost::shared_ptr<cv::Mat> image(doColorMapCallback());
//      cv::imshow( "Current View", *image );                    // Show our image inside it.
//      cv::waitKey(1); // Update the window
    // Print topics
//      std::cerr << "TEST" << endl;
//      ros::master::V_TopicInfo topics;
//      if (ros::master::getTopics(topics)) {
//        ROS_INFO("List topics:");
//        ROS_INFO("master: topic size: %d", int(topics.size()));
//        for (int idx = 0; idx < topics.size(); ++idx) {
//          ROS_INFO("\n%d:\n"
//                   "  TOPIC: %s\n"
//                   "  TYPE : %s\n",
//                   idx, topics.at(idx).name.c_str(), topics.at(idx).datatype.c_str());
//        }
//      } else {
//        ROS_ERROR("No topics listable");
//      }
    // From http://answers.ros.org/question/108176/how-to-list-all-topicsservices-that-are-known-by-the-server-with-roscpp/?answer=109931#post-id-109931
    rosapi::Topics topicsrv;
    topicfinder.call(topicsrv);
    ROS_INFO("SRV: responsesize: %d", int(topicsrv.response.topics.size()));
    for (int idx = 0; idx < topicsrv.response.topics.size(); ++idx) {
      ROS_INFO("Topics: %s", topicsrv.response.topics.at(idx).c_str());
    }
    _rate.sleep();
  }

  delete listenerTf;

  return 0;
}
