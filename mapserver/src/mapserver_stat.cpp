// MRPT
#include <mrpt/maps/CMultiMetricMap.h>
//#include <mrpt/maps.h>
//#include <mrpt/opengl.h>
//#include <mrpt/gui.h>
#if defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS)
  #define OCCUPANCY_GRIDMAP_CELL_SIZE 1
#else // defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
  #define OCCUPANCY_GRIDMAP_CELL_SIZE 2
#endif

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
std::mutex mtxSwap, mtxShowRpc, mtxShowIsm;


// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <mapserver_msgs/pnsTuple.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <rosgraph_msgs/Log.h>
#include <tf/transform_listener.h>
#include <mapserver/rsm.h>
#include <mapserver/ismStackFloat.h>
#include <tf_conversions/tf_eigen.h>
#include <rosapi/Topics.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <ros/duration.h>

ros::Publisher publisherMap, publisherIsmAsPointCloud, publisherIsmAsOgm;
ros::Subscriber subscriberIsm, subscriberTfTileName, subscriberStoreMaps, subscriberTuple;
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
#include <memory>

// Boost
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <Constants.h>
#include <utils.h>
using namespace ms::constants;
using namespace ms::constants::mappingLayers;
namespace msNumeric = ms::constants::numeric;

// Concatenated map scopes with the correct prefix, which is set by program options
static std::string mapScopes[mappingLayers::NUM_MAPS];

// Program options
  // Properties of a single occupancy grid map
  static std::string currentTileTfName(""), lastTileTfName("");
  static std::string topicMap, topicLaser, tileOriginTfPrefix, tileOriginTfSufixForRoiOrigin, currentTfNameTopic, currentTupleTopic, worldLink, storeMapsTopic, reqTopicMapStack, debugIsmTopic;
  static double idleStartupTime_s;
  static double resolution = mapping::discreteResolution;
  static float maxOccupancyUpdateCertainty = mapping::ogm::maxOccupancyUpdateCertainty;
  static float maxDistanceInsertion = std::min(mapping::roi::width, mapping::roi::height);
  static float max_x = mapping::roi::xMax;
  static float min_x = mapping::roi::xMin;
  static float max_y = mapping::roi::yMax;
  static float min_y = mapping::roi::yMin;
  static int debug = 0;
  static int doTest = 0;
  static float rate = 1;
  static std::string ismScopePrefix = scopes::map::super::ogm;
  // Marging for non present information: 0.45 .. 0.55 -> 0.1
  static float uncertaintyBoundary = mapping::ogm::minDrawOccupancyUpdateCertainty;
  static float mapInitValue = mapping::ogm::unknownOccupancyUpdateCertainty;

// Global variables
  // The map stack
  static std::vector<mrpt::maps::COccupancyGridMap2D> mapStack;  // Current stack for fusion
  static std::vector<mrpt::maps::COccupancyGridMap2D> mapStackStorageTemp;  // Temp stack for hdd storage
  static std::vector<mrpt::maps::COccupancyGridMap2D> mapStackShiftTemp; // Temp stack for shifting
  static mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
  // Discrete resolution of the map
  static int mapSizeX;
  static int mapSizeY;
  static cv::Mat storageMapBuffer;
  static std::string mapStorageLocation = mapping::ogm::mapStorageLocation;
  static int dontStoreMaps = 0, shiftMap = 1; // Shift (true) or clean (false) the map if center_dresch is received

// For sending the localization
// rsb::Informer<rst::geometry::PoseEuler>::Ptr informerOdometry;

// Actual position
// rsb::Informer<rst::geometry::PoseEuler>::DataPtr odomData(new rst::geometry::PoseEuler);

using namespace boost;
using namespace std;

std::mutex mapRefresh;
std::mutex mtxOdom;       // mutex for odometry messages

// Set all map tiles of all maps to the given value
void fillMapStack(std::vector<mrpt::maps::COccupancyGridMap2D> &mapStack, float value = mapping::ogm::unknownOccupancyUpdateCertainty) {
  for (uint idx = 0; idx < mapStack.size(); ++idx) {
    mapStack[idx].fill (value);  // Fill every map with uncertainty
  }
}

// Translate a map
void translateMap(mrpt::maps::COccupancyGridMap2D &map, int offsetx = 0, int offsety = 0, float fillProbability = mapping::ogm::unknownOccupancyUpdateCertainty) {

  // Define a transformation for the image
  cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, static_cast<double>(offsetx), 0, 1, static_cast<double>(offsety));

  // Get the raw data as an image
  cv::Mat mapAsImage(static_cast<int>(map.getSizeY()),
                     static_cast<int>(map.getSizeX()),
#if defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS)
  CV_8SC1
#else // defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
  CV_16SC1
#endif
                     , (void*)&map.getRawMap()[0]);

  // Warp the image (which refers to the map)
  cv::warpAffine(mapAsImage,mapAsImage,trans_mat,
                 cv::Size(static_cast<int>(map.getSizeX()), static_cast<int>(map.getSizeY())),
                 cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT,
                 cv::Scalar(static_cast<double>(mrpt::maps::COccupancyGridMap2D::p2l(fillProbability))));
}

// Translate a stack of maps
void translateMapStack(const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStack, int offsetx = 0, int offsety = 0, float fillProbability = mapping::ogm::unknownOccupancyUpdateCertainty) {
  for (std::map<std::string, mrpt::maps::COccupancyGridMap2D*>::iterator it=mapStack->begin(); it!=mapStack->end(); ++it) {
    translateMap(*it->second, offsetx, offsety , fillProbability);
  }
}

// Add just some picture of each other
boost::shared_ptr<cv::Mat> doColorMapCallback() {

  // TODO Define this as a global variable so that it dows not
  // have to allocate space on every call
  boost::shared_ptr<cv::Mat> dst(new cv::Mat(mapSizeX, mapSizeY, CV_8UC3));
  dst->setTo(cv::Scalar(127,127,127)); // to set all values to 127 (aka unknown)

  const float validCellValue = uncertaintyBoundary;
  for (uint mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    for (int idy = 0; idy < mapSizeY; ++idy) {
      for (int idx = 0; idx < mapSizeX; ++idx) {
        if (mapStack[mapIdx].getCell(idx, idy) > validCellValue) {
          dst->at<cv::Vec3b>(idy,idx)[0] = mappingLayers::mapColorBGR[mapIdx][0];
          dst->at<cv::Vec3b>(idy,idx)[1] = mappingLayers::mapColorBGR[mapIdx][1];
          dst->at<cv::Vec3b>(idy,idx)[2] = mappingLayers::mapColorBGR[mapIdx][2];
        }
      }
    }
  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
};


std::shared_ptr<cv::Mat> mrptOggToGrayScale(mrpt::maps::COccupancyGridMap2D &map) {

  std::shared_ptr<cv::Mat> dst(new cv::Mat(map.getSizeY(), map.getSizeX(), CV_8UC1));
//  dst->setTo(cv::Scalar(127,127,127)); // to set all values to 127 (aka unknown)


  for (int idx = 0; idx < map.getRawMap().size(); ++idx) {
    dst->at<uchar>(idx) = mrpt::maps::COccupancyGridMap2D::l2p_255(map.getRawMap().at(idx));
  }
//  for (int idy = 0; idy < map.getSizeY(); ++idy) {
//    for (int idx = 0; idx < map.getSizeX(); ++idx) {
//      const uchar intensity = uchar(map.getCell(idx, idy) * 255);
//      dst->at<uchar>(idx,idy) = intensity;
//    }
//  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
};

std::shared_ptr<cv::Mat> rosOggToGrayScale(nav_msgs::OccupancyGrid::ConstPtr map) {

  std::shared_ptr<cv::Mat> dst;

  if (map) {
    if (map->info.width > 0 && map->info.height > 0) {
      dst = std::shared_ptr<cv::Mat>(new cv::Mat(map->info.height, map->info.width, CV_8UC1));
//      dst->setTo(cv::Scalar(127,127,127)); // to set all values to 127 (aka unknown)

      for (int idx = 0; idx < map->data.size(); ++idx) {
        const int oggValue = int(map->data.at(idx));
        const int intensity = oggValue < 0 ? 50 : oggValue;
        dst->at<uchar>(idx) = uchar(float(intensity) * 2.55);
      }
    }
  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
};

// Translate a map
template <typename T>
void translateMap(cv::Mat &src, cv::Mat &dst, double offsetx = 0, double offsety = 0, T fillValue = msNumeric::invalidValue_int16) {

  // Define a transformation for the image
  const cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);

  // Warp the image (which refers to the map)
  cv::warpAffine(src, dst, trans_mat,
                 cv::Size(src.rows, src.cols),
                 cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT,
                 cv::Scalar(static_cast<double>(fillValue)));

}



template <typename T>
void mapRefreshAndStorage(const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStack,
                          const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStackShiftedResult,
                          const tf::StampedTransform transformRoiInWorld,
                          const std::string prefixString,
                          const std::string formatString,
                          const std::string formatUnitString,
                          const double resolution_meterPerTile,
                          const bool storeMapStack,
                          const bool shiftMapStack,
                          const bool clearMapStack,
                          T fillValue = 0.5f,
                          bool storeCurrentPosition = true,
                          std::string additionalInformationString = std::string("")) {

  // The message from the last time the function was called (So it is the location of the center)
  static tf::StampedTransform transformRoiInWorldLast;

//  static bool runOnce = true;
//  if (runOnce) {
//      ROS_WARN("First run, map storage coordniates might be corrupted");
//      transformRoiInWorldLast = transformRoiInWorld;
//      runOnce = false;
//  }

  ROS_INFO("storeMapStack: %d", storeMapStack);
  ROS_INFO("mapStack->size(): %d", mapStack->size());
  if (storeMapStack) {
    // Get the timestamp in microseconds
//    ros::Time stamp = ros::Time::now();
//    uint64_t timestamp = uint64_t(stamp.sec) * uint64_t(1e6) + uint64_t(stamp.nsec / 1e6);
    // Store each layer in the map
    for (auto it=mapStack->begin(); it!=mapStack->end(); ++it) {
      std::cout << it->first << " => " << it->second << '\n';

      // Get the format string
      const std::string format = formatString.empty() ? std::string("INT") + std::to_string(int(OCCUPANCY_GRIDMAP_CELL_SIZE) * int(8)) : formatString;
      // Replace the topic name "/my/topic" to "-my-topic"
      std::string mapIdx = it->first;
      std::replace( mapIdx.begin(), mapIdx.end(), '/', '-');

      // Get the filename
      std::ostringstream oss;
      oss << std::setprecision(2)
          << mapStorageLocation
          << "What_" << prefixString << "_"
          << "T_" << ros::Time::now() << "s.ns_"
          << "Format_" << format << "_"
          << "Unit_" << formatUnitString << "_"
          << "Layer_" << mapIdx << "_"
          << "Res_" << int(round(resolution_meterPerTile * double(geometry::millimeterPerMeter))) <<  "mm_"
          << "X_" << transformRoiInWorldLast.getOrigin().x() << "m_"
          << "Y_" << transformRoiInWorldLast.getOrigin().y() << "m_"
          << "Z_" << transformRoiInWorldLast.getOrigin().z() << "m_"
          << "rows_" << it->second->getSizeY() << "_"
          << "cols_" << it->second->getSizeX() << "_"
          << additionalInformationString << "_"
          << ".bin";

      ROS_INFO("Store map to: %s\nWith format: %s\n", oss.str().c_str(), format.c_str());

      // Store the layer
      std::ofstream f;
      f.open(oss.str(), std::ofstream::out | std::ofstream::binary);
      if (f.is_open()) {
         f.write((char*)it->second->getRawMap().data(),
                 it->second->getSizeX() * it->second->getSizeY() * OCCUPANCY_GRIDMAP_CELL_SIZE);
         f.close();
      } else {
          ROS_ERROR("Unable to open file %s\n", oss.str().c_str());
      }
    }
  }

  // Shift the map
  if (shiftMapStack) {
    // Calculate the shift as indices
    const double xdiff_m = transformRoiInWorldLast.getOrigin().x() - transformRoiInWorld.getOrigin().x();
    const double ydiff_m = transformRoiInWorldLast.getOrigin().y() - transformRoiInWorld.getOrigin().y();
    const double zdiff_m = transformRoiInWorldLast.getOrigin().y() - transformRoiInWorld.getOrigin().y();

    ROS_INFO("Current (x,y,z) in m: %f, %f, %f", transformRoiInWorld.getOrigin().x(), transformRoiInWorld.getOrigin().y(), transformRoiInWorld.getOrigin().z());
    ROS_INFO("Last    (x,y,z) in m: %f, %f, %f", transformRoiInWorldLast.getOrigin().x(), transformRoiInWorldLast.getOrigin().y(), transformRoiInWorldLast.getOrigin().z());
    ROS_INFO("Diff    (x,y,z) in m: %f, %f, %f", xdiff_m, ydiff_m, zdiff_m);

    const int xshift_tiles = int(std::round(xdiff_m / resolution_meterPerTile));
    const int yshift_tiles = int(std::round(ydiff_m / resolution_meterPerTile));

//    ROS_INFO_STREAM( "\nydiff_m: " << ydiff_m <<
//                     "\nresolution_meterPerTile: " << resolution_meterPerTile <<
//                     "\nydiff_m / resolution_meterPerTile: " << ydiff_m / resolution_meterPerTile <<
//                     "\n-ydiff_m / resolution_meterPerTile: " << ydiff_m / resolution_meterPerTile <<
//                     "\n-ydiff_m / resolution_meterPerTile: " << ydiff_m / resolution_meterPerTile);

    ROS_INFO("Shift of the map (res: %0.2f m/tile): x=%d tiles s.t. %f m , y=%d tiles s.t. %f m", resolution_meterPerTile, xshift_tiles, -xdiff_m, yshift_tiles, -ydiff_m);

    auto itDst = mapStackShiftedResult->begin();
    for (auto itSrc=mapStack->begin(); itSrc!=mapStack->end(); ++itSrc, ++itDst) {
#if defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS)
      const int opencvType = CV_8SC1;
#else // defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
      const int opencvType = CV_16SC1;
#endif
      cv::Mat src(itSrc->second->getSizeY(), itSrc->second->getSizeX(), opencvType, (void*)(itSrc->second->getRawMap().data()));
      cv::Mat dst(itDst->second->getSizeY(), itDst->second->getSizeX(), opencvType, (void*)(itDst->second->getRawMap().data()));
      translateMap(src, dst, xshift_tiles, yshift_tiles, fillValue);
    }
  }

  // Clear the map
  if (clearMapStack) {
    ROS_ERROR("Clear the map");
    for (std::map<std::string, mrpt::maps::COccupancyGridMap2D*>::iterator it=mapStack->begin(); it!=mapStack->end(); ++it) {
      it->second->fill(fillValue);
    }
  }

  if (storeCurrentPosition) {
      transformRoiInWorldLast = transformRoiInWorld; // Store the new location for the next time
  }

}


void mapStorage(const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStack,
                          const std::string prefixString,
                          const std::string formatString,
                          const std::string formatUnitString,
                          const double resolution_meterPerTile) {

  const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> dummyMap;
  const tf::StampedTransform dummyTf;
  const float dummyFloat = 0.0f;

  mapRefreshAndStorage( mapStack,                                 // Map to shift/store/reset
                        dummyMap,                                 // Nothing at all
                        dummyTf,                                  // Transform
                        std::string("OGM"),                       // Kind of map
                        std::string(""),                          // Format (empty: Take from type specifier)
                        std::string("logodds"),                   // Unit
                        def.resolution,                           // Resolution per tile
                        true,                                     // Maps should be stored
                        false,                                    // Maps should be shifted
                        false,                                    // Map should be reseted reset
                        dummyFloat,                               // Some float value
                        false);                                   // Don't store the current position

}

///
/// \brief Returns the pose in the desired frame
/// \param poseInSourceFrame Stamped pose in the source frame
/// \param targetFrame The target frame in which the returned pose will reside
/// \param tfListener A transformer
/// \return Transformed pose. Shared pointer is empty if an error occurs
///
std::shared_ptr<tf::Stamped<tf::Pose>> getPoseInFrame(const tf::Stamped<tf::Pose> &poseInSourceFrame, const std::string &targetFrame, const tf::TransformListener &tfListener) {

  std::shared_ptr<tf::Stamped<tf::Pose>> poseInTargetFrameReturn;
  tf::Stamped<tf::Pose> poseInTargetFrame;
  // Use source frame as target frame if empty
  const bool useSrcFrameAsDstFrame = targetFrame.empty() || !targetFrame.compare(poseInSourceFrame.frame_id_) ? true : false;
  const std::string srcFrame = poseInSourceFrame.frame_id_;
  const std::string dstFrame = useSrcFrameAsDstFrame ? srcFrame : targetFrame;
  // Get the origin of the pose in the target frame
  bool tfSuccess = true;
  if (!useSrcFrameAsDstFrame) { // We don't need this, if we stay in the same frame
    try {
      // HACK We don't wait for tf messages in the past, because they will never arrive at all
      std::string errorMsg;
      const std::string errorMsgWorthitToWaitFor("Lookup would require extrapolation into the future");
      tfListener.canTransform(dstFrame, srcFrame, poseInSourceFrame.stamp_, &errorMsg);
      std::size_t found = errorMsg.find(errorMsgWorthitToWaitFor);
      if (found != std::string::npos || errorMsg.empty()) {
          if (found != std::string::npos) {
              ROS_DEBUG_STREAM("getPoseInFrame: We'll wait for tf transform messages in the future: " << errorMsg);
              tfListener.waitForTransform(dstFrame, srcFrame, poseInSourceFrame.stamp_, ros::Duration(3.0));
          }
          tfListener.transformPose(dstFrame, poseInSourceFrame, poseInTargetFrame);
      } else {
          throw std::runtime_error(errorMsg);
      }
    } catch(const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("getPoseInFrame (%s -> %s): %s", srcFrame.c_str(), dstFrame.c_str(), excStr.c_str());
      tfSuccess = false;
    }
  } else {
      poseInTargetFrame = poseInSourceFrame;
  }

  // Return a filled shared pointer if tf was successful
  if (tfSuccess) {
      poseInTargetFrameReturn = std::make_shared<tf::Stamped<tf::Pose>>(poseInTargetFrame);
  }
  return poseInTargetFrameReturn;
}

///
/// \brief Calculate the metric coordinates of OGM cell centroids in the target frame
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetFrame The target frame in which the the point is calculated (target frame equals OGM frame if empty)
/// \param tfListener A transformer
/// \param idxStart Start index in width
/// \param idyStart Start index in height
/// \param idxWidth Amount of cells in x direction
/// \param idyHeight Amount of cells in y direction
/// \return Vector of points in row major order. Shared pointer is empty if an error occurs
///
std::shared_ptr<std::vector<tf::Point>> ogmCellCoordinatesToPoints(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                                   const std::string &targetFrame,
                                                                   const tf::TransformListener &tfListener,
                                                                   const std::size_t &idxStart,
                                                                   const std::size_t &idyStart,
                                                                   const std::size_t &idxWidth,
                                                                   const std::size_t &idyHeight) {

  std::shared_ptr<std::vector<tf::Point>> points;
  // Sanity checks
  if(!ogm) {
      ROS_ERROR("OGM is empty");
  } else if((idxStart + idxWidth) > ogm->info.width || (idyStart + idyHeight) > ogm->info.height || idxWidth == 0 || idyHeight == 0) {
      ROS_ERROR("Requested range out of bound");
  } else {
      // Get the pose in the target frame
      tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
      std::shared_ptr<tf::Stamped<tf::Pose>> ogmOriginInTargetFrame = getPoseInFrame(tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp, ogm->header.frame_id), targetFrame, tfListener);
      // Start calculating the transform
      if (ogmOriginInTargetFrame) {
          points = std::shared_ptr<std::vector<tf::Point>>(new std::vector<tf::Point>(idxWidth * idyHeight));
          const float cornerToCenterOffset = ogm->info.resolution / 2.0f;

          auto pointsIter = points->begin();
          // tf::Stamped<tf::Point> ptInOgmFrame(tf::Point(0,0,0), ogm->header.stamp, ogm->header.frame_id); // The metric point of a OGM cell in the OGM frame
          for (int idy = idyStart; idy < idyStart + idyHeight ; ++idy) {
              for (int idx = idxStart; idx < idxStart + idxWidth; ++idx, ++pointsIter) {
                  const tf::Point ptInOgmOrigin(float(idx) * ogm->info.resolution + cornerToCenterOffset,
                                                float(idy) * ogm->info.resolution + cornerToCenterOffset,
                                                0.0f);
                  // ptInOgmOrigin.setZ(0.0f); // Don't need this
                  // ptInOgmFrame = ogmOriginInSourceFrame * ptInOgmOrigin; // We don't need this
                  const tf::Point ptInTargetFrame = *ogmOriginInTargetFrame * ptInOgmOrigin;
                  *pointsIter = ptInTargetFrame;
              }
          }
      } else {
          ROS_WARN("TF unsuccessful");
      }
  }
  return points;
}

///
/// \brief Converts a OGM to a point cloud Calculate the metric coordinates of OGM cell centroids in the target frame
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetFrame The target frame in which the point is calculated (target frame equals OGM frame if empty)
/// \param tfListener A transformer
/// \param idxStart Start index in width
/// \param idyStart Start index in height
/// \param idxWidth Amount of cells in x direction
/// \param idyHeight Amount of cells in y direction
/// \return OGM as point cloud. The OGM range [-1, 0 .. 100] is converted to [-0.01f, 0 .. 1.0f]. If message pointer is empty, an error occurred
///
sensor_msgs::PointCloud::Ptr ogmCellsToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                                   const std::string &targetFrame,
                                                                   const tf::TransformListener &tfListener,
                                                                   const std::size_t &idxStart,
                                                                   const std::size_t &idyStart,
                                                                   const std::size_t &idxWidth,
                                                                   const std::size_t &idyHeight) {
  sensor_msgs::PointCloud::Ptr ogmAsPointCloud;
  // First get the metric coordinates
  std::shared_ptr<std::vector<tf::Point>> points = ogmCellCoordinatesToPoints(ogm, targetFrame, tfListener, idxStart, idyStart, idxWidth , idyHeight);
  // Sanity checks: We rely on ogmCellCoordinatesToPoints
  if (points) {
      ogmAsPointCloud = boost::shared_ptr<sensor_msgs::PointCloud>(new sensor_msgs::PointCloud);
      ogmAsPointCloud->header = ogm->header;
      ogmAsPointCloud->header.frame_id = targetFrame.empty() ? ogmAsPointCloud->header.frame_id : targetFrame;
      ogmAsPointCloud->points.resize(points->size());
      ogmAsPointCloud->channels.resize(1);
      ogmAsPointCloud->channels.at(0).name = "OGM";
      ogmAsPointCloud->channels.at(0).values.resize(points->size());
      auto pointCloudValueIter = ogmAsPointCloud->channels.at(0).values.begin();
      auto pointCloudPointIter = ogmAsPointCloud->points.begin();
      auto pointIter = points->begin();
      for (int idy = idyStart; idy < idyStart + idyHeight ; ++idy) {
          for (int idx = idxStart; idx < idxStart + idxWidth; ++idx, ++pointCloudValueIter, ++pointCloudPointIter, ++pointIter) {
              const int index = idx + (idy * ogm->info.width);
              *pointCloudValueIter = float(ogm->data.at(index)) / 100.0f;
              pointCloudPointIter->x = float(pointIter->x());
              pointCloudPointIter->y = float(pointIter->y());
              pointCloudPointIter->z = float(pointIter->z());
          }
      }
  } else {
      ROS_WARN("ogmCellCoordinatesToPoints is empty");
  }
  return ogmAsPointCloud;

}

void correctInvalidOrientation(tf::Pose &pose) {
  if (pose.getRotation().x() < tf::QUATERNION_TOLERANCE &&
      pose.getRotation().y() < tf::QUATERNION_TOLERANCE &&
      pose.getRotation().z() < tf::QUATERNION_TOLERANCE &&
      pose.getRotation().w() < tf::QUATERNION_TOLERANCE) {
      ROS_WARN_ONCE("correctInvalidOrientation: Pose with quaternion(0,0,0,0) detected. Interpretation as (0,0,0,1)");
      pose.setRotation(tf::Quaternion(0,0,0,1));
    }
}

///
/// \brief Returns the coordinates of the four corner points of the OGM in the given frame
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetFrame The target frame in which the point is calculated (target frame equals OGM frame if empty)
/// \param tfListener A transformer
/// \return Vector of points starting top-left. Shared pointer is empty if an error occurs
///
std::shared_ptr<std::vector<tf::Point>> getOgmCornerPoints(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                           const std::string &targetFrame,
                                                           const tf::TransformListener &tfListener) {

  std::shared_ptr<std::vector<tf::Point>> points;
  // Sanity checks
  if(!ogm) {
      ROS_ERROR("OGM is empty");
  } else {
      // Get the pose in the target frame
      tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
      correctInvalidOrientation(ogmOriginPose);
      std::shared_ptr<tf::Stamped<tf::Pose>> ogmOriginInTargetFrame = getPoseInFrame(tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp, ogm->header.frame_id), targetFrame, tfListener);
      // Start calculating the transform
      if (ogmOriginInTargetFrame) {
          const std::size_t numCorners = 4;
          points = std::shared_ptr<std::vector<tf::Point>>(new std::vector<tf::Point>(numCorners));
          // pointInTargetFrame = transformMatrixFromTargetFrameToSourceFrame * pointInSourceFrame;
          points->at(0) = *ogmOriginInTargetFrame * tf::Point(0.0f, 0.0f, 0.0f);
          points->at(1) = *ogmOriginInTargetFrame * tf::Point(ogm->info.width * ogm->info.resolution, 0.0f, 0.0f);
          points->at(2) = *ogmOriginInTargetFrame * tf::Point(ogm->info.width * ogm->info.resolution, ogm->info.height * ogm->info.resolution, 0.0f);
          points->at(3) = *ogmOriginInTargetFrame * tf::Point(0.0f, ogm->info.height * ogm->info.resolution, 0.0f);
      } else {
          ROS_WARN("TF unsuccessful");
      }
  }
  return points;
}

///
/// \brief Returns the coordinates of the four corner points of the OGM in the given pose
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetPose The target Pose in which the point is calculated
/// \param tfListener A transformer
/// \return Vector of points starting top-left. Shared pointer is empty if an error occurs
///
std::shared_ptr<std::vector<tf::Point>> getOgmCornerPoints(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                           const tf::Stamped<tf::Pose> &targetPose,
                                                           const tf::TransformListener &tfListener) {
  std::shared_ptr<std::vector<tf::Point>> points;
  // Get pose of the OGM in the target frame
  std::shared_ptr<tf::Stamped<tf::Pose>> targetPoseInOgmFrame = getPoseInFrame(targetPose, ogm->header.frame_id, tfListener);
  if (targetPoseInOgmFrame) {
      // First get the corner points in own frame
      points = getOgmCornerPoints(ogm, ogm->header.frame_id, tfListener);
      if (points) {
          points->at(0) = targetPoseInOgmFrame->inverse() * points->at(0);
          points->at(1) = targetPoseInOgmFrame->inverse() * points->at(1);
          points->at(2) = targetPoseInOgmFrame->inverse() * points->at(2);
          points->at(3) = targetPoseInOgmFrame->inverse() * points->at(3);
      }
  }
  return points;
}

cv::Point2f getFarthestPoints(std::vector<cv::Point2f> &points) {
  float abs = 0.0;
  std::size_t idx = 0, id=0;
  for (auto it = points.begin(); it != points.end(); ++it, ++idx) {
      const float absTmp = it->x * it->x + it->y * it->y;
      if ( abs < absTmp) {
          abs = absTmp;
          id = idx;
      }
  }
  return points.at(id);
}

cv::Point2f getNearestPoints(std::vector<cv::Point2f> &points) {
  float abs = FLT_MAX;
  std::size_t idx = 0, id=0;
  for (auto it = points.begin(); it != points.end(); ++it, ++idx) {
      const float absTmp = it->x * it->x + it->y * it->y;
      if ( abs > absTmp) {
          abs = absTmp;
          id = idx;
      }
  }
  return points.at(id);
}

cv::Point2f getXYMax(std::vector<cv::Point2f> &points) {
  float xMax = 0.0;
  float yMax = 0.0;
  for (auto it = points.begin(); it != points.end(); ++it) {
      if ( it->y > yMax) {
          yMax = it->y;
      }
      if ( it->x > xMax) {
          xMax = it->x;
      }
  }
  return cv::Point2f(xMax, yMax);
}

cv::Point2f getXYMin(std::vector<cv::Point2f> &points) {
  float xMax = FLT_MAX;
  float yMax = FLT_MAX;
  for (auto it = points.begin(); it != points.end(); ++it) {
      if ( it->y < yMax) {
          yMax = it->y;
      }
      if ( it->x < xMax) {
          xMax = it->x;
      }
  }
  return cv::Point2f(xMax, yMax);
}


///
/// \brief Compares to values with a given slack. The slack is the absolute value, the two numbers may differ
/// \param a First value
/// \param b Second value
/// \param slack The maximum allowed value, the two numbers may differ
/// \return True if values are equal
///
template<typename T>
bool compare(T a, T b, T slack = T(0.0)) {
  T diff;
  // Sanity check
  if (slack < T(0.0)) {
      slack = -slack;
  }
  if (a > b) {
      diff = a - b;
  } else {
      diff = b - a;
  }
  if (diff <= slack) {
      return true;
  } else {
      return false;
  }
}

///
/// \brief Resize a OGM to the desired resolution
/// \param ogm The OGM which should be resized
/// \param targetResolution The target resolution of the OGM
/// \return Resized OGM
///
nav_msgs::OccupancyGrid::ConstPtr ogmResize(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                   const float &targetResolution) {

  nav_msgs::OccupancyGrid::Ptr ogmTf = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);

  const float scaling = ogm->info.resolution / targetResolution;
  // Get data as OpenCV
  // HACK Type should be CV_8SC1, but resize function only works with CV_8UC1
  const cv::Mat ogmCv(ogm->info.height, ogm->info.width, CV_8UC1, (void*)(ogm->data.data()));
  cv::Mat ogmCvResized;
  // Resize
  cv::resize(ogmCv, ogmCvResized, cv::Size(), scaling, scaling, cv::INTER_LINEAR);
  // Copy back
  ogmTf->header = ogm->header;
  ogmTf->info = ogm->info;
  ogmTf->info.height = ogmCvResized.rows;
  ogmTf->info.width = ogmCvResized.cols;
  ogmTf->info.resolution = targetResolution;
  ogmTf->data.resize(ogmCvResized.rows * ogmCvResized.cols);
  memcpy((void*)ogmTf->data.data(), (void*)ogmCvResized.data,ogmCvResized.rows * ogmCvResized.cols);

  return ogmTf;
}

///
/// \brief Warps a OGM to the desired pose and resolution such that the new OGM is aligned with this pose
/// \param ogm The OGM which should be transformed
/// \param targetPose The target pose in which the new OGM resides
/// \param targetResolution The target resolution of the OGM
/// \param tfListener A transformer
/// \param resetPoseToOgmBoundary The origin of the new OGM will not reside in targetOrigin, but with respect to the minimum possible size of the which pose lies in the XY-plane of targetOrigin.
/// \return OGM in the new frame. If message pointer is empty, an error occurred
///
nav_msgs::OccupancyGrid::ConstPtr ogmTf(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                   const tf::Stamped<tf::Pose> &targetOrigin,
                                   const float &targetResolution,
                                   const tf::TransformListener &tfListener,
                                   const bool resetPoseToOgmBoundary = false) {

  nav_msgs::OccupancyGrid::Ptr ogmTf;

  // TODO Check for XY shift, which can be easily calculated (Just shift the pose in xy and maybe check for resolution)



  // Sanity check for invalid orientation
  tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
  correctInvalidOrientation(ogmOriginPose);

  // First check, if pose and resolution are the same, to minimize workload
  if (ogmOriginPose == targetOrigin) {
      const bool resolutionIsSame = compare(targetResolution, ogm->info.resolution, min(ogm->info.resolution, targetResolution) / 2 );
      if (!resolutionIsSame) { // Scale the OGM
          ROS_DEBUG("Pose is the same: Just change the resolution of the OGM");
          return ogmResize(ogm, targetResolution);
      } else {
          ROS_DEBUG("Pose and resolution are the same: Just return the original OGM");
          return ogm;
      }
  } else { // Do a full transform of the OGM
    // First get the metric coordinates of the four corner points in the target and source frame
    std::shared_ptr<std::vector<tf::Point>> pointsInTargetFrame = getOgmCornerPoints(ogm, targetOrigin, tfListener);
//    tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
    std::shared_ptr<std::vector<tf::Point>> pointsInSourceFrame = getOgmCornerPoints(ogm, tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp, ogm->header.frame_id), tfListener);
    // Calculate the homography
    if (pointsInTargetFrame && pointsInSourceFrame) {
        // Load OGM as OpenCV image
        // HACK Type should be CV_8SC1, but resize function only works with CV_8UC1
        cv::Mat ogmCv(ogm->info.height, ogm->info.width, CV_8UC1, (void*)(ogm->data.data()));

        std::vector<cv::Point2f> pointsSource(4), pointsTarget(4);
        pointsSource.at(0) = cv::Point2f(float(0),float(0));
        pointsSource.at(1) = cv::Point2f(float(ogm->info.width),float(0));
        pointsSource.at(2) = cv::Point2f(float(ogm->info.width),float(ogm->info.height));
        pointsSource.at(3) = cv::Point2f(float(0),float(ogm->info.height));

        pointsTarget.at(0) = cv::Point2f(float(pointsInTargetFrame->at(0).x() / targetResolution),float(pointsInTargetFrame->at(0).y() / targetResolution));
        pointsTarget.at(1) = cv::Point2f(float(pointsInTargetFrame->at(1).x() / targetResolution),float(pointsInTargetFrame->at(1).y() / targetResolution));
        pointsTarget.at(2) = cv::Point2f(float(pointsInTargetFrame->at(2).x() / targetResolution),float(pointsInTargetFrame->at(2).y() / targetResolution));
        pointsTarget.at(3) = cv::Point2f(float(pointsInTargetFrame->at(3).x() / targetResolution),float(pointsInTargetFrame->at(3).y() / targetResolution));

        // If the pose will be reset to the boundary
        tf::Stamped<tf::Pose> targetOriginNew = targetOrigin;
        if (resetPoseToOgmBoundary) {
            const cv::Point2f xyMinPoint = getXYMin(pointsTarget);
            tf::Pose resetPose(tf::Quaternion(0,0,0,1), tf::Vector3(tfScalar(xyMinPoint.x * targetResolution), tfScalar(xyMinPoint.y * targetResolution), tfScalar(0)));
            pointsTarget.at(0) -= xyMinPoint;
            pointsTarget.at(1) -= xyMinPoint;
            pointsTarget.at(2) -= xyMinPoint;
            pointsTarget.at(3) -= xyMinPoint;
            targetOriginNew *= resetPose;
        }

        // Do the warping to get the PGM in the new pose
        cv::Mat H = cv::findHomography(  pointsSource,pointsTarget,0 );
        const cv::Point2f xyMaxPoint = getXYMax(pointsTarget);
        cv::Mat ogmCvWarped(xyMaxPoint.y, xyMaxPoint.x, CV_8SC1);
        // Warp the OGM with linear interpolation (Boarders are set to unknown s.t. 50)
        cv::warpPerspective(ogmCv,ogmCvWarped,H,ogmCvWarped.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(50));

        // Copy back
        ogmTf = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);
        ogmTf->header = ogm->header;
        ogmTf->header.frame_id = targetOriginNew.frame_id_;
        geometry_msgs::PoseStamped poseMsg;
        tf::poseStampedTFToMsg(targetOriginNew, poseMsg);
        ogmTf->info.origin = poseMsg.pose;
        ogmTf->info.height = ogmCvWarped.rows;
        ogmTf->info.width = ogmCvWarped.cols;
        ogmTf->info.resolution = targetResolution;
        ogmTf->data.resize(ogmCvWarped.rows * ogmCvWarped.cols);
        memcpy((void*)ogmTf->data.data(), (void*)ogmCvWarped.data,ogmCvWarped.rows * ogmCvWarped.cols);
    } else {
        ROS_WARN("pointsInTargetFrame or pointsInSourceFrame are empty");
    }
  }
  return ogmTf;

}

void calcIsm4Mapserver(const double xIsmCenter_m, const double yIsmCenter_m, const double phiIsm_rad,
                const double ismResolution, cv::Mat &ismInRoi, cv::Mat &ism) {

  const double roiResolution = resolution;
  double ismMagnificationFactor = ismResolution / roiResolution;
  const int32_t x_dis_r = xIsmCenter_m / roiResolution;
  const int32_t y_dis_r = yIsmCenter_m / roiResolution;

  // Handle axis flipping
//  utils::rot90(ism,ismFlipcode);

  // Allocate space
  const cv::Size roiSizePx = ismInRoi.size();
  const cv::Point2f centerRoi(roiSizePx.height / 2.0f, roiSizePx.width /2.0f);
  ismInRoi = cv::Mat(roiSizePx, ismInRoi.type(), cv::Scalar(50/*% Unknown*/));  // Flush input image

  // Put the ISM in a bigger rectangular image, to handle rotation without cropping
  int newIsmRowsCols = sqrt(pow(ism.rows,2) + pow(ism.cols,2));
  const cv::Point2f ismOldCenter(ism.size().height / 2.0f, ism.size().width /2.0f);
  cv::Mat ismContainer(newIsmRowsCols,newIsmRowsCols, ism.type(), cv::Scalar(50 /*% Unknown*/));
  cv::Rect ismInNewIsm((newIsmRowsCols - ism.cols) / 2, (newIsmRowsCols - ism.rows) / 2, ism.cols, ism.rows);
  ism.copyTo(ismContainer(ismInNewIsm));

  // Resize ISM
  cv::Size newIsmSize(ismContainer.size().width * ismMagnificationFactor,
                      ismContainer.size().height * ismMagnificationFactor);
  cv::Mat ismResized;
  cv::resize(ismContainer,ismResized, newIsmSize, 0, 0, cv::INTER_NEAREST);

  // Rotate the ISM around its center
  const cv::Point2f centerISM(ismResized.size().height / 2.0f, ismResized.size().width /2.0f);
  cv::Mat rotation( 2, 3, CV_32FC1 );
  rotation = cv::getRotationMatrix2D( centerISM, - phiIsm_rad * rad2deg , 1.0f/*ZoomFactor*/ ); // Negative angle, because OpenCV is CW regarding our convention
  cv::Mat ismRotated;
  cv::warpAffine( ismResized, ismRotated, rotation, ismResized.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(50 /*% Unknown*/));

  // Copy the rotated ISM to its translated position in the ROI
  const cv::Point2f centerRotatedISM(ismRotated.size().height / 2.0f, ismRotated.size().width /2.0f);
  cv::Rect ismInRoiRect = cv::Rect(centerRoi.x + x_dis_r - centerRotatedISM.x, centerRoi.y  + y_dis_r - centerRotatedISM.y, ismRotated.cols, ismRotated.rows);
  ismRotated.copyTo(ismInRoi(ismInRoiRect));

//  // Store the first ISM to be converted
//  if (storeFirstIsm) {
//    static bool once = false;
//    if (!once) {
//      cv::imwrite(std::string("/tmp/") + storeFirstIsmName + std::string(".bmp"),ismInRoi); // RSB ISM
//      cv::imwrite(std::string("/tmp/") + storeFirstIsmName + std::string("Ism.bmp"),ism); // Native ROS ISM
//      once = true;
//    }
//  }
}

// Todo Do we some mutex around here ?
nav_msgs::OccupancyGrid::Ptr oggTf(const std::string &targetFrame, const nav_msgs::OccupancyGrid::ConstPtr ismRos, const double targetRes = -1/*meter/tile*/)
{

  nav_msgs::OccupancyGrid::Ptr oggTrans;

  // Get the transform between the frames
  tf::StampedTransform tfOcc;
  try {
    listenerTf->waitForTransform(ismRos->header.frame_id, targetFrame, ros::Time(0.0), ros::Duration(3.0));
    listenerTf->lookupTransform(ismRos->header.frame_id, targetFrame, ros::Time(0.0), tfOcc);
  } catch(const std::exception &exc) {
    const std::string excStr(exc.what());
    ROS_ERROR("tfTileNameHandler: %s", excStr.c_str());
    return oggTrans;
  }

  // Get the arguments to pack the current ISM into the ROI frame
  // We need to respect Z and also non aligned occupancy maps
//  int32_t x_dis_r_copy = x_dis_r;
//  int32_t y_dis_r_copy = y_dis_r;
//  int32_t z_dis_r_copy = z_dis_r;
  double x = tfOcc.getOrigin().getX();
  double y = tfOcc.getOrigin().getY();
//  double z = tfOcc.getOrigin().getZ();
  double phi_rad = utils::conversion::quaternion2eulerYaw(tfOcc.getRotation().getW(), tfOcc.getRotation().getX(), tfOcc.getRotation().getY(), tfOcc.getRotation().getZ());

  // Allocate the new Occ TODO: This is to sloppy, need propper definition
  oggTrans = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);
  oggTrans->header = ismRos->header;
  oggTrans->header.frame_id = targetFrame;
  oggTrans->info.resolution = targetRes < 0.0 ? ismRos->info.resolution : targetRes;
//  oggTrans->info.origin = ???
  const int width = (max_x - min_x) / oggTrans->info.resolution;
  const int height = (max_y - min_y) / oggTrans->info.resolution;
  oggTrans->info.width = width;
  oggTrans->info.height = height;

  // Allocate opencv images to do the work
  cv::Mat ismInRoi(height,width,CV_8SC1);
  cv::Mat ism(ismRos->info.height, ismRos->info.width, CV_8SC1, (void*)(&ismRos->data[0]));


  ROS_DEBUG("TF.xyz to %s: %f %f %f", ismRos->header.frame_id.c_str(), tfOcc.getOrigin().x(), tfOcc.getOrigin().y(), tfOcc.getOrigin().z());
  ROS_DEBUG("ismRos->info.origin.position.x,y: %f %f", ismRos->info.origin.position.x, ismRos->info.origin.position.y);

  // Calculate the tranlation from the base_footprint to the ISM origin
  // assuming that the coordinate systems are aligned after REP-103, and
  // that the height does not care
  const double xBasefootprint2Ism_m = tfOcc.getOrigin().x() + ismRos->info.origin.position.x;
  const double yBasefootprint2Ism_m = tfOcc.getOrigin().y() + ismRos->info.origin.position.y;
  const double ismWidthX_m = ismRos->info.width * ismRos->info.resolution;
  const double ismHeightY_m = ismRos->info.height * ismRos->info.resolution;
  double fz_rad = utils::conversion::quaternion2eulerYaw(ismRos->info.origin.orientation.w, ismRos->info.origin.orientation.x,
                                                  ismRos->info.origin.orientation.y, ismRos->info.origin.orientation.z);
  ROS_DEBUG("Euler orientation z (deg) %f",fz_rad*rad2deg);

  // Get the Center of the ISM after the rotation, assuming that we only operate in the plane
  double ismCenterX_m = ismWidthX_m / 2.0;
  double ismCenterY_m = ismHeightY_m / 2.0;
  utils::conversion::rotXY(ismCenterX_m, ismCenterY_m, fz_rad);
  ROS_DEBUG("ismCenterXY_m %f %f", ismCenterX_m, ismCenterY_m);

  // Rotate the center of the ISM in the footprint
  double xBasefootprint2IsmCenter_m = xBasefootprint2Ism_m + ismCenterX_m;
  double yBasefootprint2IsmCenter_m = yBasefootprint2Ism_m + ismCenterY_m;
  utils::conversion::rotXY(xBasefootprint2IsmCenter_m, yBasefootprint2IsmCenter_m, phi_rad);

  // Add the position inside the ROI
  const double xRoi2IsmCenter = xBasefootprint2IsmCenter_m + x;
  const double yRoi2IsmCenter = yBasefootprint2IsmCenter_m + y;
  // Get the final orientation of the ISM
  double ismOrientation = fz_rad + phi_rad; // TODO Check if fz needs to be assigned with minus

  // Merge ROS-ISM into RSB-ISM
  calcIsm4Mapserver(xRoi2IsmCenter, yRoi2IsmCenter, ismOrientation,
                     ismRos->info.resolution, ismInRoi, ism);

  const size_t size = width * height;
  oggTrans->data.resize(size);
  memcpy((void*) oggTrans->data.data(), (void*) ismInRoi.data, size );


  return oggTrans;

}

// Get topic name with callback: http://answers.ros.org/question/68434/get-topic-name-in-callback/?answer=68545#post-id-68545
// Using bind function: http://en.cppreference.com/w/cpp/utility/functional/bind
static std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> currentMapStack;
static std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> lastMapStack;
static nav_msgs::OccupancyGrid::ConstPtr msgTmp;
void doIsmFusion(const nav_msgs::OccupancyGrid::ConstPtr &msg, const std::string &topic) {


  // The current OGM (map) with which the ISM (msg) is fused
  mrpt::maps::COccupancyGridMap2D *map = NULL;

  // Sanity checks
  if (currentTileTfName.empty()) {
      ROS_WARN("currentTileTfName is empty: Skipping sensor fusion");
      return;
  }

  // Get the current map stack, or allocate new space
  mapRefresh.lock();
  std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack = currentMapStack;
  try {
//      for (auto& x: *mapStack) {
//        std::cout << x.first << ": " << x.second << '\n';
//      }
      map = mapStack->at(topic.c_str());
  } catch (...) { // It only throws if the topic is not in the map
      ROS_INFO("Add new map for topic %s", topic.c_str());
      map = mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def);
      currentMapStack->insert(
          std::pair<std::string,mrpt::maps::COccupancyGridMap2D*>(topic,map) );
      lastMapStack->insert(
          std::pair<std::string,mrpt::maps::COccupancyGridMap2D*>(topic,mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def)) );
  }
  std::string tileTfName = currentTileTfName;
  mapRefresh.unlock();

// Some debug stuff
//  tf::Pose ogmPose; tf::poseMsgToTF(msg->info.origin, ogmPose);
//  tf::Stamped<tf::Pose> originAsTargetPose(ogmPose, msg->header.stamp, msg->header.frame_id);
//  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, originAsTargetPose, msg->info.resolution, *listenerTf); // Check minimal function
//  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, originAsTargetPose, msg->info.resolution / 2, *listenerTf); // Check only resizing

  // Target pose is the origin of the OGM frame
  tf::Stamped<tf::Pose> targetPose(tf::Pose(tf::Quaternion(0,0,0,1)), msg->header.stamp, tileTfName + tileOriginTfSufixForRoiOrigin);
  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, targetPose, resolution, *listenerTf); // Check homography


  // Sanity check to speed up
  if (!ogmTransformed) {
      ROS_ERROR("ISM as OGM not valid");
      return;
  } else if (ogmTransformed->info.height <= 0 || ogmTransformed->info.width <= 0) {
      ROS_ERROR("ISM has no size");
      return;
  } else if (map == NULL) {
      ROS_ERROR("OGM is empty");
      return;
  } else if (map->getSizeY() <= 0 || map->getSizeX() <= 0) {
      ROS_ERROR("OGM has no size");
      return;
  } else {
      ROS_DEBUG("OK: Do the sensor fusion");
  }

  if (debug) {
      if (!topic.compare(debugIsmTopic)) {
        publisherIsmAsOgm.publish(ogmTransformed);
      }
      // Send OGM as point cloud
      //      sensor_msgs::PointCloudPtr ogmPtCloud = ogmCellsToPointCloud(msg, "world", *listenerTf, 0, 0, msg->info.width, msg->info.height);
      //      if(ogmPtCloud) {
      //          publisherIsmAsPointCloud.publish(ogmPtCloud);
      //      } else {
      //          ROS_ERROR("Debug: ISM as point cloud not valid");
      //      }
  }

  // Update a sensor scan:
  // The transformation already points to the lower left edge of the sensor scan
  // Rotation is not allowed for now
  // TODO Check with pose to speed up everything regarding the parameter ogmTf(..., true)
  // Assumptions are made for ISM and OGM: origin pose coincide, resolution is same
  const float max_x = ogmTransformed->info.height * ogmTransformed->info.resolution;
  const float min_x = 0.0f;
  const float max_y = ogmTransformed->info.width * ogmTransformed->info.resolution;
  const float min_y = 0.0f;
  const float transX = ogmTransformed->info.origin.position.x;
  const float transY = ogmTransformed->info.origin.position.y;
  const float transZ = ogmTransformed->info.origin.position.z;

  ROS_DEBUG("\n  Everything should be OK, if the pose is 0,0,0 and the frames are the same:"
            "  ISM coordinate x, y, z, frame_id: %f, %f, %f, %s\n"
            "  OGM frame name: %s\n\n",
            transX, transY, transZ, ogmTransformed->header.frame_id, targetPose.frame_id_.c_str());

//  const int mapCenterX = int((def.max_x - def.min_x) / def.resolution / 2.0);
//  const int mapCenterY = int((def.max_y - def.min_y) / def.resolution / 2.0);
//  const int mapOffsetXStart = transX / def.resolution + mapCenterX;
//  const int mapOffsetXEnd   = (transX + max_x - min_x) / def.resolution + mapCenterX;
//  const int mapOffsetYStart = transY / def.resolution + mapCenterY;
//  const int mapOffsetYEnd   = (transY + max_y - min_y) / def.resolution + mapCenterY;

  // Do a point wise sensor fusion
//  for (int idy = mapOffsetYStart; idy < mapOffsetYEnd ; ++idy) {
//    for (int idx = mapOffsetXStart; idx < mapOffsetXEnd ; ++idx) {

  for (int idy = 0; idy < min(ogmTransformed->info.height, map->getSizeY()) ; ++idy) {
    for (int idx = 0; idx < min(ogmTransformed->info.width, map->getSizeX()) ; ++idx) {
      // Get the index to get the correct cell out of the update
//      const int y = idx-mapOffsetXStart;
//      const int x = idy-mapOffsetYStart;
//      const int index = x + (y * msg->info.width);
      const int idxOgm = idx + (idy * map->getSizeX());
      const int idxIsm = idx + (idy * ogmTransformed->info.width);
      // Update the cell
      // There are only int values in the update:
      // -1 (unknown), 0 - 100 (occupancyness in percent)
//      if (msg->data.at(index) >= 0) {
//      if (ogmTransformed->data.at(idxIsm) >= 0) {
//      if (idx > 20) {
//        if(debug)INFO_MSG("Before: " << mapStack[mapIdx].getCell(idx,idy))
//        INFO_MSG("Before: " << map->getCell(idx,idy));
        map->updateCell(idx,idy,float(ogmTransformed->data.at(idxIsm)) / 100.0f);
//        map->updateCell(idx,idy,60.0f / 100.0f);
//        INFO_MSG("After: " << map->getCell(idx,idy));
//        if(debug)INFO_MSG("After: " << mapStack[mapIdx].getCell(idx,idy))
//      }
    }
  }

  // TEST: Draw the ISM in the upper left corner of the Occ
//  for (int idy = 0; idy < msg->info.height ; ++idy) {
//    for (int idx = 0; idx < msg->info.width ; ++idx) {
//      // Get the index to get the correct cell out of the update
//      const int index = idx + (idy * msg->info.width);
//      map->updateCell(idx,idy,float(msg->data.at(index)) / 100.0f);
//    }
//  }

  if (debug) {
    msgTmp = msg;
  }
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


nav_msgs::OccupancyGrid getIsmOutput(const mapserver_msgs::mapPrimitive &view, const mrpt::maps::COccupancyGridMap2D &input) {

  cv::Mat statisticMat , croppedMat; /*float matrices*/
  nav_msgs::OccupancyGrid output;

  std::stringstream os;
  os << view;
  ROS_INFO("view:\n%s\n", os.str().c_str());

  // Calculate the requested ROI
  const double xView = view.pose.pose.position.x; // Meter
  const double yView = view.pose.pose.position.y; // Meter
  const double wView = view.width * view.resolution; // Tiles * meter/tiles
  const double hView = view.height * view.resolution; // Tiles * meter/tiles
  const double dView = view.depth * view.resolution; // Tiles * meter/tiles
  const std::string sourceframe = view.frame_id.empty() ? machine::frames::names::BASE_LINK : view.frame_id;
  tf::Quaternion q;
  tf::quaternionMsgToTF(view.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  const double zRotView = yaw;


  // Resize the resolution
  cv::Size size(view.width, view.depth);
  cv::resize(statisticMat,croppedMat,size,0,0,cv::INTER_NEAREST);

  // Copy the meta information
  output.header.stamp = ros::Time::now();
  output.header.frame_id = sourceframe;
  output.info.resolution = view.resolution;
  output.info.width  = croppedMat.cols;
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

//class mapServerStockEdge: public rsb::patterns::LocalServer::Callback<rst::ms::mapPrimitive, rst::navigation::OccupancyGrid2DInt> {
//  boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> call(const std::string& /*methodName*/,
//              boost::shared_ptr<rst::ms::mapPrimitive> input) {
//        INFO_MSG( "mapServer-stockEdge method called" );
//        const boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> output(getIsmOutput(input, mapStack.at(ms::constants::mappingLayers::maps::stockEdge)));
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

///
/// \brief Add subscriber to newly emerged topic with super-topic
/// \param T [template] The message type
/// \param subList Subscriber list which will be expanded by subscriber if new topics emerge
/// \param f Function name with declaration "void myFun(const myRosMsgType::ConstPtr &msg, const std::string &topic)"
/// \param superTopic Prefix of the topic in the form of "/my/super/topic/"
/// \param debug Print additional information
/// \param n The node handle
///
template<typename T>
void advertiseSubscribers(
    std::vector<ros::Subscriber> &subList,
    void (&f)(const boost::shared_ptr< T const>&, const std::string&),
    const std::string &superTopic,
    const int &debug,
    ros::NodeHandle &n = ros::NodeHandle("~")) {

  ros::master::V_TopicInfo topics; // List of topics
  bool someSubscription = false;   // Indicator, if subscription occurs
  bool subscribe = true;           // Indicator, if subscription should take place

  if (ros::master::getTopics(topics)) {
    if (debug) { // Print topics
      ROS_DEBUG("List topics:\n");
      for (int idx = 0; idx < topics.size(); ++idx) {
        ROS_DEBUG("\n%d:\n"
                 "  TOPIC: %s\n"
                 "  TYPE : %s\n",
                 idx, topics.at(idx).name.c_str(), topics.at(idx).datatype.c_str());
      }
    }
    // Get a list topics which needs to be investigated (Check with ism_scope_prefix)
    std::vector<bool> topicsForSubscription(topics.size(), false);
    if ((superTopic.size() == 1) || superTopic.empty()) { // Check if "/" is the only content
        ROS_WARN("Using all scopes, because ism_scope_prefix is empty");
        topicsForSubscription = std::vector<bool>(topics.size(), true);
    } else { // Set the indicator to true, if the super-topic is the same
        for (int idx = 0; idx < topics.size(); ++idx) {
            if (!topics.at(idx).name.substr(0, superTopic.size()).compare(superTopic)) {
                topicsForSubscription.at(idx) = true;
            }
        }
    }
    // Check if the topics are already subscribed, otherwise add a subscription
    for (int idx = 0; idx < topics.size(); ++idx) {
        if (!topicsForSubscription.at(idx)) { // Skip if the topic is not valid
            continue;
        } else {
            someSubscription = true;
            if(subList.empty()) { // If we haven't subscribed to anything yet, get the first
                ROS_INFO("First subscription to: %s", topics.at(idx).name.c_str());
                subList.push_back(
                    n.subscribe<T>(topics.at(idx).name, 100, std::bind(f, std::placeholders::_1, topics.at(idx).name)));
            } else {
                subscribe = true;
                for (int idy = 0; idy < subList.size(); ++idy) { // Check if topic already subscribed, ...
                    if (!subList.at(idy).getTopic().compare(topics.at(idx).name)) {
                      subscribe = false;
                      break;
                    }
                }
                if (subscribe) { // ... otherwise do the subscription
                    ROS_INFO("Subscription %d to: %s", int(subList.size()+1), topics.at(idx).name.c_str());
                    subList.push_back(
                            n.subscribe<T>(topics.at(idx).name, 100, std::bind(f, std::placeholders::_1, topics.at(idx).name)));
                }
            }
        }
    }
    if (!someSubscription) {
        ROS_DEBUG("Nothing to subscribe");
    } else {
        ROS_DEBUG("Valid topics available");
    }
  } else {
    ROS_ERROR("No topics listable");
  }
}

void storeMaps (const std_msgs::String nameMsg) {

  if (nameMsg.data.empty()) {
      ROS_INFO("storeMaps: Store all maps");
  } else {
      // TODO Store maps defined in nameMsg
  }

  mapRefresh.lock();
  std::string tileTfName = currentTileTfName;
  auto mapStack = currentMapStack;
  mapRefresh.unlock();

  if (currentTileTfName.empty()) {
      ROS_WARN("storeMaps: No current frame name available. Skipping storage");
      return;
  }

  mapStorage( currentMapStack,
              std::string("OGM"),
              std::string(""),
              std::string("logodds"),
              def.resolution);
}


///
/// \brief Store the current tf tile name and swap the storage
/// \param nameMsg Name of the current tile tf
///
void tfTileNameHandler(const std_msgs::String nameMsg) {
  bool currentTileTfNameChange = false;
  mapRefresh.lock();
  if ((nameMsg.data.back() != currentTileTfName.back())) {
      if (currentTileTfName.empty()) {
          // First round, we bootstrap
          currentTileTfName = nameMsg.data;
      } else {
        std::swap(currentMapStack, lastMapStack);
        lastTileTfName    = currentTileTfName;
        currentTileTfName = nameMsg.data;
        currentTileTfNameChange = true;
      }
  }

  if (currentTileTfNameChange) {
    tf::StampedTransform transformRoiInWorld;
    try {
      listenerTf->waitForTransform(worldLink, lastTileTfName, ros::Time(0.0), ros::Duration(3.0));
      listenerTf->lookupTransform(worldLink, lastTileTfName, ros::Time(0.0), transformRoiInWorld);
    } catch(const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("tfTileNameHandler: %s", excStr.c_str());
      mapRefresh.unlock();
      return;
    }
    ROS_INFO("NEW MAP");

    // Wait until all references are gone
    std::size_t lockCnt = 0;
    const std::size_t lockCntMax = 200000; // 2 seconds if we sleep for 10 us
    while(!(lastMapStack.unique() && currentMapStack.unique())) {
        usleep(10);
        if (++lockCnt > lockCntMax) {
            ROS_ERROR("tfTileNameHandler: Locked for to long, skip storage (maybe deadlock or out if resources?)");
            mapRefresh.unlock();
            return;
        }
    }

    mapRefreshAndStorage( lastMapStack,                          // Map to shift/store/reset
                          currentMapStack,                       // The result of the shifted map
                          transformRoiInWorld,                   // Transform
                          std::string("OGM"),                    // Kind of map
                          std::string(""),                       // Format (empty: Take from type specifier)
                          std::string("logodds"),                // Unit
                          def.resolution,                        // Resolution per tile
                          !dontStoreMaps,                        // Info if maps should be stored
                          bool(shiftMap),                        // Info if maps should be shifted
                          !shiftMap,                             // If map is not shifted, reset the content of mapStack
                          mrpt::maps::COccupancyGridMap2D::p2l(0.5)); // Fill-up value (logodds(0.5f) = unknown)
  }

  mapRefresh.unlock();

}

///
/// \brief Store the current tf tile name and swap the storage
/// \param msg tuple of position, NavSat, and name name of the current tile tf
///
static mapserver_msgs::pnsTuple lastPnsTuple;
void tupleHandler(const mapserver_msgs::pnsTuple msg) {
  bool currentTileTfNameChange = false;
  mapRefresh.lock();
  if ((msg.string.data.back() != currentTileTfName.back())) {
      if (currentTileTfName.empty()) {
          // First round, we bootstrap
          currentTileTfName = msg.string.data;
          lastPnsTuple = msg;
      } else {
        std::swap(currentMapStack, lastMapStack);
        lastTileTfName    = currentTileTfName;
        currentTileTfName = msg.string.data;
        currentTileTfNameChange = true;
      }
  }

  if (currentTileTfNameChange) {
    ROS_INFO("NEW MAP");
    tf::StampedTransform transformRoiInWorld;
    transformRoiInWorld.setOrigin(tf::Vector3(lastPnsTuple.point.x, lastPnsTuple.point.y, lastPnsTuple.point.z));
    transformRoiInWorld.setRotation(tf::Quaternion(0,0,0,1));

    // Wait until all references are gone
    std::size_t lockCnt = 0;
    const std::size_t lockCntMax = 200000; // 2 seconds if we sleep for 10 us
    while(!(lastMapStack.unique() && currentMapStack.unique())) {
        usleep(10);
        if (++lockCnt > lockCntMax) {
            ROS_ERROR("tfTileNameHandler: Locked for to long, skip storage (maybe deadlock or out if resources?)");
            mapRefresh.unlock();
            return;
        }
    }

    std::stringstream navSatSs;
    navSatSs << std::setprecision(12)
        << "lat_" << msg.navsat.latitude << "_"
        << "lon_" << msg.navsat.longitude << "_"
        << "alt_" << msg.navsat.altitude;

     mapRefreshAndStorage( lastMapStack,                          // Map to shift/store/reset
                           currentMapStack,                       // The result of the shifted map
                           transformRoiInWorld,                   // Transform
                           std::string("OGM"),                    // Kind of map
                           std::string(""),                       // Format (empty: Take from type specifier)
                           std::string("logodds"),                // Unit
                           def.resolution,                        // Resolution per tile
                           !dontStoreMaps,                        // Info if maps should be stored
                           bool(shiftMap),                        // Info if maps should be shifted
                           !shiftMap,                             // If map is not shifted, reset the content of mapStack
                           mrpt::maps::COccupancyGridMap2D::p2l(0.5), // Fill-up value (logodds(0.5f) = unknown)
                           true,
                           navSatSs.str());
    // Store the current tile information as next last one
    lastPnsTuple = msg;
  }

  mapRefresh.unlock();


}

#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud2.h>

void formatAndSendGrid(std::vector<std::string> &list,
                       std::string &frame,
                       const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack,
                       ros::NodeHandle &n,
                       const std::string topicPrefixGrid = "",
                       const std::shared_ptr<tf::Pose> tfPose = NULL,
                       std::string topicSufixGrid = "/grid",
                       std::string topicSufixPointCloud = "/pointCloud") {
  const char fName[] = "formatAndSendGrid";
  const float gridSpacing_m = 0.01;
  const float minDrawOccupancyUpdateCertainty = 0.5;

  // Sanity checks
  if (frame.empty()) {
      ROS_WARN("%s: formatAndSendGrid: frame is empty", fName);
      return;
  } else if (!mapStack) {
      ROS_WARN("%s: mapStack pointer is empty", fName);
      return;
  }
  if (list.empty()) {
      ROS_WARN_ONCE("%s: list is empty -> sending all maps", fName);
  }

  // Initialize the iterator and decide which one to use
  // If the list is empty, process the whole map stack, otherwise
  // send the content defined in list
  std::map<std::string, mrpt::maps::COccupancyGridMap2D*>::iterator mapIt;
  std::vector<std::string>::iterator listIt;
  int idxIt = 0;
  if (list.size() > 0) {
      listIt = list.begin();
  }
  if (mapStack->size() > 0) {
      mapIt = mapStack->begin();
  } else {
      ROS_WARN("%s: mapStack is empty", fName);
      return;
  }

  // Allocate the maximal size for the points to send
  std::vector<geometry_msgs::Point> points(mapIt->second->getSizeY() * mapIt->second->getSizeX());

  while (true) {
      // Check if we have to work with the list iterator, if not ...
      if (list.size() > 0) {
          if (listIt == list.end()) {
              break;
          }
          mapIt = mapStack->find(*listIt);
          if (mapIt == mapStack->end()) {
              ROS_WARN("%s: No entry for '%s'", fName,  listIt->c_str());
              continue;
          } else if (mapIt->second == NULL) {
              ROS_WARN("%s: Map pointer for '%s' is empty", fName, listIt->c_str());
              continue;
          }
          ++listIt;
      } else { // ... just process the full map stack
          if (idxIt > 0) {
              ++mapIt;
          } // else, it is the first round
          if(mapIt == mapStack->end()) {
              break;
          }
      }

      // Check for publisher and advertise one, if missing
      static std::map<std::string, ros::Publisher> mapPublisher;
      auto publisherIt = mapPublisher.find(mapIt->first);
      if (publisherIt == mapPublisher.end()) {
          ROS_INFO("%s: Advertise map publisher with topic %s", fName, mapIt->first.c_str());
          mapPublisher.insert(
                    std::pair<std::string,ros::Publisher>(mapIt->first,
                                                          n.advertise<nav_msgs::GridCells>(topicPrefixGrid + mapIt->first + topicSufixGrid, 1)) );
          publisherIt = --mapPublisher.end();
      }

      // Format the map
      tf::Point pointTf;
      geometry_msgs::Point pointMsg;
      nav_msgs::GridCells msg;
      msg.cell_height = resolution;
      msg.cell_width = resolution;
      auto pointsIt = points.begin();
      for (int idy = 0; idy < mapIt->second->getSizeY(); ++idy) {
        for (int idx = 0; idx < mapIt->second->getSizeX(); ++idx) {
            // We get values from 0 .. 100
            if (mapIt->second->getCell(idx, idy) >  minDrawOccupancyUpdateCertainty) {
              pointTf[0] = (float(idx) * resolution);
              pointTf[1] = (float(idy) * resolution);
              pointTf[2] = float(idxIt) * gridSpacing_m;  // Overlay each layer by some distance

              if (tfPose) {  // Check for transformation
                  pointTf = *tfPose * pointTf;
              }
              tf::pointTFToMsg(pointTf, pointMsg);
              *pointsIt = pointMsg;
              if (pointsIt != points.end()) {
                  ++pointsIt;
              }
            }
        }
      }
      msg.cells.assign(points.begin(), pointsIt);

      // Publish the map
      msg.header.frame_id = frame;
      msg.header.stamp = ros::Time::now();
      publisherIt->second.publish(msg);

      ++idxIt;
  }


}

///
/// \brief Shows the current RPC if debug is on
///
//void showRpc() {
//    if (debug) {
//        ROS_INFO("DEBUG: Show the requests");
//        if(mapStackStatisticDebug.empty() || mapStackStatisticDebug.empty()) {
//            ROS_ERROR("mapStackStatistic is empty");
//            return;
//        }
//        cv::Mat mapStackStatisticRequestDebugTmp, mapStackStatisticDebugTmp;
//        mtxShowRpc.lock();
//            mapStackStatisticRequestDebug.copyTo(mapStackStatisticRequestDebugTmp);
//            mapStackStatisticDebug.copyTo(mapStackStatisticDebugTmp);
//            mapStackStatisticDebugTmp.release();
//            mapStackStatisticRequestDebugTmp.release();
//            cv::RotatedRect rectTmp = rect;
//        mtxShowRpc.unlock();
//
//        std::stringstream os;
//        os << ros::Time::now();
//        {
//            cv::imshow(std::string("mapStackStatisticRequest"), mapStackStatisticRequestDebugTmp);
//            cv::setWindowTitle(std::string("mapStackStatisticRequest"),
//                               std::string("mapStackStatisticRequest at ") + os.str());
//        }
//        {
//            // Draw the request in pseudo-colors
//            if (debugDrawRpc) {
//                // TODO Make a cast which is possible to handle all data types
//                utils::castCopyImage(mapStackStatisticDebugTmp, mapStackStatisticDebugTmp, CV_16UC1);
//                mapStackStatisticDebugTmp.convertTo(mapStackStatisticDebugTmp, CV_8UC3);
//                cv::cvtColor( mapStackStatisticDebugTmp, mapStackStatisticDebugTmp, CV_GRAY2BGR );
//                utils::drawRotatedRectInImage(mapStackStatisticDebugTmp, rect, cv::Scalar(0,0,255));
//            }
//
//            cv::imshow(std::string("mapStackStatistic"), mapStackStatisticDebugTmp);
//            cv::setWindowTitle(std::string("mapStackStatistic"),
//                               std::string("mapStackStatistic at ") + os.str());
//        }
//        cv::waitKey(1);
//    }
//}


void addMapToResponse(const std::string &name, mrpt::maps::COccupancyGridMap2D* map, mapserver::ismStackFloat::Response &response) {
  if (map == NULL) {
      ROS_WARN("Map pointer empty");
      return;
  }

  response.response.mapNames.strings.resize(response.response.mapNames.strings.size() + 1);
  response.response.mapStack.resize(response.response.mapStack.size() + 1);

  auto mapName = response.response.mapNames.strings.end()-1;
  *mapName = name;

  const std::size_t numCells = map->getSizeX() * map->getSizeY();
  // Copy the cells
  auto mapRes = (response.response.mapStack.end()-1);
  mapRes->map.resize(numCells);
  for (int idx = 0; idx < numCells; ++idx) {
      int xIdx = idx % map->getSizeX();
      int yIdx = idx / map->getSizeX();
      mapRes->map.at(idx) = map->getCell(xIdx, yIdx);
  }

  mapRes->header.stamp = ros::Time::now();
  mapRes->info.height = map->getSizeY();
  mapRes->info.width = map->getSizeX();
  const tf::Pose pose = tf::Pose(tf::Quaternion(0,0,0,1));
  tf::poseTFToMsg(pose, mapRes->info.origin);
  mapRes->info.resolution = map->getResolution();
}

bool mapStatServerMapStack(mapserver::ismStackFloat::Request &req, mapserver::ismStackFloat::Response &res) {
  mapRefresh.lock();
  auto mapStack = currentMapStack;
  std::string tileName = currentTileTfName;
  mapRefresh.unlock();

  ROS_INFO( "statMapserver-mapStack method called" );
  if (req.request.strings.empty()) {
      ROS_INFO( "Respond the full map stack" );
      for (auto it=mapStack->begin(); it!=mapStack->end(); ++it) {
          addMapToResponse(it->first, it->second, res);
      }
  } else {
      for (auto it=req.request.strings.begin(); it!=req.request.strings.end(); ++it) {
          auto itMapStack = mapStack->find(*it);
          if (itMapStack != mapStack->end()) {
              addMapToResponse(itMapStack->first, itMapStack->second, res);
          }
      }
  }


  res.response.header.frame_id = tileName + tileOriginTfSufixForRoiOrigin;
  res.response.header.stamp = ros::Time::now();
  for (auto it = res.response.mapStack.begin(); it != res.response.mapStack.end(); ++it) {
      it->header.frame_id = res.response.header.frame_id;
  }
  ROS_INFO( "statMapserver-mapStack method called: Finish" );
  return true;
}


int main(int argc, char **argv){

  // ROS
  ros::init(argc, argv, "mapserver_stat");
  ros::NodeHandle n("~");

  n.param<std::string>("tile_origin_tf_prefix", tileOriginTfPrefix, "map_base_link_");
  n.param<std::string>("tile_origin_tf_sufix_for_roi_origin", tileOriginTfSufixForRoiOrigin, machine::frames::names::ROI_ORIGIN);
  n.param<std::string>("current_tf_name_topic", currentTfNameTopic, "/currentTfTile");
  n.param<std::string>("current_tuple_topic", currentTupleTopic, "");
  n.param<std::string>("world_link", worldLink, "odom");
  n.param<std::string>("store_maps_topic", storeMapsTopic, "/storemaps");
  n.param<std::string>("req_topic_map_stack", reqTopicMapStack, "/reqMapStack");
  n.param<double>("idle_startup_time", idleStartupTime_s, -1.0); // Wait before mapping (< 0 to disable)

  n.param<int>("debug", debug, 0); // Enable debug outputs
  n.param<std::string>("debug_ism_topic", debugIsmTopic, "/ism/radar/tracking/radar_return"); // The topic of the ISM which is resend as transformed ISM in the mapserver frame
  std::string debugTopic;
  n.param<std::string>("debug_topic", debugTopic, "/amiro2/ism/cam"); // The topic of the fused map to show via opencv
  n.param<int>("test", doTest, 0); // Enable testing
  n.param<double>("resolution", resolution, double(mapping::discreteResolution)); // Resolution of map in meter/cell
  n.param<float>("max_occupancy_update_certainty", maxOccupancyUpdateCertainty, 0); // Maximum update uncertainty
  n.param<float>("max_distance_insertion", maxDistanceInsertion, 0); // Maximum distance insertion
  n.param<float>("max_x_m", max_x, mapping::roi::xMax); // Maxmium value of x in meter
  n.param<float>("min_x_m", min_x, mapping::roi::xMin); // Minimum value of x in meter
  n.param<float>("max_y_m", max_y, mapping::roi::yMax); // Maxmium value of y in meter
  n.param<float>("min_y_m", min_y, mapping::roi::yMin); // Minimum value of y in meter
  n.param<float>("uncertainty_boundary", uncertaintyBoundary, 0.5); // Uncertainty boundary for displaying a feature (standard: 0.5 (unknown))
  n.param<float>("mapInit_value", mapInitValue, 0.5); // Probability (0 .. 1.0) for initializing the map
  n.param<std::string>("ism_scope_prefix", ismScopePrefix, scopes::map::super::ogm); // Scope prefix for the inverse sensor models
  n.param<std::string>("map_storage_location", mapStorageLocation, "/tmp/"); // Location for the maps to store (preceeding / is needed, like /tmp/)
  n.param<int>("shift_map", shiftMap, 0); // Shift (1) or wipe (0) the map on center_dresch
  n.param<int>("dont_store_maps", dontStoreMaps, 0);
  n.param<float>("rate", rate, 1); // Rate for publishing debug information
  std::string topicDebugGridPrefix;
  n.param<std::string>("topic_debug_grid_prefix", topicDebugGridPrefix, "/viz");

  ROS_INFO_STREAM("Enable debug outputs: "            << debug                        );
  ROS_INFO_STREAM("Enable testing: "                  << doTest                       );
  ROS_INFO_STREAM("Resolution of map in meter/cell: " << resolution                   );
  ROS_INFO_STREAM("Maximum update uncertainty: "      << maxOccupancyUpdateCertainty  );
  ROS_INFO_STREAM("Maximum distance insertion: "      << maxDistanceInsertion         );
  ROS_INFO_STREAM("Maxmium value of x in meter: "     << max_x                        );
  ROS_INFO_STREAM("Minimum value of x in meter: "     << min_x                        );
  ROS_INFO_STREAM("Maxmium value of y in meter: "     << max_y                        );
  ROS_INFO_STREAM("Minimum value of y in meter: "     << min_y                        );
  ROS_INFO_STREAM("Uncertainty boundary for displaying a feature (standard: 0.5 (unknown)): "     << uncertaintyBoundary);
  ROS_INFO_STREAM("Probability (0 .. 1.0) for initializing the map: " << mapInitValue);
  // Sanity check for ism_scope_prefix, so it looks like /some/scope
  if (ismScopePrefix.empty()) {
    ismScopePrefix = "/";
  } else {
    if (ismScopePrefix.at(0) != '/') {
      ismScopePrefix = std::string("/").append(ismScopePrefix);
    }
    if (ismScopePrefix.at(ismScopePrefix.size()-1) != '/') {
      ismScopePrefix.append("/");
    }
  }

  if (mapInitValue > uncertaintyBoundary)
    ROS_WARN_STREAM("'uncertaintyBoundary' is less than 'mapInitValue', display might be corrupted");
  ROS_INFO_STREAM("Scope prefix for the inverse sensor models: " << ismScopePrefix    );
  ROS_INFO_STREAM("Location for the maps to store: " << mapStorageLocation);


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

  // Allocate space for the map and mapstacks
  storageMapBuffer = cv::Mat(mapSizeX, mapSizeY, CV_8UC1);
  currentMapStack = std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>>(new std::map<std::string, mrpt::maps::COccupancyGridMap2D*>);
  lastMapStack = std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>>(new std::map<std::string, mrpt::maps::COccupancyGridMap2D*>);
//  msgTmp = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid);

  // Create the maps
//  for (unsigned char mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
//    mapStack.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
//    mapStackStorageTemp.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
//    mapStackShiftTemp.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
//  }
//  fillMapStack(mapStack, mapInitValue);
//  fillMapStack(mapStackStorageTemp, mapInitValue);
//  fillMapStack(mapStackShiftTemp, mapInitValue);

  // Prepare subscriber for all future OGM
  listenerTf = new tf::TransformListener;
  std::vector<ros::Subscriber> subIsmList; // List of subscribers TODO Replace by std::map?
  if (currentTupleTopic.empty()) {
      subscriberTfTileName = n.subscribe<std_msgs::String>(currentTfNameTopic, 2, tfTileNameHandler);
  } else {
      subscriberTuple = n.subscribe<mapserver_msgs::pnsTuple>(currentTupleTopic, 2, tupleHandler);
  }
  subscriberStoreMaps = n.subscribe<std_msgs::String>(storeMapsTopic, 1, storeMaps);



  publisherIsmAsPointCloud = n.advertise<sensor_msgs::PointCloud>("foo",1);
  publisherIsmAsOgm = n.advertise<nav_msgs::OccupancyGrid>(std::string("debug") + debugIsmTopic,1);

  // Prepare ROS service
  // TODO: Do this on demand for given subscribed topics
  const std::string s("/");
  ros::ServiceServer service_mapStack     = n.advertiseService(reqTopicMapStack/*scopes::map::statServer::parent + s + scopes::map::statServer::requests::mapStack*/, mapStatServerMapStack);

//  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::compressedMapImage , mapServerCompressedMap);
//  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::stockEdge , mapServerStockEdge);
//  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::singleLayerOgm , mapServerSingleLayerOgm);


  ros::AsyncSpinner spinner(5);
  spinner.start();
  // Do stuff periodically
  ros::Rate _rate(rate);
  while(ros::ok()) {
      // Plot map
//      boost::shared_ptr<cv::Mat> image(doColorMapCallback());
//      cv::imshow( "Current View", *image );                    // Show our image inside it.
//      cv::waitKey(1); // Update the window
      if (debug) {
        try {
            std::shared_ptr<cv::Mat> image(mrptOggToGrayScale(*currentMapStack->at(debugTopic)));
  //            std::shared_ptr<cv::Mat> image(rosOggToGrayScale(msgTmp));
              if (image) {
                  cv::flip(*image, *image, 0);
                  cv::imshow( "Current View", *image);
                  cv::waitKey(1); // Update the window
              }
        } catch (...) {
            ROS_WARN("Debug visualization: No such topic '%s' in currentMapStack", debugTopic.c_str());
        }
        // Publish the maps
        std::vector<std::string> foo;
//        tf::Vector3 trans = tf::Vector3(tfScalar(-(max_x - min_x)/2.0f), tfScalar(-(max_y - min_y)/2.0f), tfScalar(0.0f));
//        std::shared_ptr<tf::Pose> poseOffset = std::shared_ptr<tf::Pose>(new tf::Pose(tf::Quaternion(0,0,0,1), trans));
        std::string reference = currentTileTfName + tileOriginTfSufixForRoiOrigin;
        formatAndSendGrid(foo, reference, currentMapStack, n, topicDebugGridPrefix);
      }

//      showRpc();

      // Add new subscribers
      advertiseSubscribers<nav_msgs::OccupancyGrid>(subIsmList, doIsmFusion, ismScopePrefix, debug, n);
      _rate.sleep();
  }

  delete listenerTf;

  return 0;
}
