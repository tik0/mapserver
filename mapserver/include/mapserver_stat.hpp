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

#include "Constants.hpp"
using namespace ms::constants;
using namespace ms::constants::mappingLayers;
namespace msNumeric = ms::constants::numeric;

// OpenCV
#include <opencv2/opencv.hpp>

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <mapserver_msgs/pnsTuple.h>
#include <rosgraph_msgs/Log.h>
#include <tf_conversions/tf_eigen.h>
#include <rosapi/Topics.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud2.h>
#include <type_traits>

#include "mapserver.hpp"

#include <utils.h>

class MapserverStat : private Mapserver<mrpt::maps::COccupancyGridMap2D,
    nav_msgs::OccupancyGrid, mrpt::maps::COccupancyGridMap2D::cellType,
    MapserverStat> {

 public:
  mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
 private:
  ros::NodeHandle n;
 public:
  std::string topicDebugGridPrefix;
  float uncertaintyBoundary = mapping::ogm::minDrawOccupancyUpdateCertainty;
  float maxOccupancyUpdateCertainty = mapping::ogm::maxOccupancyUpdateCertainty;
  std::string debugTopic;
  std::string debugIsmTopic;

  ros::ServiceServer service_mapStack;
  ros::Publisher publisherIsmAsPointCloud, publisherIsmAsOgm;

  MapserverStat(ros::NodeHandle& nh);
  virtual ~MapserverStat() {
  }

 protected:

 private:

  // TODO Do we need this really?
  nav_msgs::OccupancyGrid::ConstPtr msgTmp;

 public:
  // Add just some picture of each other
  boost::shared_ptr<cv::Mat> doColorMapCallback(
      std::vector<mrpt::maps::COccupancyGridMap2D> mapStack);
  std::shared_ptr<cv::Mat> mrptOggToGrayScale(
      mrpt::maps::COccupancyGridMap2D &map);
  std::shared_ptr<cv::Mat> rosOggToGrayScale(
      nav_msgs::OccupancyGrid::ConstPtr map);

  ///
  /// \brief Returns the pose in the desired frame
  /// \param poseInSourceFrame Stamped pose in the source frame
  /// \param targetFrame The target frame in which the returned pose will reside
  /// \param tfListener A transformer
  /// \return Transformed pose. Shared pointer is empty if an error occurs
  ///
  std::shared_ptr<tf::Stamped<tf::Pose>> getPoseInFrame(
      const tf::Stamped<tf::Pose> &poseInSourceFrame,
      const std::string &targetFrame, const tf::TransformListener &tfListener);

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
  std::shared_ptr<std::vector<tf::Point>> ogmCellCoordinatesToPoints(
      const nav_msgs::OccupancyGrid::ConstPtr ogm,
      const std::string &targetFrame, const tf::TransformListener &tfListener,
      const std::size_t &idxStart, const std::size_t &idyStart,
      const std::size_t &idxWidth, const std::size_t &idyHeight);
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
  sensor_msgs::PointCloud::Ptr ogmCellsToPointCloud(
      const nav_msgs::OccupancyGrid::ConstPtr ogm,
      const std::string &targetFrame, const tf::TransformListener &tfListener,
      const std::size_t &idxStart, const std::size_t &idyStart,
      const std::size_t &idxWidth, const std::size_t &idyHeight);
  void correctInvalidOrientation(tf::Pose &pose);
  ///
  /// \brief Returns the coordinates of the four corner points of the OGM in the given frame
  /// \param ogm The OGM which cell coordinates are converted to points
  /// \param targetFrame The target frame in which the point is calculated (target frame equals OGM frame if empty)
  /// \param tfListener A transformer
  /// \return Vector of points starting top-left. Shared pointer is empty if an error occurs
  ///
  std::shared_ptr<std::vector<tf::Point>> getOgmCornerPoints(
      const nav_msgs::OccupancyGrid::ConstPtr ogm,
      const std::string &targetFrame, const tf::TransformListener &tfListener);
  ///
  /// \brief Returns the coordinates of the four corner points of the OGM in the given pose
  /// \param ogm The OGM which cell coordinates are converted to points
  /// \param targetPose The target Pose in which the point is calculated
  /// \param tfListener A transformer
  /// \return Vector of points starting top-left. Shared pointer is empty if an error occurs
  ///
  std::shared_ptr<std::vector<tf::Point>> getOgmCornerPoints(
      const nav_msgs::OccupancyGrid::ConstPtr ogm,
      const tf::Stamped<tf::Pose> &targetPose,
      const tf::TransformListener &tfListener);
  ///
  /// \brief Resize a OGM to the desired resolution
  /// \param ogm The OGM which should be resized
  /// \param targetResolution The target resolution of the OGM
  /// \return Resized OGM
  ///
  nav_msgs::OccupancyGrid::ConstPtr ogmResize(
      const nav_msgs::OccupancyGrid::ConstPtr ogm,
      const float &targetResolution);
  ///
  /// \brief Warps a OGM to the desired pose and resolution such that the new OGM is aligned with this pose
  /// \param ogm The OGM which should be transformed
  /// \param targetPose The target pose in which the new OGM resides
  /// \param targetResolution The target resolution of the OGM
  /// \param tfListener A transformer
  /// \param resetPoseToOgmBoundary The origin of the new OGM will not reside in targetOrigin, but with respect to the minimum possible size of the which pose lies in the XY-plane of targetOrigin.
  /// \return OGM in the new frame. If message pointer is empty, an error occurred
  ///
  nav_msgs::OccupancyGrid::ConstPtr ogmTf(
      const nav_msgs::OccupancyGrid::ConstPtr ogm,
      const tf::Stamped<tf::Pose> &targetOrigin, const float &targetResolution,
      const tf::TransformListener &tfListener,
      const bool resetPoseToOgmBoundary = false);
  void calcIsm4Mapserver(const double xIsmCenter_m, const double yIsmCenter_m,
                         const double phiIsm_rad, const double ismResolution,
                         cv::Mat &ismInRoi, cv::Mat &ism);
  nav_msgs::OccupancyGrid::Ptr oggTf(
      const std::string &targetFrame,
      const nav_msgs::OccupancyGrid::ConstPtr ismRos, const double targetRes =
          -1/*meter/tile*/);
  void doIsmFusion(const nav_msgs::OccupancyGrid::ConstPtr &msg,
                   const std::string &topic);
  nav_msgs::OccupancyGrid getIsmOutput(
      const mapserver_msgs::mapPrimitive &view,
      const mrpt::maps::COccupancyGridMap2D &input);
  void formatAndSendGrid(
      std::vector<std::string> &list,
      std::string &frame,
      const std::shared_ptr<
          std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack,
      ros::NodeHandle &n, const std::string topicPrefixGrid = "",
      const std::shared_ptr<tf::Pose> tfPose = NULL,
      std::string topicSufixGrid = "/grid", std::string topicSufixPointCloud =
          "/pointCloud");
  void addMapToResponse(const std::string &name,
                        mrpt::maps::COccupancyGridMap2D* map,
                        mapserver::ismStackFloat::Response &response);
  bool mapStatServerMapStack(mapserver::ismStackFloat::Request &req,
                             mapserver::ismStackFloat::Response &res);
  void spinOnce();
  void spin();

};
