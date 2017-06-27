// MRPT
#include <mrpt/maps/CMultiMetricMap.h>
#if defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS)
#define OCCUPANCY_GRIDMAP_CELL_SIZE 1
#else // defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
#define OCCUPANCY_GRIDMAP_CELL_SIZE 2
#endif


#include "mapserver.hpp"
#include <Constants.hpp>
#include <nav_msgs/GridCells.h>
#include <utils.h>

using namespace constants;
using namespace constants::mappingLayers;
namespace numerics = constants::numeric;

class MapserverStat : public Mapserver<mrpt::maps::COccupancyGridMap2D,
    nav_msgs::OccupancyGrid, mrpt::maps::COccupancyGridMap2D::cellType,
    MapserverStat> {

 private:

  //! Storage which shows the callback message as image for debug
  nav_msgs::OccupancyGrid::ConstPtr msgDebug;
  //! The node handle
  ros::NodeHandle n;
  //! Definition of a map
  mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
  //! Prefix for sending the debug messages
  std::string topicDebugGridPrefix;
  //! Uncertainty boundary s.t. the offset around 0.5 which is interpreted as unknown
  float uncertaintyBoundary = mapping::ogm::minDrawOccupancyUpdateCertainty;
  //! Maximum value to fuse with
  float maxOccupancyUpdateCertainty = mapping::ogm::maxOccupancyUpdateCertainty;
  //! Topic for sending debug grid messages
  std::string debugTopic;
  //! Topic for sending the transformed ISM
  std::string debugIsmTopic;
  //! The mapstack service
  ros::ServiceServer service_mapStack;
  //! Publisher for sending the transformed ISM as occupancy grid message
  ros::Publisher publisherIsmAsOgm;
  //! Publisher for sending the transformed ISM as point cloud
  ros::Publisher publisherIsmAsPointCloud;

 public:

  ///
  /// \brief The constructor
  /// \param nh The node handle
  ///
  MapserverStat(ros::NodeHandle& nh);

  ///
  /// \brief The destructor
  ///
  virtual ~MapserverStat() {
  };

  ///
  /// \brief Returns the data pointer of the map
  /// \param map The map
  /// \return Pointer to the begin of the map
  ///
  virtual void* getRawData(mrpt::maps::COccupancyGridMap2D *map);

  ///
  /// \brief Converts the mapstack to a colored image
  /// \param mapStack The mapstack to convert
  /// \return Colored map as image
  ///
  boost::shared_ptr<cv::Mat> doColorMapCallback(
      std::vector<mrpt::maps::COccupancyGridMap2D> mapStack);

  ///
  /// \brief converts MRPT occ map to grayscale image
  /// \param map The map to convert
  /// \return Shared pointer to the image. Is Null pointer if image conversion was unsuccessful
  std::shared_ptr<cv::Mat> mrptOggToGrayScale(
      mrpt::maps::COccupancyGridMap2D &map);

//  std::shared_ptr<cv::Mat> rosOggToGrayScale(
//      nav_msgs::OccupancyGrid::ConstPtr map);

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


  // Get topic name with callback: http://answers.ros.org/question/68434/get-topic-name-in-callback/?answer=68545#post-id-68545
  // Using bind function: http://en.cppreference.com/w/cpp/utility/functional/bind
  ///
  /// \brief Receive an occupancy message and fuse it with the mapstack
  /// \param msg The occupancy message
  /// \param topic Topic name which is used as identifier for the mapstack to fuse the message
  ///
  void doIsmFusion(const nav_msgs::OccupancyGrid::ConstPtr &msg,
                   const std::string &topic);
  nav_msgs::OccupancyGrid getIsmOutput(
      const mapserver_msgs::mapPrimitive &view,
      const mrpt::maps::COccupancyGridMap2D &input);


  ///
  /// \brief Send a list formated maps
  /// \param list List of map identifiers to send
  /// \param frame Frame id for the header
  /// \param mapStack The mapstack wich is going to be formated
  /// \param topicPrefixGrid Prefix to add to the grid
  /// \param tfPose Extra pose for transformation
  /// \param topicSuffixGrid Suffix for grid map message
  ///
  void formatAndSendGrid(
      std::vector<std::string> &list,
      std::string &frame,
      const std::shared_ptr<
          std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack,
      ros::NodeHandle &n, const std::string topicPrefixGrid = "",
      const std::shared_ptr<tf::Pose> tfPose = NULL,
      std::string topicSuffixGrid = "/grid");

  ///
  /// \brief Adds a map to a requested response
  /// \param name Name of the map to add
  /// \param map Pointer to the map
  /// \param response The response to append
  ///
  void addMapToResponse(const std::string &name,
                        mrpt::maps::COccupancyGridMap2D* map,
                        mapserver::ismStackFloat::Response &response);


  ///
  /// \brief Requesting the maps stack with float values
  /// \param req The request
  /// \param res The response
  /// \return True if response was successful
  ///
  bool mapStatServerMapStack(mapserver::ismStackFloat::Request &req,
                             mapserver::ismStackFloat::Response &res);

  ///
  /// \brief The spinning realization for this class
  ///
  virtual void spinOnce();

  ///
  /// \brief Translates the content of a map and fills up the boarders
  /// \param map The map
  /// \param offsetx Offset to move in X pixel direction
  /// \param offsety Offset to move in Y pixel direction
  /// \param fillProbability_logodds Fill-up value as probability in logodds
  ///
  virtual void translateMap(
      mrpt::maps::COccupancyGridMap2D &map,
      int offsetx = 0,
      int offsety = 0,
      mrpt::maps::COccupancyGridMap2D::cellType fillProbability_logodds =
          mrpt::maps::COccupancyGridMap2D::p2l(
              mapping::ogm::unknownOccupancyUpdateCertainty));

  ///
  /// \brief Set all map tiles of all maps to the given value
  /// \param mapStack The mapstack to reset
  /// \param fillValue Probability fill-up value in loggodds
  ///
  virtual void fillMapStack(
      std::vector<mrpt::maps::COccupancyGridMap2D> &mapStack,
      mrpt::maps::COccupancyGridMap2D::cellType fillValue =
          mrpt::maps::COccupancyGridMap2D::cellType(
              mapping::ogm::unknownOccupancyUpdateCertainty));

  ///
  /// \brief Set all map tiles of all maps to the given value
  /// \param mapStack The mapstack to reset
  /// \param fillValue Probability fill-up value
  ///
  void fillMapStack(std::vector<mrpt::maps::COccupancyGridMap2D> &mapStack,
                    float fillValue =
                        mapping::ogm::unknownOccupancyUpdateCertainty);

  ///
  /// \brief Set all map tiles of a map to the given value
  /// \param mapStack The mapstack to reset
  /// \param fillValue Probability fill-up value in loggodds
  ///
  virtual void fillMap(
      mrpt::maps::COccupancyGridMap2D &map,
      mrpt::maps::COccupancyGridMap2D::cellType fillValue =
          mrpt::maps::COccupancyGridMap2D::cellType(
              mapping::ogm::unknownOccupancyUpdateCertainty));

};
