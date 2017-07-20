#include <boost/math/special_functions/erf.hpp>
#include <Constants.hpp>
#include <nav_msgs/GridCells.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "mapserver.hpp"

using namespace constants;
using namespace constants::mappingLayers;
namespace numerics = constants::numeric;

class MapserverRaw : public Mapserver<short, nav_msgs::OccupancyGrid, short,
    MapserverRaw> {

 private:

  //! The node handle
  ros::NodeHandle n;

  std::mutex mtxSwap;
  std::mutex mtxShowRpc;

 public:

  ///
  /// \brief The constructor
  /// \param nh The node handle
  ///
  MapserverRaw(ros::NodeHandle& nh);

  ///
  /// \brief The destructor
  ///
  virtual ~MapserverRaw() {
  }
  ;

  //! The subscriber for the sensor data (Laser, Pointcloud, or Pointcloud2)
  ros::Subscriber subscriberData;
  //! A projector to transform laser data into point cloud data
  laser_geometry::LaserProjection projector;

  //! The common data topic
  std::string topicData;
  //! Define the data which is received via topicData (0: Laser, 1: Pointcloud, 2: Pointcloud2)
  int topicDataIsPointCloud;
  //! Cutoff distance
  float maxDistanceInsertion_m;

  // MAPSERVER STORAGE STRUCTURE
  //! Common buffer
  cv::Mat storageMapBuffer;
  //! Number of layers
  int maxLayer;
  //! Double buffer for heights
  std::shared_ptr<std::vector<cv::Mat>> currentMapHeight_mm, lastMapHeight_mm;
  //! Double buffer for inensities
  std::shared_ptr<std::vector<cv::Mat>> currentPulsewidth_ps, lastPulsewidth_ps;
  //! Double buffer for the iterator
  std::shared_ptr<std::vector<cv::Mat>> currentMapIterator, lastMapIterator;
  //! Pointer to the current position in the mapHeight map
  cv::Mat mapIterator_1, mapIterator_2;

  // DEBUG STUFF
  //! Shows the RPCs as OpenCV images if true
  int debugDrawRpc = 0;
  //! Send the top layer of the mapstack if true
  int sendTopLayerOfDistanceAsOgm;
  //! Topic over which the top layer of the mapstack is published
  std::string topicMap;
  //! Publisher over which the top layer of the mapstack is published
  ros::Publisher publisherMap;
  // Debug stuff for service RPC
  cv::Mat mapStackStatisticRequestDebug, mapStackStatisticDebug;
  cv::RotatedRect rect;

  //! Services
  ros::ServiceServer service_meanHeight;
  ros::ServiceServer service_varianceHeight;
  ros::ServiceServer service_quantil90Height;
  ros::ServiceServer service_quantil95Height;
  ros::ServiceServer service_meanPulsewidth;
  ros::ServiceServer service_variancePulsewidth;
  ros::ServiceServer service_quantil90Pulsewidth;
  ros::ServiceServer service_quantil95Pulsewidth;

  ///
  /// \brief Translate a map
  /// \param src Input image
  /// \param dst Result image
  /// \param offsetx Offset in x direction in pixel
  /// \param offsety Offset in y direction in pixel
  /// \param fillValue Value to fill up the boarders
  ///
  template<typename T>
  void translateMap(cv::Mat &src, cv::Mat &dst, double offsetx = 0,
                    double offsety = 0, T fillValue =
                        numerics::invalidValue_int16);

  ///
  /// \brief Refresh or shift the map layers in the stack, plus the possibility to store the map stack as images on the hard drive
  /// \param mapStack The stack
  /// \param mapStackShiftedResult The result of the shifted map
  /// \param transformRoiInWorld The transformation refers the center of the ROI in the world, so that the information is stored in the map name
  /// \param prefixString A prefix for the filename
  /// \param formatString The string describing the the encoding format of the stack content (e.g. 8UC1: unsigned char (See OpenCV format for this string: http://docs.opencv.org/java/2.4.7/org/opencv/core/CvType.html)). If empty, OpenCV type is taken.
  /// \param formatUnitString Unit of the data content (eg. mm for millimeter)
  /// \param resolution_meterPerTile Size of quadratic tile in meter
  /// \param storeMapStack Indicator for storing the map stack to the hard drive (>0: store maps)
  /// \param shiftMapStack Indicator for shifting the map stack
  /// \param clearMapStack Set the whole map stack to the default value
  /// \param fillValue Value to fill up a cleared or shifted map
  ///
  template<typename T>
  void mapRefreshAndStorage(
      std::shared_ptr<std::vector<cv::Mat>> mapStack,
      std::shared_ptr<std::vector<cv::Mat>> mapStackShiftedResult,
      const tf::StampedTransform transformRoiInWorld,
      const std::string prefixString, const std::string formatString,
      const std::string formatUnitString, const double resolution_meterPerTile,
      const bool storeMapStack, const bool shiftMapStack,
      const bool clearMapStack, T fillValue = numerics::invalidValue_int16,
      bool storeCurrentPosition = true /*unused*/,
      std::string additionalInformationString = std::string(""),
      ros::Time storageTime = ros::Time::now());

  ///
  /// \brief Wrapper for mapRefreshAndStorage when it comes to a new tile
  /// \param transformRoiInWorld Transform from the current ROI (mapStack should be already deprecated) to the world frame
  /// \param storeCurrentPosition Stores the current ROI position to relate to it for the next storage (necessary for shifting)
  /// \param additionalInformationString Some arbitrary string which can be added to the file name
  /// \param storageTime The timestamp which is used in the filenames
  ///
  virtual void mapRefreshAndStorageOnTileChange(
      const tf::StampedTransform &transformRoiInWorld,
      const bool storeCurrentPosition = true,
      const std::string &additionalInformationString = std::string(""),
      const ros::Time &storageTime = ros::Time::now());

  ///
  /// \brief Transforms the received laser messages into the map frame and stores them in the next free map layer
  /// \param scanMsg Laser message to process
  ///
  void dataHandler(const sensor_msgs::PointCloud2::Ptr scanMsg);

  ///
  /// \brief Transforms the received laser messages into the map frame and stores them in the next free map layer
  /// \param scanMsg Laser message to process
  ///
  void dataHandler(const sensor_msgs::PointCloud::Ptr scanMsg);

  ///
  /// \brief Transforms the received point cloud messages into the map frame and stores them in the next free map layer
  /// \param scanMsg Point cloud message to process
  ///
  void dataHandler(const sensor_msgs::LaserScan::ConstPtr scanMsg);

  ///////////////////////////////////////////SERVER///////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  enum statistics {
    mean,
    variance,
    quantil,
    quantil90,
    quantil95
  };

  ///
  /// \brief Calculates the requested statistic measure out of the map layer stack
  /// \param src Map layer stack
  /// \param dst Processed statistic as a single layer
  /// \param statisticReq The statistics which are stored into the return value
  /// \param quantil The quantil measure from 1 to 99 which is processed if quantil is requested
  ///
  void getStatistic(const std::vector<cv::Mat> &src, cv::Mat &dst,
                    const statistics statisticReq,
                    const uint8_t quantil = 0/*1 .. 99 %*/);

//  ///
//  /// \brief Cuts the requested view out of the map stack and processes the statistics (Return the cropped statistic view of a std::vector<cv::Mat_<int16_t>> mapStack as cv::Mat_<float>)
//  /// \param mapStack The map layer stack
//  /// \param xView Horizontal translation in meter
//  /// \param yView Lateral translation in meter
//  /// \param wView Width of the requested view in meter (expansion along xView)
//  /// \param dView Depth of the requested view in meter (expansion along yView)
//  /// \param zRotView Rotation around the vertical axis in rad
//  /// \param stat The statistics to be calculated out of the map layer stack
//  /// \param targetFrame Target frame of the request
//  /// \param sourceFrame Source frame of the request
//  /// \param stamp Time at which the transform from source to target frame should be done
//  /// \return The requested and calculated view as a rectangle
//  ///
//  cv::Mat getStatisticView(const std::shared_ptr<std::vector<cv::Mat>> mapStack,
//                           const double xView/*x*/, const double yView/*y*/,
//                           const double wView/*w*/, const double dView/*h*/,
//                           const double zRotView /*rotZ*/,
//                           const statistics stat, const std::string targetFrame,
//                           const std::string sourceFrame,
//                           const ros::Time stamp = ros::Time(0.0));


  ///
  /// \brief Cuts the requested view out of the map stack and processes the statistics (Return the cropped statistic view of a std::vector<cv::Mat_<int16_t>> mapStack as cv::Mat_<float>)
  /// \param mapStack The map layer stack
  /// \param stat The statistics to be calculated out of the map layer stack
  /// \param targetFrame_id Target frame of the request
  /// \param view The request
  /// \return The requested and calculated view as a rectangle
  ///
  cv::Mat getStatisticView(
        const std::shared_ptr<std::vector<cv::Mat>> mapStack,
        const statistics stat, const std::string targetFrame_id,
        const mapserver_msgs::mapPrimitive &view);

  ///
  /// \brief Wrapper function for getStatisticView() which resizes the answer to the requested resolution
  /// \param view Request
  /// \param output Answer
  /// \param input Map layer stack from where the answer should be processed
  /// \param stat The statistic measure to calculate out of the map layer stack
  /// \param outputDimension An informative string describing the units stored in the answer
  ///
  void getStatisticOutput(const mapserver_msgs::mapPrimitive &view,
                          mapserver_msgs::rsm &output,
                          const std::shared_ptr<std::vector<cv::Mat>> &input,
                          const statistics stat,
                          const std::string &outputDimension);

  bool mapRawServerMeanHeight(mapserver::rsm::Request &req,
                              mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-meanHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::mean, std::string("mm"));
    ROS_INFO("rawMapserver-meanHeight method called: Finish");
    return true;
  }

  bool mapRawServerVarianceHeight(mapserver::rsm::Request &req,
                                  mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::variance, std::string("mm^2"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil90Height(mapserver::rsm::Request &req,
                                   mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::quantil90, std::string("mm"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil95Height(mapserver::rsm::Request &req,
                                   mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::quantil95, std::string("mm"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerMeanPulsewidth(mapserver::rsm::Request &req,
                                  mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::mean, std::string("ps"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerVariancePulsewidth(mapserver::rsm::Request &req,
                                      mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::variance, std::string("ps^2"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil90Pulsewidth(mapserver::rsm::Request &req,
                                       mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::quantil90, std::string("ps"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil95Pulsewidth(mapserver::rsm::Request &req,
                                       mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::quantil95, std::string("ps"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  ///
  /// \brief Sends the top layer of the mapstack
  ///
  void sendMap(void) {
    if (sendTopLayerOfDistanceAsOgm) {
      // Send the top layer as occupancy grid map
      mtxSwap.lock();
      std::string frameTarget(currentTileTfName);
      std::shared_ptr<std::vector<cv::Mat>> mapHeight_mm = currentMapHeight_mm;
      mtxSwap.unlock();
      nav_msgs::OccupancyGrid ogm;
      ogm.header.frame_id = frameTarget + std::string("_")
          + machine::frames::names::ROI_ORIGIN;
      ogm.info.height = mapSizeY;
      ogm.info.width = mapSizeX;
      ogm.info.resolution = resolution_mPerTile;
      ogm.data.resize(mapSizeY * mapSizeX, 0);
      // HACK Needs parametrization for display: look at 1e-1
      for (int idx = 0; idx < ogm.data.size(); ++idx) {
        ogm.data.at(idx) = int8_t(
            float(mapHeight_mm->at(0).at<int16_t>(idx)) * 0.1f);
        //       std::cerr << ogm.data.at(idx) << " ";
        //       std::cerr << currentMapHeight_mm->at(0).at<int16_t>(idx) << " ";
      }
      //     std::cerr << std::endl;
      publisherMap.publish(ogm);
    }
  }

  ///
  /// \brief Swaps all necessary stacks
  ///
  virtual void swapStack() {
    std::swap(this->currentMapHeight_mm, this->lastMapHeight_mm);
    std::swap(this->currentPulsewidth_ps, this->lastPulsewidth_ps);
    std::swap(this->currentMapIterator, this->lastMapIterator);
  }

  ///
  /// \brief Evaluates condition for storing refreshing and storing the mapstack
  /// \return True if all mapstacks are ready to be stored and swapped
  ///
  virtual bool mapRefreshAndStorageCondition() {
    // The last map is hold by another process
    return ((lastMapHeight_mm.unique() && lastPulsewidth_ps.unique() && lastMapIterator.unique()));
  }

  ///
  /// \brief Shows the current RPC if debug is on
  ///
  void showRpc() {
    if (debug) {
      ROS_INFO("DEBUG: Show the requests");
      if (mapStackStatisticDebug.empty() || mapStackStatisticRequestDebug.empty()) {
        ROS_ERROR("mapStackStatistic is empty");
        return;
      }
      cv::Mat mapStackStatisticRequestDebugTmp, mapStackStatisticDebugTmp;
      mtxShowRpc.lock();
      mapStackStatisticRequestDebug.copyTo(mapStackStatisticRequestDebugTmp);
      mapStackStatisticDebug.copyTo(mapStackStatisticDebugTmp);
      mapStackStatisticDebug.release();
      mapStackStatisticRequestDebug.release();
      cv::RotatedRect rectTmp = rect;
      mtxShowRpc.unlock();

      std::stringstream os;
      os << ros::Time::now();
      {
        cv::imshow(std::string("mapStackStatisticRequest"),
                   mapStackStatisticRequestDebugTmp);
        cv::setWindowTitle(
            std::string("mapStackStatisticRequest"),
            std::string("mapStackStatisticRequest at ") + os.str());
      }
      {
        // Draw the request in pseudo-colors
        if (debugDrawRpc) {
          // TODO Make a cast which is possible to handle all data types
          utils::castCopyImage(mapStackStatisticDebugTmp,
                               mapStackStatisticDebugTmp, CV_16UC1);
          mapStackStatisticDebugTmp.convertTo(mapStackStatisticDebugTmp,
          CV_8UC3);
          cv::cvtColor(mapStackStatisticDebugTmp, mapStackStatisticDebugTmp,
                       CV_GRAY2BGR);
          utils::drawRotatedRectInImage(mapStackStatisticDebugTmp, rect,
                                        cv::Scalar(0, 0, 255));
        }

        cv::imshow(std::string("mapStackStatistic"), mapStackStatisticDebugTmp);
        cv::setWindowTitle(std::string("mapStackStatistic"),
                           std::string("mapStackStatistic at ") + os.str());
      }
      cv::waitKey(1);
    }
  }

  void spinOnce() {
    sendMap();
    showRpc();
  }

};

MapserverRaw::MapserverRaw(ros::NodeHandle &nh)
    : Mapserver(&nh),
      n(nh) {

  n.param<std::string>("topic_data", topicData, scopes::root);
  n.param<int>("topic_data_is_point_cloud", topicDataIsPointCloud, 0);
  n.param<float>("max_distance_insertion_m", maxDistanceInsertion_m,
                 std::min(mapping::roi::width, mapping::roi::height));
  n.param<int>("max_layer", maxLayer, 20);
  n.param<int>("send_top_layer_of_distancestack_as_ogm",
               sendTopLayerOfDistanceAsOgm, 1);
  n.param<std::string>("send_top_layer_of_distancestack_as_ogm_topic", topicMap, "/debug/map");
  n.param<int>("debug_draw_rpc_in_view", debugDrawRpc, 0);

  publisherMap = n.advertise<nav_msgs::OccupancyGrid>(topicMap, 1);

  if (this->topicDataIsPointCloud == 2) {
    ROS_INFO("Subscribe to sensor_msgs::PointCloud2 messages");
    subscriberData = n.subscribe<sensor_msgs::PointCloud2::Ptr>(
        topicData, 1, &MapserverRaw::dataHandler, this);
   } else if (this->topicDataIsPointCloud == 1) {
    ROS_INFO("Subscribe to sensor_msgs::PointCloud messages");
    subscriberData = n.subscribe<sensor_msgs::PointCloud::Ptr>(
        topicData, 1, &MapserverRaw::dataHandler, this);
  } else if (this->topicDataIsPointCloud == 0) {
    ROS_INFO("Subscribe to sensor_msgs::LaserScan messages");
    subscriberData = n.subscribe<sensor_msgs::LaserScan::ConstPtr>(
        topicData, 1, &MapserverRaw::dataHandler, this);
  } else {
    ROS_ASSERT(this->topicDataIsPointCloud < 0 || this->topicDataIsPointCloud > 2);
  }

  // Allocate space for the maps and init the double-buffer
  storageMapBuffer = cv::Mat(mapSizeX, mapSizeY, CV_8UC1);
  cv::Mat tmpTopLayerHeight = cv::Mat(mapSizeX, mapSizeY, CV_16SC1);

  {
    // Initiate the data for the maps
    cv::Mat mapInit(mapSizeX, mapSizeX, CV_16SC1,
                    cv::Scalar_<int16_t>(numerics::invalidValue_int16));
    cv::Mat iteratorInit(mapSizeX, mapSizeX, CV_8UC1, cv::Scalar_<uint8_t>(0));

    // Allocate memory for the maps
    currentMapHeight_mm = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    lastMapHeight_mm = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    currentPulsewidth_ps = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    lastPulsewidth_ps = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    currentMapIterator = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(1));
    lastMapIterator = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(1));

    // Copy the initiated date to the map stacks
    for (std::size_t idx = 0; idx < maxLayer; ++idx) {
      mapInit.copyTo(currentMapHeight_mm->at(idx));
      mapInit.copyTo(lastMapHeight_mm->at(idx));
      mapInit.copyTo(currentPulsewidth_ps->at(idx));
      mapInit.copyTo(lastPulsewidth_ps->at(idx));
      ROS_DEBUG_STREAM(
          " Pointer: " << (int* )currentMapHeight_mm->at(idx).data);
    }
    iteratorInit.copyTo(currentMapIterator->at(0));
    iteratorInit.copyTo(lastMapIterator->at(0));
  }

  // Server
  // TODO Make request and scope changeable via program options concerning DisplayRoi
  const std::string s("/");
  service_meanHeight = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::meanHeight,
      &MapserverRaw::mapRawServerMeanHeight, this);
  service_varianceHeight = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::varianceHeight,
      &MapserverRaw::mapRawServerVarianceHeight, this);
  service_quantil90Height = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil90Height,
      &MapserverRaw::mapRawServerQuantil90Height, this);
  service_quantil95Height = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil95Height,
      &MapserverRaw::mapRawServerQuantil95Height, this);
  service_meanPulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::meanPulsewidth,
      &MapserverRaw::mapRawServerMeanPulsewidth, this);
  service_variancePulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::variancePulsewidth,
      &MapserverRaw::mapRawServerVariancePulsewidth, this);
  service_quantil90Pulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil90Pulsewidth,
      &MapserverRaw::mapRawServerQuantil90Pulsewidth, this);
  service_quantil95Pulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil95Pulsewidth,
      &MapserverRaw::mapRawServerQuantil95Pulsewidth, this);

}
