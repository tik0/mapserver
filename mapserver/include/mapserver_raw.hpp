#include "mapserver.hpp"
#include <boost/math/special_functions/erf.hpp>
#include <Constants.hpp>
#include <nav_msgs/GridCells.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <utils.h>

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

  ros::Publisher publisherMap;
  ros::Subscriber subscriberData, subscriberTfTileName;
  laser_geometry::LaserProjection projector;

  // Variables
  std::string topicMap;
  std::string topicData;
  int topicDataIsPointCloud;
  float maxDistanceInsertion_m;
  int debugDrawRpc = 0;
  int sendTopLayerOfDistanceAsOgm;
  cv::Mat storageMapBuffer;
  int maxLayer;
  std::shared_ptr<std::vector<cv::Mat>> currentMapHeight_mm, lastMapHeight_mm;
  std::shared_ptr<std::vector<cv::Mat>> currentPulsewidth_ps, lastPulsewidth_ps;
  std::shared_ptr<std::vector<cv::Mat>> currentMapIterator, lastMapIterator;
  cv::Mat mapIterator_1, mapIterator_2;  // Pointer to the current position in the mapHeight map

  // Services
  ros::ServiceServer service_meanHeight;
  ros::ServiceServer service_varianceHeight;
  ros::ServiceServer service_quantil90Height;
  ros::ServiceServer service_quantil95Height;
  ros::ServiceServer service_meanPulsewidth;
  ros::ServiceServer service_variancePulsewidth;
  ros::ServiceServer service_quantil90Pulsewidth;
  ros::ServiceServer service_quantil95Pulsewidth;

  // Debug stuff for service RPC
  cv::Mat mapStackStatisticRequestDebug, mapStackStatisticDebug;
  cv::RotatedRect rect;

  // Translate a map
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
      const bool clearMapStack, T fillValue = numerics::invalidValue_int16);

  ///
  /// \brief Transforms the received laser messages into the map frame and stores them in the next free map layer
  /// \param scanMsg Laser message to process
  ///
  void dataHandler(const sensor_msgs::PointCloud::Ptr scanMsg);

  ///
  /// \brief Transforms the received point cloud messages into the map frame and stores them in the next free map layer
  /// \param scanMsg Point cloud message to process
  ///
  void laserDataHandler(const sensor_msgs::LaserScan::ConstPtr scanMsg);

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

  ///
  /// \brief Cuts the requested view out of the map stack and processes the statistics (Return the cropped statistic view of a std::vector<cv::Mat_<int16_t>> mapStack as cv::Mat_<float>)
  /// \param mapStack The map layer stack
  /// \param xView Horizontal translation in meter
  /// \param yView Lateral translation in meter
  /// \param wView Width of the requested view in meter (expansion along xView)
  /// \param dView Depth of the requested view in meter (expansion along yView)
  /// \param zRotView Rotation around the vertical axis in rad
  /// \param stat The statistics to be calculated out of the map layer stack
  /// \param targetFrame Target frame of the request
  /// \param sourceFrame Source frame of the request
  /// \return The requested and calculated view as a rectangle
  ///
  cv::Mat getStatisticView(const std::shared_ptr<std::vector<cv::Mat>> mapStack,
                           const double xView/*x*/, const double yView/*y*/,
                           const double wView/*w*/, const double dView/*h*/,
                           const double zRotView /*rotZ*/,
                           const statistics stat, const std::string targetFrame,
                           const std::string sourceFrame);

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

  void sendMap(void) {
    if (sendTopLayerOfDistanceAsOgm) {
      // Send the top layer as occupancy grid map
      mtxSwap.lock();
      const std::string frameTarget(currentTileTfName);
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
        std::swap(currentMapHeight_mm, lastMapHeight_mm);
        std::swap(currentPulsewidth_ps, lastPulsewidth_ps);
        std::swap(currentMapIterator, lastMapIterator);
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

      const bool clearMap = !shiftMap;
      while ((!(lastMapHeight_mm.unique() && lastPulsewidth_ps.unique()
          && lastMapIterator.unique()) && !clearMap)  // The last map is hold by another process, but we don't care if we don't clear it
          || !(currentMapHeight_mm.unique() && currentPulsewidth_ps.unique()
              && currentMapIterator.unique()) && !bool(shiftMap)) {  // The current map needs to be filled with the old values if shifted, but we don't care if we don't shift it
        usleep(10);
        if (++lockCnt > lockCntMax) {
          ROS_ERROR(
              "tfTileNameHandler: Locked for to long, skip storage (maybe deadlock or out if resources?)");
          mtxSwap.unlock();
          return;
        }
      }

      mapRefreshAndStorage(lastMapHeight_mm, currentMapHeight_mm,
                           transformRoiInWorld, std::string("height"),
                           std::string(""), std::string("mm"),
                           resolution_mPerTile, !dontStoreMaps, bool(shiftMap),
                           clearMap, numerics::invalidValue_int16);
      mapRefreshAndStorage(lastPulsewidth_ps,        // Map to shift/store/reset
          currentPulsewidth_ps,                 // The result of the shifted map
          transformRoiInWorld,                   // Transform
          std::string("pulsewidth"),             // Kind of map
          std::string(""),           // Format (empty: Take from type specifier)
          std::string("ps"),                     // Unit
          resolution_mPerTile,               // Tile resolution
          !dontStoreMaps,                       // Info if maps should be stored
          bool(shiftMap),                      // Info if maps should be shifted
          clearMap,      // If map is not shifted, reset the content of mapStack
          numerics::invalidValue_int16);     // Fill-up value
      mapRefreshAndStorage(lastMapIterator,          // Map to shift/store/reset
          currentMapIterator,                   // The result of the shifted map
          transformRoiInWorld,                   // Transform
          std::string("mapiterator"),            // Kind of map
          std::string(""),           // Format (empty: Take from type specifier)
          std::string("1"),                      // Unit
          resolution_mPerTile,               // Tile resolution
          false,                                // Info if maps should be stored
          bool(shiftMap),                      // Info if maps should be shifted
          clearMap,      // If map is not shifted, reset the content of mapStack
          0);                                    // Fill-up value
    }

    mtxSwap.unlock();

  }

  ///
  /// \brief Shows the current RPC if debug is on
  ///
  void showRpc() {
    if (debug) {
      ROS_INFO("DEBUG: Show the requests");
      if (mapStackStatisticDebug.empty() || mapStackStatisticDebug.empty()) {
        ROS_ERROR("mapStackStatistic is empty");
        return;
      }
      cv::Mat mapStackStatisticRequestDebugTmp, mapStackStatisticDebugTmp;
      mtxShowRpc.lock();
      mapStackStatisticRequestDebug.copyTo(mapStackStatisticRequestDebugTmp);
      mapStackStatisticDebug.copyTo(mapStackStatisticDebugTmp);
      mapStackStatisticDebugTmp.release();
      mapStackStatisticRequestDebugTmp.release();
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

  n.param<std::string>("topic_map", topicMap, "/map");
  n.param<std::string>("topic_data", topicData, scopes::root);
  n.param<int>("topic_data_is_point_cloud", topicDataIsPointCloud, 0);
  n.param<float>("max_distance_insertion_m", maxDistanceInsertion_m,
                 std::min(mapping::roi::width, mapping::roi::height));
  n.param<int>("max_layer", maxLayer, 20);
  n.param<int>("send_top_layer_of_distancestack_as_ogm",
               sendTopLayerOfDistanceAsOgm, 1);
  n.param<int>("debug_draw_rpc_in_view", debugDrawRpc, 0);

  publisherMap = n.advertise<nav_msgs::OccupancyGrid>(topicMap, 1);

  if (this->topicDataIsPointCloud) {
    ROS_INFO("Subscribe to sensor_msgs::PointCloud messages");
    subscriberData = n.subscribe<sensor_msgs::PointCloud::Ptr>(
        topicData, 100, &MapserverRaw::dataHandler, this);
  } else {
    ROS_INFO("Subscribe to sensor_msgs::LaserScan messages");
    subscriberData = n.subscribe<sensor_msgs::LaserScan::ConstPtr>(
        topicData, 100, &MapserverRaw::laserDataHandler, this);
  }

  subscriberTfTileName = n.subscribe<std_msgs::String>(
      currentTfNameTopic, 2, &MapserverRaw::tfTileNameHandler, this);

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
