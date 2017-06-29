#include "Constants.hpp"

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

// ROS
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <mapserver_msgs/pnsTuple.h>
#include <rosgraph_msgs/Log.h>
#include <tf_conversions/tf_eigen.h>
#include <rosapi/Topics.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpc.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <mapserver/rsm.h>
#include <mapserver/ismStackFloat.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <ros/duration.h>

// OpenCV
#include <opencv2/opencv.hpp>

template<typename TMapstack, typename TData, typename TValue, typename TChild>
class Mapserver {

 public:

  ///
  /// \brief Constructor
  /// \param nh Current node handle
  ///
  Mapserver(ros::NodeHandle *nh);

  ///
  /// \brief Destructor
  ///
  virtual ~Mapserver();

// Common variables
 protected:
  //! The node handle
  ros::NodeHandle *n;

// Parameter
 protected:
  //! Holding the current tile name
  std::string currentTileTfName;
  //! Holding the former tile name
  std::string lastTileTfName;
  //! Map size in x direction in meter (s.t. width)
  int mapSizeX_m;
  //! Map size in y direction in meter (s.t. depth)
  int mapSizeY_m;
  //! Map size in z direction in meter (s.t. height)
  int mapSizeZ_m;
  //! Discrete map size in x direction (s.t. width)
  int mapSizeX;
  //! Discrete map size in y direction (s.t. depth)
  int mapSizeY;
  //! Discrete map size in z direction (s.t. height)
  int mapSizeZ;
  //! Location to store the maps
  std::string mapStorageLocation;
  //! Indicator, if maps should be stored if currentTileTfName changes
  int dontStoreMaps = 0;
  //! Indicator, if maps should be shifted in the memory if currentTileTfName changes
  int shiftMap = 1;
  //! The topic prefix used by advertiseSubscribers to subscribe to all child topics
  std::string topicPrefix;
  //! The prefix of the frame id for the origin of the tile
  std::string tileOriginTfPrefix;
  //! The suffix of the frame id for the ROI origin of the tile
  std::string tileOriginTfSufixForRoiOrigin;
  //! Topic name which sends the current frame id
  std::string currentTfNameTopic;
  //! Topic name which sends the tuple containing current frame id, local and global position (if this is defined, currentTfNameTopic is not handled)
  std::string currentTupleTopic;
  //! Prefix for adaptive subscription (if empty, no adaptive subscription is applied)
  std::string adaptiveSubscriptionTopicPrefix;
  //! World frame id
  std::string worldLink;
  //! Topic which triggers the immediately storing of the maps, without shifting or swapping
  std::string storeMapsTopic;
  //! Topic to request the whole mapstack
  std::string reqTopicMapStack;
  //! Wait in seconds before initialization of the node (negative values to disable)
  double idleStartupTime_s;
  //! Resolution in meter per tile
  double resolution_mPerTile;
  //! Maximum insertion distance
  float maxDistanceInsertion;
  //! Maximum x dimension of the map in meter
  float maxX_m;
  //! Minimum x dimension of the map in meter
  float minX_m;
  //! Maximum y dimension of the map in meter
  float maxY_m;
  //! Minimum y dimension of the map in meter
  float minY_m;
  //! Maximum z dimension of the map in meter
  float maxZ_m;
  //! Minimum z dimension of the map in meter
  float minZ_m;
  //! Value which is treated as the initialization value for every allocated tile (e.g. Probability 0.5 for initializing the statistical map)
  TValue mapInitValue;
  //! Enable debug functionality
  int debug = 0;
  //! Run a test
  int doTest = 0;
  //! Rate to run the main loop
  double rate = 1;
  //! Number of spinners for asynchronous processing
  int numSpinner = 5;
  //! Storage name: Kind of map
  std::string storageNameMapKind;
  //! Storage name: Format (empty: Take from type specifier)
  std::string storageNameFormat;
  //! Storage name: Unit
  std::string storageNameUnit;
  //! Mutex to lock the mapstacks
  std::mutex mapRefresh;
  //! Mutex for odometry messages
  std::mutex mtxOdom;

// The common map stacks
 public:
  //! The current mapstack (Active mapstack in the double buffer)
  std::shared_ptr<std::map<std::string, TMapstack*>> currentMapStack;
  //! The last mapstack (Passive mapstack in the double buffer)
  std::shared_ptr<std::map<std::string, TMapstack*>> lastMapStack;

// ROS listener and subscribers
 public:
  //! Common TF listener
  tf::TransformListener *listenerTf;
  //! Subscriber for tile name messages
  ros::Subscriber subscriberTfTileName;
  //! Subscriber to store the maps on demand
  ros::Subscriber subscriberStoreMaps;
  //! Subscriber for pns tuple messages
  ros::Subscriber subscriberTuple;
  //! List of subscribers which is filled up by function advertiseSubscribers
  std::vector<ros::Subscriber> subIsmList;

  ///
  /// \brief Fetches the map initialization value from the parameter server
  /// \param paramName parameter name of the init. value
  /// \param mapInitValue Pointer to the value to fill
  /// \param n The node handle
  ///
  static void getMapInitValue(const std::string paramName, TValue &mapInitValue,
                              ros::NodeHandle *n);

 private:

  ///
  /// \brief Store the current tf tile name and swap the storage
  /// \param nameMsg Name of the current tile tf
  ///
  void tfTileNameHandler(const std_msgs::String nameMsg);

  ///
  /// \brief Store the current tf tile name and swap the storage
  /// \param msg tuple of position, NavSat, and name name of the current tile tf
  ///
  void tupleHandler(const mapserver_msgs::pnsTuple msg);

// Member which can be redefined by other classes to meet their demands
 public:


  ///
  /// \brief Returns the data pointer of the map
  /// \param map The map
  /// \return Pointer to the begin of the map
  ///
  virtual void* getRawData(TMapstack *map) {};

  ///
  /// \brief Add subscriber to newly emerged topic with super-topic
  /// \param T [template] The message type
  /// \param subList Subscriber list which will be expanded by subscriber if new topics emerge
  /// \param f Function name with declaration "void myFun(const myRosMsgType::ConstPtr &msg, const std::string &topic)"
  /// \param obj The object to call f on
  /// \param superTopic Prefix of the topic in the form of "/my/super/topic/"
  /// \param debug Print additional information
  ///
  void advertiseSubscribers(
      std::vector<ros::Subscriber> &subList,
      void (TChild::*f)(const boost::shared_ptr<TData const>&,
                        const std::string&),
      TChild *obj, const std::string &superTopic, const int &debug);

  ///
  /// \brief Simplified wrapper function for mapRefreshAndStorage()
  ///
  virtual void mapStorage(
      const std::shared_ptr<
          std::map<std::string, TMapstack*>> &mapStack,
      const std::string prefixString, const std::string formatString,
      const std::string formatUnitString, const double resolution_meterPerTile);

  ///
  /// \brief Store the current mapstack without shifting or swapping
  /// \param nameMsg Name of layer to store. Store all if string is empty
  ///
  virtual void storeMaps(const std_msgs::String nameMsg);

  ///
  /// \brief Swaps all necessary stacks (Needs to be redefined if )
  ///
  virtual void swapStack();

  ///
  /// \brief Evaluates condition for storing refreshing and storing the mapstack
  /// \return True if all mapstacks are ready to be stored and swapped
  ///
  virtual inline bool mapRefreshAndStorageCondition();

  ///
  /// \brief Called by spin() with specific period
  ///
  virtual void spinOnce() {};

  ///
  /// \brief Call to run the mapserver
  ///
  void spin();

  ///
  /// \brief Correct the quaternions of a pose if invalid
  /// \param pose The pose to correct
  ///
  void correctInvalidOrientation(tf::Pose &pose);

  ///
  /// \brief Shift/purge/stores the map stack to hard drive (stores binary to hard drive)
  /// \param mapStack Map to shift/store/reset
  /// \param mapStackShiftedResult The result of mapStack which is shift/store/reset
  /// \param transformRoiInWorld Transform from the current ROI (mapStack should be already deprecated) to the world frame
  /// \param prefixString Free identifier name for the map (fills the 'What' key)
  /// \param formatString Format e.g. int, uint8, double, etc. (empty: Take from type specifier)
  /// \param formatUnitString Unit of the digets
  /// \param resolution_meterPerTile Resolution in meter per tile
  /// \param storeMapStack Maps are stored to hard drive if true
  /// \param shiftMapStack Maps are shifted by transform to new ROI if true
  /// \param clearMapStack Maps are purged if true
  /// \param fillValue Value to fill up boarders or maps if they are purged
  /// \param storeCurrentPosition Stores the current ROI position to relate to it for the next storage (necessary for shifting)
  /// \param additionalInformationString Some arbitrary string which can be added to the file name
  ///
  virtual void mapRefreshAndStorage(
      const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack,
      const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStackShiftedResult,
      const tf::StampedTransform transformRoiInWorld,
      const std::string prefixString, const std::string formatString,
      const std::string formatUnitString, const double resolution_meterPerTile,
      const bool storeMapStack, const bool shiftMapStack,
      const bool clearMapStack,
      TValue fillValue,
      bool storeCurrentPosition = true,
      std::string additionalInformationString = std::string(""));

  ///
  /// \brief Translates the content of a map and fills up the boarders
  /// \param map The map
  /// \param offsetx Offset to move in X pixel direction
  /// \param offsety Offset to move in Y pixel direction
  /// \param fillValue Fill-up value
  ///
  virtual void translateMap(TMapstack &map, int offsetx, int offsety,
                            TValue fillValue) {
  }
  ;

  ///
  /// \brief Set all map tiles of all maps to the given value
  /// \param mapStack The mapstack to reset
  /// \param value Fill-up value
  ///
  virtual void fillMapStack(std::vector<TMapstack> &mapStack,
                            TValue fillValue) {
  }
  ;

  ///
  /// \brief Set all map tiles of a maps to the given value
  /// \param map The map to reset
  /// \param value Fill-up value
  ///
  virtual void fillMap(TMapstack &mapStack, TValue fillValue) {
  }
  ;

 public:

  ///
  /// \brief Sanity check for topic name, so it looks like "/some/scope" or at least "/"
  /// \param topic The topic name to refine
  ///
  static void topicRefinement(std::string &topic);

  ///
  /// \brief Translates the content and fills up the boarders
  /// \param src The source map
  /// \param dst Destination map
  /// \param offsetx Offset to move in X pixel direction
  /// \param offsety Offset to move in Y pixel direction
  /// \param fillValue Fill-up value
  ///
  void translateMap(cv::Mat &src, cv::Mat &dst, double offsetx,
                    double offsety, TValue fillValue);

  ///
  /// \brief Converts and occupancy grid map to a CV_8UC1 grayscale image in OpenCV
  /// \param map The occupancy grid map
  /// \return Shared pointer on image. Pointer is zero, if image allocation fails
  ///
  static std::shared_ptr<cv::Mat> rosOccToGrayScale(nav_msgs::OccupancyGrid::ConstPtr map);

  ///
  /// \brief Converts and occupancy grid map to a CV_8SC1 image in OpenCV
  /// \param map The occupancy grid map
  /// \return Shared pointer on image. Pointer is zero, if image allocation fails
  ///
  static std::shared_ptr<cv::Mat> rosOccToImage(nav_msgs::OccupancyGrid::ConstPtr map);

  ///
  /// \brief Converts any one channel, row major map to the corresponding OpenCV image
  /// \param map The map
  /// \param width_cells Number of columns
  /// \param height_cells Number of rows
  /// \return Shared pointer on image. Pointer is zero, if image allocation fails
  ///
  static std::shared_ptr<cv::Mat> mapToImage(TMapstack &map, int width_cells, int height_cells);

  ///
  /// \brief Transforms pose into target frame
  /// \param poseInSourceFrame The pose to transform
  /// \param targetFrame Target frame of the frame
  /// \param tfListener A listener for transformation
  /// \return Shared pointer on transformed pose
  ///
  static std::shared_ptr<tf::Stamped<tf::Pose>> getPoseInFrame(
      const tf::Stamped<tf::Pose> &poseInSourceFrame,
      const std::string &targetFrame, const tf::TransformListener &tfListener);

  ///
  /// \brief Translates a mapstack and fills up the boarders
  /// \param mapStack
  /// \param offsetx Offset to move in X pixel direction
  /// \param offsety Offset to move in Y pixel direction
  /// \param fillValue Fill-up value
  ///
  void translateMapStack(
      const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack,
      int offsetx, int offsety, TValue fillValue);

};

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::advertiseSubscribers(
    std::vector<ros::Subscriber> &subList,
    void (TChild::*f)(const boost::shared_ptr<TData const>&,
                      const std::string&),
    TChild *obj, const std::string &superTopic, const int &debug) {
//    ros::NodeHandle &n = ros::NodeHandle("~")) {

  if (superTopic.empty()) {
    ROS_ERROR_ONCE("No subscription to root topic");
    return;
  }

  ros::master::V_TopicInfo topics;  // List of topics
  bool someSubscription = false;   // Indicator, if subscription occurs
  bool subscribe = true;         // Indicator, if subscription should take place

  if (ros::master::getTopics(topics)) {
    if (debug) {  // Print topics
      ROS_DEBUG("List topics:\n");
      for (int idx = 0; idx < topics.size(); ++idx) {
        ROS_DEBUG("\n%d:\n"
                  "  TOPIC: %s\n"
                  "  TYPE : %s\n",
                  idx, topics.at(idx).name.c_str(),
                  topics.at(idx).datatype.c_str());
      }
    }
    // Get a list topics which needs to be investigated (Check with ism_scope_prefix)
    std::vector<bool> topicsForSubscription(topics.size(), false);
    if ((superTopic.size() == 1) || superTopic.empty()) {  // Check if "/" is the only content
      ROS_WARN("Using all scopes, because ism_scope_prefix is empty");
      topicsForSubscription = std::vector<bool>(topics.size(), true);
    } else {  // Set the indicator to true, if the super-topic is the same
      for (int idx = 0; idx < topics.size(); ++idx) {
        if (!topics.at(idx).name.substr(0, superTopic.size()).compare(
            superTopic)) {
          topicsForSubscription.at(idx) = true;
        }
      }
    }
    // Check if the topics are already subscribed, otherwise add a subscription
    for (int idx = 0; idx < topics.size(); ++idx) {
      if (!topicsForSubscription.at(idx)) {  // Skip if the topic is not valid
        continue;
      } else {
        someSubscription = true;
        if (subList.empty()) {  // If we haven't subscribed to anything yet, get the first
          ROS_INFO("First subscription to: %s", topics.at(idx).name.c_str());
          subList.push_back(
              n->subscribe<TData>(
                  topics.at(idx).name,
                  100,
                  std::bind(f, obj, std::placeholders::_1,
                            topics.at(idx).name)));
        } else {
          subscribe = true;
          for (int idy = 0; idy < subList.size(); ++idy) {  // Check if topic already subscribed, ...
            if (!subList.at(idy).getTopic().compare(topics.at(idx).name)) {
              subscribe = false;
              break;
            }
          }
          if (subscribe) {  // ... otherwise do the subscription
            ROS_INFO("Subscription %d to: %s", int(subList.size() + 1),
                     topics.at(idx).name.c_str());
            subList.push_back(
                n->subscribe<TData>(
                    topics.at(idx).name,
                    100,
                    std::bind(f, obj, std::placeholders::_1,
                              topics.at(idx).name)));
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

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::swapStack() {
  std::swap(this->currentMapStack, this->lastMapStack);
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
bool Mapserver<TMapstack, TData, TValue, TChild>::mapRefreshAndStorageCondition() {
  return (this->lastMapStack.unique() && this->currentMapStack.unique());
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::tfTileNameHandler(
    const std_msgs::String nameMsg) {
  bool currentTileTfNameChange = false;
  mapRefresh.lock();
  if ((nameMsg.data.back() != currentTileTfName.back())) {
    if (currentTileTfName.empty()) {
      // First round, we bootstrap
      currentTileTfName = nameMsg.data;
    } else {
      this->swapStack();
      lastTileTfName = currentTileTfName;
      currentTileTfName = nameMsg.data;
      currentTileTfNameChange = true;
    }
  }

  if (currentTileTfNameChange) {
    tf::StampedTransform transformRoiInWorld;
    try {
      listenerTf->waitForTransform(worldLink, lastTileTfName, ros::Time(0.0),
                                   ros::Duration(3.0));
      listenerTf->lookupTransform(worldLink, lastTileTfName, ros::Time(0.0),
                                  transformRoiInWorld);
    } catch (const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("tfTileNameHandler: %s", excStr.c_str());
      mapRefresh.unlock();
      return;
    }
    ROS_INFO("NEW MAP");

    // Wait until all references are gone
    std::size_t lockCnt = 0;
    const std::size_t lockCntMax = 200000;  // 2 seconds if we sleep for 10 us
    while (!this->mapRefreshAndStorageCondition()) {
      usleep(10);
      if (++lockCnt > lockCntMax) {
        ROS_ERROR(
            "tfTileNameHandler: Locked for to long, skip storage (maybe deadlock or out if resources?)");
        mapRefresh.unlock();
        return;
      }
    }

    mapRefreshAndStorage(lastMapStack,               // Map to shift/store/reset
        currentMapStack,                       // The result of the shifted map
        transformRoiInWorld,                   // Transform
        storageNameMapKind,                    // Kind of map
        storageNameFormat,           // Format (empty: Take from type specifier)
        storageNameUnit,                       // Unit
        resolution_mPerTile,                   // Resolution per tile
        !dontStoreMaps,                        // Info if maps should be stored
        bool(shiftMap),                        // Info if maps should be shifted
        !shiftMap,       // If map is not shifted, reset the content of mapStack
        mapInitValue);  // Fill-up value
  }

  mapRefresh.unlock();

}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::tupleHandler(
    const mapserver_msgs::pnsTuple msg) {
  bool currentTileTfNameChange = false;
  static mapserver_msgs::pnsTuple lastPnsTuple;
  mapRefresh.lock();
  if ((msg.string.data.back() != currentTileTfName.back())) {
    if (currentTileTfName.empty()) {
      // First round, we bootstrap
      currentTileTfName = msg.string.data;
      lastPnsTuple = msg;
    } else {
      std::swap(currentMapStack, lastMapStack);
      lastTileTfName = currentTileTfName;
      currentTileTfName = msg.string.data;
      currentTileTfNameChange = true;
    }
  }

  if (currentTileTfNameChange) {
    ROS_INFO("NEW MAP");
    tf::StampedTransform transformRoiInWorld;
    transformRoiInWorld.setOrigin(
        tf::Vector3(msg.point.x, msg.point.y, msg.point.z));
    transformRoiInWorld.setRotation(tf::Quaternion(0, 0, 0, 1));

    // Wait until all references are gone
    std::size_t lockCnt = 0;
    const std::size_t lockCntMax = 200000;  // 2 seconds if we sleep for 10 us
    while (!(lastMapStack.unique() && currentMapStack.unique())) {
      usleep(10);
      if (++lockCnt > lockCntMax) {
        ROS_ERROR(
            "tfTileNameHandler: Locked for to long, skip storage (maybe deadlock or out if resources?)");
        mapRefresh.unlock();
        return;
      }
    }

    std::stringstream navSatSs;
    navSatSs << std::setprecision(12) << "lat_" << lastPnsTuple.navsat.latitude
             << "_" << "lon_" << lastPnsTuple.navsat.longitude << "_" << "alt_"
             << lastPnsTuple.navsat.altitude;

    mapRefreshAndStorage(lastMapStack,               // Map to shift/store/reset
        currentMapStack,                       // The result of the shifted map
        transformRoiInWorld,                   // Transform
        storageNameMapKind,                    // Kind of map
        storageNameFormat,           // Format (empty: Take from type specifier)
        storageNameUnit,                       // Unit
        resolution_mPerTile,                   // Resolution per tile
        !dontStoreMaps,                        // Info if maps should be stored
        bool(shiftMap),                        // Info if maps should be shifted
        !shiftMap,       // If map is not shifted, reset the content of mapStack
        mapInitValue,  // Fill-up value
        true, navSatSs.str());
    // Store the current tile information as next last one
    lastPnsTuple = msg;
  }

  mapRefresh.unlock();
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::mapStorage(
    const std::shared_ptr<
        std::map<std::string, TMapstack*>> &mapStack,
    const std::string prefixString, const std::string formatString,
    const std::string formatUnitString, const double resolution_meterPerTile) {

  const std::shared_ptr<std::map<std::string, TMapstack*>> dummyMap;
  const tf::StampedTransform dummyTf;
  const TValue dummy = 0.0;

  mapRefreshAndStorage(mapStack,                     // Map to shift/store/reset
      dummyMap,                                 // Nothing at all
      dummyTf,                                  // Transform
      std::string("OGM"),                       // Kind of map
      std::string(""),               // Format (empty: Take from type specifier)
      std::string("logodds"),                   // Unit
      resolution_meterPerTile,                  // Resolution per tile
      true,                                     // Maps should be stored
      false,                                    // Maps should be shifted
      false,                                    // Map should be reseted reset
      dummy,                                    // Some float value
      false);                                // Don't store the current position

}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::storeMaps(
    const std_msgs::String nameMsg) {

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

  mapStorage(currentMapStack, storageNameMapKind,                 // Kind of map
             storageNameFormat,      // Format (empty: Take from type specifier)
             storageNameUnit,                       // Unit
             resolution_mPerTile);                  // Resolution per tile
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::topicRefinement(
    std::string &topic) {
  if (topic.empty()) {
    topic = "/";
  } else {
    if (topic.at(0) != '/') {
      topic = std::string("/").append(topic);
    }
    if (topic.at(topic.size() - 1) != '/') {
      topic.append("/");
    }
  }
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
Mapserver<TMapstack, TData, TValue, TChild>::Mapserver(ros::NodeHandle *nh)
    : n(nh),
      currentTileTfName(""),
      lastTileTfName(""),
      mapSizeX(1),
      mapSizeY(1),
      mapStorageLocation(constants::mapping::ogm::mapStorageLocation),
      dontStoreMaps(0),
      shiftMap(1),
      topicPrefix("/ism"),
      tileOriginTfPrefix("map_base_link_"),
      tileOriginTfSufixForRoiOrigin(
          constants::machine::frames::names::ROI_ORIGIN),
      currentTfNameTopic("/currentTfTile"),
      currentTupleTopic(""),
      adaptiveSubscriptionTopicPrefix(""),
      worldLink("odom"),
      storeMapsTopic("/storemaps"),
      reqTopicMapStack("/reqMapStack"),
      idleStartupTime_s(-1.0),
      resolution_mPerTile(constants::mapping::discreteResolution),
      maxDistanceInsertion(
          std::min(
              constants::mapping::roi::altitude,
              std::min(constants::mapping::roi::width,
                       constants::mapping::roi::height))),
      maxX_m(constants::mapping::roi::xMax),
      minX_m(constants::mapping::roi::xMin),
      maxY_m(constants::mapping::roi::yMax),
      minY_m(constants::mapping::roi::yMin),
      maxZ_m(constants::mapping::roi::zMax),
      minZ_m(constants::mapping::roi::zMin),
      mapInitValue(TValue(0.0)),
      debug(0),
      doTest(0),
      rate(1.0),
      storageNameMapKind(""),
      storageNameFormat(""),
      storageNameUnit("") {

  n->getParam("topic_prefix", this->topicPrefix);
  n->getParam("tile_origin_tf_prefix", this->tileOriginTfPrefix);
  n->getParam("tile_origin_tf_sufix_for_roi_origin",
              this->tileOriginTfSufixForRoiOrigin);
  n->getParam("current_tf_name_topic", this->currentTfNameTopic);
  n->getParam("current_tuple_topic", this->currentTupleTopic);
  n->getParam("world_link", this->worldLink);
  n->getParam("store_maps_topic", this->storeMapsTopic);
  n->getParam("req_topic_map_stack", this->reqTopicMapStack);
  n->getParam("idle_startup_time", this->idleStartupTime_s);
  n->getParam("debug", this->debug);
  n->getParam("test", this->doTest);
  n->getParam("resolution", this->resolution_mPerTile);
  n->getParam("max_distance_insertion", this->maxDistanceInsertion);
  n->getParam("max_x_m", this->maxX_m);
  n->getParam("min_x_m", this->minX_m);
  n->getParam("max_y_m", this->maxY_m);
  n->getParam("min_y_m", this->minY_m);
  n->getParam("max_z_m", this->maxZ_m);
  n->getParam("min_z_m", this->minZ_m);
  n->getParam("num_spinner", this->numSpinner);
  Mapserver::getMapInitValue(std::string("mapInit_value"), this->mapInitValue,
                             n);
  n->getParam("map_storage_location", this->mapStorageLocation);
  n->getParam("shift_map", this->shiftMap);
  n->getParam("dont_store_maps", this->dontStoreMaps);
  n->getParam("rate", this->rate);
  n->getParam("storage_map_name", this->storageNameMapKind);
  n->getParam("storage_format_name", this->storageNameFormat);
  n->getParam("storage_unit_name", this->storageNameUnit);

  listenerTf = new tf::TransformListener;
  Mapserver::topicRefinement(topicPrefix);

  mapSizeX_m = (this->maxX_m - this->minX_m);
  mapSizeY_m = (this->maxY_m - this->minY_m);
  mapSizeZ_m = (this->maxZ_m - this->minZ_m);
  mapSizeX   = (this->maxX_m - this->minX_m) / this->resolution_mPerTile;
  mapSizeY   = (this->maxY_m - this->minY_m) / this->resolution_mPerTile;
  mapSizeZ   = (this->maxZ_m - this->minZ_m) / this->resolution_mPerTile;

  // Check whether we should get our tile information via a tuple or just via a name
  if (currentTupleTopic.empty()) {
//      subscriberTfTileName = Mapserver::n.subscribe<std_msgs::String>(currentTfNameTopic, 2, &Mapserver::tfTileNameHandler, this);
    this->subscriberTfTileName = n->subscribe(currentTfNameTopic, 2,
                                              &Mapserver::tfTileNameHandler,
                                              this);
  } else {
//      subscriberTuple = Mapserver::n.subscribe<mapserver_msgs::pnsTuple>(currentTupleTopic, 2, &Mapserver::tupleHandler, this);
    this->subscriberTuple = n->subscribe(currentTupleTopic, 2,
                                         &Mapserver::tupleHandler, this);
  }

  // Command scope to store the maps on demand
//  subscriberStoreMaps = Mapserver::n.subscribe<std_msgs::String>(storeMapsTopic, 1, &Mapserver::storeMaps, this);
  this->subscriberStoreMaps = n->subscribe(this->storeMapsTopic, 1,
                                           &Mapserver::storeMaps, this);

}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::mapRefreshAndStorage(
    const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack,
    const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStackShiftedResult,
    const tf::StampedTransform transformRoiInWorld,
    const std::string prefixString, const std::string formatString,
    const std::string formatUnitString, const double resolution_meterPerTile,
    const bool storeMapStack, const bool shiftMapStack,
    const bool clearMapStack, TValue fillValue, bool storeCurrentPosition,
    std::string additionalInformationString) {

  // The message from the last time the function was called (So it is the location of the center)
  static tf::StampedTransform transformRoiInWorldLast;

//  static bool runOnce = true;
//  if (runOnce) {
//      ROS_WARN("First run, map storage coordniates might be corrupted");
//      transformRoiInWorldLast = transformRoiInWorld;
//      runOnce = false;
//  }

  ROS_INFO_STREAM("storeMapStack: " << storeMapStack);
  ROS_INFO_STREAM("mapStack->size(): " << mapStack->size());
  if (storeMapStack) {
    // Get the timestamp in microseconds
//    ros::Time stamp = ros::Time::now();
//    uint64_t timestamp = uint64_t(stamp.sec) * uint64_t(1e6) + uint64_t(stamp.nsec / 1e6);
    // Store each layer in the map
    for (auto it = mapStack->begin(); it != mapStack->end(); ++it) {
      std::cout << it->first << " => " << it->second << '\n';

      // Get the format string
      const std::string format =
          formatString.empty() ?
              std::string("INT")
                  + std::to_string(int(sizeof(TValue)) * int(8)) :
              formatString;
      // Replace the topic name "/my/topic" to "-my-topic"
      std::string mapIdx = it->first;
      std::replace(mapIdx.begin(), mapIdx.end(), '/', '-');

      // Get the filename
      std::ostringstream oss;
      oss << std::setprecision(2) << mapStorageLocation << "What_"
          << prefixString << "_" << "T_" << ros::Time::now() << "s.ns_"
          << "Format_" << format << "_" << "Unit_" << formatUnitString << "_"
          << "Layer_" << mapIdx << "_" << "Res_"
          << int(
              round(
                  resolution_meterPerTile
                      * double(constants::geometry::millimeterPerMeter))) << "mm_" << "X_"
          << transformRoiInWorldLast.getOrigin().x() << "m_" << "Y_"
          << transformRoiInWorldLast.getOrigin().y() << "m_" << "Z_"
          << transformRoiInWorldLast.getOrigin().z() << "m_" << "rows_"
          << mapSizeY << "_" << "cols_" << mapSizeX
          << "_" << additionalInformationString << "_" << ".bin";

      ROS_INFO("Store map to: %s\nWith format: %s\n", oss.str().c_str(),
               format.c_str());

      // Store the layer
      std::ofstream f;
      f.open(oss.str(), std::ofstream::out | std::ofstream::binary);
      if (f.is_open()) {
        f.write(
            (char*) getRawData(it->second),
            mapSizeX * mapSizeY
                * int(sizeof(TValue)));
        f.close();
      } else {
        ROS_ERROR("Unable to open file %s\n", oss.str().c_str());
      }
    }
  }

  // Shift the map
  if (shiftMapStack) {
    // Calculate the shift as indices
    const double xdiff_m = transformRoiInWorldLast.getOrigin().x()
        - transformRoiInWorld.getOrigin().x();
    const double ydiff_m = transformRoiInWorldLast.getOrigin().y()
        - transformRoiInWorld.getOrigin().y();
    const double zdiff_m = transformRoiInWorldLast.getOrigin().y()
        - transformRoiInWorld.getOrigin().y();

    ROS_INFO("Current (x,y,z) in m: %f, %f, %f",
             transformRoiInWorld.getOrigin().x(),
             transformRoiInWorld.getOrigin().y(),
             transformRoiInWorld.getOrigin().z());
    ROS_INFO("Last    (x,y,z) in m: %f, %f, %f",
             transformRoiInWorldLast.getOrigin().x(),
             transformRoiInWorldLast.getOrigin().y(),
             transformRoiInWorldLast.getOrigin().z());
    ROS_INFO("Diff    (x,y,z) in m: %f, %f, %f", xdiff_m, ydiff_m, zdiff_m);

    const int xshift_tiles = int(std::round(xdiff_m / resolution_meterPerTile));
    const int yshift_tiles = int(std::round(ydiff_m / resolution_meterPerTile));

//    ROS_INFO_STREAM( "\nydiff_m: " << ydiff_m <<
//                     "\nresolution_meterPerTile: " << resolution_meterPerTile <<
//                     "\nydiff_m / resolution_meterPerTile: " << ydiff_m / resolution_meterPerTile <<
//                     "\n-ydiff_m / resolution_meterPerTile: " << ydiff_m / resolution_meterPerTile <<
//                     "\n-ydiff_m / resolution_meterPerTile: " << ydiff_m / resolution_meterPerTile);

    ROS_INFO(
        "Shift of the map (res: %0.2f m/tile): x=%d tiles s.t. %f m , y=%d tiles s.t. %f m",
        resolution_meterPerTile, xshift_tiles, -xdiff_m, yshift_tiles,
        -ydiff_m);

    auto itDst = mapStackShiftedResult->begin();
    for (auto itSrc = mapStack->begin(); itSrc != mapStack->end();
        ++itSrc, ++itDst) {
#if defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS)
      const int opencvType = CV_8SC1;
#else // defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
      const int opencvType = CV_16SC1;
#endif
      cv::Mat src(mapSizeY, mapSizeX,
                  opencvType, (void*) (getRawData(itSrc->second)));
      cv::Mat dst(mapSizeY, mapSizeX,
                  opencvType, (void*) (getRawData(itDst->second)));
      translateMap(src, dst, xshift_tiles, yshift_tiles, fillValue);
    }
  }

  // Clear the map
  if (clearMapStack) {
    ROS_ERROR("Clear the map");
    for (auto it = mapStack->begin(); it != mapStack->end(); ++it) {
      fillMap(*(it->second), fillValue);
    }
  }

  if (storeCurrentPosition) {
    transformRoiInWorldLast = transformRoiInWorld;  // Store the new location for the next time
  }

}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
Mapserver<TMapstack, TData, TValue, TChild>::~Mapserver() {
  delete listenerTf;
}

// Translate a map
template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::translateMap(
    cv::Mat &src, cv::Mat &dst, double offsetx, double offsety,
    TValue fillValue) {

  // Define a transformation for the image
  const cv::Mat trans_mat =
      (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);

  // Warp the image (which refers to the map)
  cv::warpAffine(src, dst, trans_mat, cv::Size(src.rows, src.cols),
                 cv::INTER_NEAREST, cv::BORDER_CONSTANT,
                 cv::Scalar(static_cast<double>(fillValue)));

}

// Translate a stack of maps
template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::translateMapStack(
    const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack,
    int offsetx, int offsety, TValue fillValue) {
  for (auto it = mapStack->begin(); it != mapStack->end(); ++it) {
    translateMap(*it->second, offsetx, offsety, fillValue);
  }
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::getMapInitValue(
    const std::string paramName, TValue &mapInitValue, ros::NodeHandle *n) {
  if (std::is_same<TValue, bool>::value) {
    bool mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    mapInitValue = TValue(mapInit_value);
  } else if (std::is_same<TValue, double>::value) {
    double mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    mapInitValue = TValue(mapInit_value);
  } else if (std::is_same<TValue, float>::value) {
    float mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    mapInitValue = TValue(mapInit_value);
  } else if (std::is_same<TValue, int>::value) {
    int mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    mapInitValue = TValue(mapInit_value);
  } else if (std::is_same<TValue, short>::value ||
      std::is_same<TValue, int16_t>::value) {
    int mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    ROS_ASSERT(
        mapInit_value < std::numeric_limits<short>::lowest()
            || mapInit_value > std::numeric_limits<short>::max());
    mapInitValue = TValue(mapInit_value);
  } else if (std::is_same<TValue, char>::value ||
      std::is_same<TValue, int8_t>::value) {
    int mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    ROS_ASSERT(
        mapInit_value < std::numeric_limits<char>::lowest()
            || mapInit_value > std::numeric_limits<char>::max());
    mapInitValue = TValue(mapInit_value);
  } else {
    ROS_ERROR("No known conversion for TValue in mapserver");
    ROS_BREAK();
  }
}


template<typename TMapstack, typename TData, typename TValue, typename TChild>
std::shared_ptr<cv::Mat> Mapserver<TMapstack, TData, TValue, TChild>::rosOccToGrayScale(
    nav_msgs::OccupancyGrid::ConstPtr map) {

  std::shared_ptr<cv::Mat> dst;

  if (map) {
    if (map->info.width > 0 && map->info.height > 0) {
      dst = std::shared_ptr<cv::Mat>(
          new cv::Mat(map->info.height, map->info.width, CV_8UC1));
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
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
std::shared_ptr<cv::Mat> Mapserver<TMapstack, TData, TValue, TChild>::rosOccToImage(
    nav_msgs::OccupancyGrid::ConstPtr map) {

  std::shared_ptr<cv::Mat> dst;

  if (map) {
    if (map->info.width > 0 && map->info.height > 0) {
      dst = std::shared_ptr<cv::Mat>(
          new cv::Mat(map->info.height, map->info.width, CV_8SC1, (void*) map->data.data()));
    }
  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
std::shared_ptr<cv::Mat> Mapserver<TMapstack, TData, TValue, TChild>::mapToImage(
    TMapstack &map, int width_cells, int height_cells) {

  std::shared_ptr<cv::Mat> dst;
  int type;

  if (std::is_same<TValue, bool>::value) {
    type = CV_8U;
  } else if (std::is_same<TValue, float>::value) {
    type = CV_32F;
  } else if (std::is_same<TValue, double>::value) {
    type = CV_64F;
  } else if (std::is_same<TValue, int>::value ||
      std::is_same<TValue, int32_t>::value) {
    type = CV_32S;
  } else if (std::is_same<TValue, short>::value ||
      std::is_same<TValue, int16_t>::value) {
    type = CV_16S;
  } else if (std::is_same<TValue, char>::value ||
      std::is_same<TValue, int8_t>::value) {
    type = CV_8S;
  } else if (std::is_same<TValue, uint>::value ||
      std::is_same<TValue, uint32_t>::value) {
    type = CV_32S; // Should be CV_32U
    ROS_WARN_ONCE("Unknown conversion CV_32U");
  } else if (std::is_same<TValue, ushort>::value ||
      std::is_same<TValue, uint16_t>::value) {
    type = CV_16U;
  } else if (std::is_same<TValue, uchar>::value ||
      std::is_same<TValue, uint8_t>::value) {
    type = CV_8U;
  } else {
    ROS_ERROR("No known conversion for TValue in mapserver");
    return dst;
  }

  dst = std::shared_ptr<cv::Mat>(
      new cv::Mat(height_cells, width_cells, type),getRawData(map));

  return dst;
}


template<typename TMapstack, typename TData, typename TValue, typename TChild>
std::shared_ptr<tf::Stamped<tf::Pose>> Mapserver<TMapstack, TData, TValue, TChild>::getPoseInFrame(
    const tf::Stamped<tf::Pose> &poseInSourceFrame,
    const std::string &targetFrame, const tf::TransformListener &tfListener) {

  std::shared_ptr<tf::Stamped<tf::Pose>> poseInTargetFrameReturn;
  tf::Stamped<tf::Pose> poseInTargetFrame;
  // Use source frame as target frame if empty
  const bool useSrcFrameAsDstFrame =
      targetFrame.empty() || !targetFrame.compare(poseInSourceFrame.frame_id_) ?
          true : false;
  const std::string srcFrame = poseInSourceFrame.frame_id_;
  const std::string dstFrame = useSrcFrameAsDstFrame ? srcFrame : targetFrame;
  // Get the origin of the pose in the target frame
  bool tfSuccess = true;
  if (!useSrcFrameAsDstFrame) {  // We don't need this, if we stay in the same frame
    try {
      // HACK We don't wait for tf messages in the past, because they will never arrive at all
      std::string errorMsg;
      const std::string errorMsgWorthitToWaitFor(
          "Lookup would require extrapolation into the future");
      tfListener.canTransform(dstFrame, srcFrame, poseInSourceFrame.stamp_,
                              &errorMsg);
      std::size_t found = errorMsg.find(errorMsgWorthitToWaitFor);
      if (found != std::string::npos || errorMsg.empty()) {
        if (found != std::string::npos) {
          ROS_DEBUG_STREAM(
              "getPoseInFrame: We'll wait for tf transform messages in the future: " << errorMsg);
          tfListener.waitForTransform(dstFrame, srcFrame,
                                      poseInSourceFrame.stamp_,
                                      ros::Duration(3.0));
        }
        tfListener.transformPose(dstFrame, poseInSourceFrame,
                                 poseInTargetFrame);
      } else {
        throw std::runtime_error(errorMsg);
      }
    } catch (const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("getPoseInFrame (%s -> %s): %s", srcFrame.c_str(),
                dstFrame.c_str(), excStr.c_str());
      tfSuccess = false;
    }
  } else {
    poseInTargetFrame = poseInSourceFrame;
  }

  // Return a filled shared pointer if tf was successful
  if (tfSuccess) {
    poseInTargetFrameReturn = std::make_shared<tf::Stamped<tf::Pose>>(
        poseInTargetFrame);
  }
  return poseInTargetFrameReturn;
}


template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::spin() {
  ros::AsyncSpinner spinner(this->numSpinner);
  spinner.start();
  // Do stuff periodically
  ros::Rate _rate(this->rate);
  ROS_INFO("Mapserver starts spinning");
  while (ros::ok()) {
    this->spinOnce();
    _rate.sleep();
  }
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::correctInvalidOrientation(tf::Pose &pose) {
  if (pose.getRotation().x() < tf::QUATERNION_TOLERANCE
      && pose.getRotation().y() < tf::QUATERNION_TOLERANCE
      && pose.getRotation().z() < tf::QUATERNION_TOLERANCE
      && pose.getRotation().w() < tf::QUATERNION_TOLERANCE) {
    ROS_WARN_ONCE(
        "correctInvalidOrientation: Pose with quaternion(0,0,0,0) detected. Interpretation as (0,0,0,1)");
    pose.setRotation(tf::Quaternion(0, 0, 0, 1));
  }
}


