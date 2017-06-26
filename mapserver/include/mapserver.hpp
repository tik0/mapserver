
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

template<typename TMapstack, typename TData, typename TValue, typename TChild>
class Mapserver {

 public:
  Mapserver(ros::NodeHandle *nh, TChild* obj);

  virtual ~Mapserver();

  ///
  /// \brief Add subscriber to newly emerged topic with super-topic
  /// \param T [template] The message type
  /// \param subList Subscriber list which will be expanded by subscriber if new topics emerge
  /// \param f Function name with declaration "void myFun(const myRosMsgType::ConstPtr &msg, const std::string &topic)"
  /// \param superTopic Prefix of the topic in the form of "/my/super/topic/"
  /// \param debug Print additional information
  /// \param n The node handle
  ///
  void advertiseSubscribers(
      std::vector<ros::Subscriber> &subList,
      void (TChild::*f)(const boost::shared_ptr< TData const>&, const std::string&),
      TChild *obj,
      const std::string &superTopic,
      const int &debug);

 protected:
  //! The node handle
  ros::NodeHandle *n;
  //! Holding the current tile name
  std::string currentTileTfName;
  //! Holding the former tile name
  std::string lastTileTfName;
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
  //! The current mapstack (Active mapstack in the double buffer)
  std::shared_ptr<std::map<std::string, TMapstack*>> currentMapStack;
  //! The last mapstack (Passive mapstack in the double buffer)
  std::shared_ptr<std::map<std::string, TMapstack*>> lastMapStack;

  //! TF listener
  tf::TransformListener *listenerTf;
  //! Subscriber for tile name messages
  ros::Subscriber subscriberTfTileName;
  //! Subscriber to store the maps on demand
  ros::Subscriber subscriberStoreMaps;
  //! Subscriber for pns tuple messages
  ros::Subscriber subscriberTuple;
  //! List of subscribers which is filled up by function advertiseSubscribers
  std::vector<ros::Subscriber> subIsmList;

 private:

  //! Derived from mapInitValue which is actually applied
  TValue mapInitValueApplied;


  ///
  /// \brief Fetches the map initialization value from the parameter server
  /// \param paramName parameter name of the init. value
  /// \param mapInitValue Pointer to the value to fill
  /// \param n The node handle
  ///
  static void getMapInitValue(const std::string paramName, TValue &mapInitValue, ros::NodeHandle *n);

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

  ///
  /// \brief Simplified wrapper function for mapRefreshAndStorage()
  ///
  void mapStorage(const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStack,
                            const std::string prefixString,
                            const std::string formatString,
                            const std::string formatUnitString,
                            const double resolution_meterPerTile);

  ///
  /// \brief Store the current mapstack without shifting or swaping
  /// \param nameMsg Name of layer to store. Store all if string is empty
  ///
  void storeMaps (const std_msgs::String nameMsg);

  ///
  /// \brief Swaps all necessary stacks
  ///
  virtual void swapStack();

  ///
  /// \brief Evaluates condition for storing refreshing and storing the mapstack
  /// \return True if all mapstacks are ready to be stored and swapped
  ///
  virtual inline bool mapRefreshAndStorageCondition();


  virtual void mapRefreshAndStorage(const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack,
                            const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStackShiftedResult,
                            const tf::StampedTransform transformRoiInWorld,
                            const std::string prefixString,
                            const std::string formatString,
                            const std::string formatUnitString,
                            const double resolution_meterPerTile,
                            const bool storeMapStack,
                            const bool shiftMapStack,
                            const bool clearMapStack,
                            TValue fillValue = 0.5f,
                            bool storeCurrentPosition = true,
                            std::string additionalInformationString = std::string(""));

  //  std::string topicMap;
  //  std::string topicLaser;
//  // Marging for non present information: 0.45 .. 0.55 -> 0.1

//  //!


//  // Global variables
//    // The map stack
//    cv::Mat storageMapBuffer;
//    std::vector<mrpt::maps::COccupancyGridMap2D> mapStack;  // Current stack for fusion
//    std::vector<mrpt::maps::COccupancyGridMap2D> mapStackStorageTemp;  // Temp stack for hdd storage
//    std::vector<mrpt::maps::COccupancyGridMap2D> mapStackShiftTemp; // Temp stack for shifting
//    mrpt::maps::COccupancyGridMap2D::TMapDefinition def;


 public:

  ///
  /// \brief // Sanity check for topic name, so it looks like "/some/scope" or at least "/"
  /// \param topic The topic name to refine
  ///
  static void topicRefinement(std::string &topic);


  void translateMap(cv::Mat &src, cv::Mat &dst, double offsetx = 0, double offsety = 0, TValue fillValue = msNumeric::invalidValue_int16);

  // Set all map tiles of all maps to the given value
  void fillMapStack(std::vector<TMapstack> &mapStack, float value = mapping::ogm::unknownOccupancyUpdateCertainty);

  // Translate a map
  void translateMap(TMapstack &map, int offsetx = 0, int offsety = 0, float fillProbability = mapping::ogm::unknownOccupancyUpdateCertainty);

  // Translate a stack of maps
  void translateMapStack(const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack, int offsetx = 0, int offsety = 0, float fillProbability = mapping::ogm::unknownOccupancyUpdateCertainty);



};


///
/// \brief Add subscriber to newly emerged topic with super-topic
/// \param T [template] The message type
/// \param subList Subscriber list which will be expanded by subscriber if new topics emerge
/// \param f Function name with declaration "void myFun(const myRosMsgType::ConstPtr &msg, const std::string &topic)"
/// \param superTopic Prefix of the topic in the form of "/my/super/topic/"
/// \param debug Print additional information
/// \param n The node handle
///
template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::advertiseSubscribers(
    std::vector<ros::Subscriber> &subList,
    void (TChild::*f)(const boost::shared_ptr< TData const>&, const std::string&),
    TChild *obj,
    const std::string &superTopic,
    const int &debug) {
//    ros::NodeHandle &n = ros::NodeHandle("~")) {

  if (superTopic.empty()) {
    ROS_ERROR_ONCE("No subscription to root topic");
    return;
  }

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
                        n->subscribe<TData>(topics.at(idx).name, 100, std::bind(f, obj, std::placeholders::_1, topics.at(idx).name)));
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
                        n->subscribe<TData>(topics.at(idx).name, 100, std::bind(f, obj, std::placeholders::_1, topics.at(idx).name)));
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
void
Mapserver<TMapstack, TData, TValue, TChild>::swapStack() {
  std::swap(this->currentMapStack, this->lastMapStack);
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
bool
Mapserver<TMapstack, TData, TValue, TChild>::mapRefreshAndStorageCondition() {
  return (this->lastMapStack.unique() && this->currentMapStack.unique());
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::tfTileNameHandler(const std_msgs::String nameMsg) {
  bool currentTileTfNameChange = false;
  mapRefresh.lock();
  if ((nameMsg.data.back() != currentTileTfName.back())) {
      if (currentTileTfName.empty()) {
          // First round, we bootstrap
          currentTileTfName = nameMsg.data;
      } else {
        this->swapStack();
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
    while(!this->mapRefreshAndStorageCondition()) {
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
                          storageNameMapKind,                    // Kind of map
                          storageNameFormat,                     // Format (empty: Take from type specifier)
                          storageNameUnit,                       // Unit
                          resolution_mPerTile,                   // Resolution per tile
                          !dontStoreMaps,                        // Info if maps should be stored
                          bool(shiftMap),                        // Info if maps should be shifted
                          !shiftMap,                             // If map is not shifted, reset the content of mapStack
                          mapInitValueApplied);                  // Fill-up value
  }

  mapRefresh.unlock();

}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::tupleHandler(const mapserver_msgs::pnsTuple msg) {
  ROS_ERROR("----------------- THIS IS SUBSCRIBPTION EXEC");
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
        lastTileTfName    = currentTileTfName;
        currentTileTfName = msg.string.data;
        currentTileTfNameChange = true;
      }
  }

  if (currentTileTfNameChange) {
    ROS_INFO("NEW MAP");
    tf::StampedTransform transformRoiInWorld;
    transformRoiInWorld.setOrigin(tf::Vector3(msg.point.x, msg.point.y, msg.point.z));
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
        << "lat_" << lastPnsTuple.navsat.latitude << "_"
        << "lon_" << lastPnsTuple.navsat.longitude << "_"
        << "alt_" << lastPnsTuple.navsat.altitude;

     mapRefreshAndStorage( lastMapStack,                          // Map to shift/store/reset
                           currentMapStack,                       // The result of the shifted map
                           transformRoiInWorld,                   // Transform
                           storageNameMapKind,                    // Kind of map
                           storageNameFormat,                       // Format (empty: Take from type specifier)
                           storageNameUnit,                       // Unit
                           resolution_mPerTile,                   // Resolution per tile
                           !dontStoreMaps,                        // Info if maps should be stored
                           bool(shiftMap),                        // Info if maps should be shifted
                           !shiftMap,                             // If map is not shifted, reset the content of mapStack
                           mapInitValueApplied,                   // Fill-up value
                           true,
                           navSatSs.str());
    // Store the current tile information as next last one
    lastPnsTuple = msg;
  }

  mapRefresh.unlock();
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::mapStorage(const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStack,
                          const std::string prefixString,
                          const std::string formatString,
                          const std::string formatUnitString,
                          const double resolution_meterPerTile) {

  const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> dummyMap;
  const tf::StampedTransform dummyTf;
  const TValue dummy = 0.0;

  mapRefreshAndStorage( mapStack,                                 // Map to shift/store/reset
                        dummyMap,                                 // Nothing at all
                        dummyTf,                                  // Transform
                        std::string("OGM"),                       // Kind of map
                        std::string(""),                          // Format (empty: Take from type specifier)
                        std::string("logodds"),                   // Unit
                        resolution_meterPerTile,                  // Resolution per tile
                        true,                                     // Maps should be stored
                        false,                                    // Maps should be shifted
                        false,                                    // Map should be reseted reset
                        dummy,                                    // Some float value
                        false);                                   // Don't store the current position

}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void Mapserver<TMapstack, TData, TValue, TChild>::storeMaps (const std_msgs::String nameMsg) {

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

  mapStorage(currentMapStack,
             storageNameMapKind,                    // Kind of map
             storageNameFormat,                       // Format (empty: Take from type specifier)
             storageNameUnit,                       // Unit
             resolution_mPerTile);                  // Resolution per tile
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::topicRefinement(std::string &topic) {
  if (topic.empty()) {
    topic = "/";
  } else {
    if (topic.at(0) != '/') {
      topic = std::string("/").append(topic);
    }
    if (topic.at(topic.size()-1) != '/') {
      topic.append("/");
    }
  }
}


template<typename TMapstack, typename TData, typename TValue, typename TChild>
Mapserver<TMapstack, TData, TValue, TChild>::Mapserver(ros::NodeHandle *nh, TChild* obj) :
        n(nh),
        currentTileTfName(""),
        lastTileTfName(""),
        mapSizeX(1),
        mapSizeY(1),
        mapStorageLocation(ms::constants::mapping::ogm::mapStorageLocation),
        dontStoreMaps(0),
        shiftMap(1),
        topicPrefix("/ism"),
        tileOriginTfPrefix("map_base_link_"),
        tileOriginTfSufixForRoiOrigin(ms::constants::machine::frames::names::ROI_ORIGIN),
        currentTfNameTopic("/currentTfTile"),
        currentTupleTopic(""),
        adaptiveSubscriptionTopicPrefix(""),
        worldLink("odom"),
        storeMapsTopic("/storemaps"),
        reqTopicMapStack("/reqMapStack"),
        idleStartupTime_s(-1.0),
        resolution_mPerTile(ms::constants::mapping::discreteResolution),
        maxDistanceInsertion(std::min(ms::constants::mapping::roi::altitude, std::min(ms::constants::mapping::roi::width, ms::constants::mapping::roi::height))),
        maxX_m(ms::constants::mapping::roi::xMax),
        minX_m(ms::constants::mapping::roi::xMin),
        maxY_m(ms::constants::mapping::roi::yMax),
        minY_m(ms::constants::mapping::roi::yMin),
        maxZ_m(ms::constants::mapping::roi::zMax),
        minZ_m(ms::constants::mapping::roi::zMin),
        mapInitValue(TValue(0.5)),
        debug(0),
        doTest(0),
        rate(1.0),
        storageNameMapKind(""),
        storageNameFormat(""),
        storageNameUnit("") {

  n->getParam("topic_prefix", this->topicPrefix);
  n->getParam("tile_origin_tf_prefix", this->tileOriginTfPrefix);
  n->getParam("tile_origin_tf_sufix_for_roi_origin", this->tileOriginTfSufixForRoiOrigin);
  n->getParam("current_tf_name_topic", this->currentTfNameTopic);
  n->getParam("current_tuple_topic", this->currentTupleTopic);
  n->getParam("current_tuple_topic", this->currentTupleTopic);
  ROS_ERROR_STREAM("2 Tuble topic " << this->currentTupleTopic);
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
  Mapserver::getMapInitValue(std::string("mapInit_value"), this->mapInitValue, n);
  n->getParam("map_storage_location", this->mapStorageLocation);
  n->getParam("shift_map", this->shiftMap);
  n->getParam("dont_store_maps", this->dontStoreMaps);
  n->getParam("rate", this->rate);
  n->getParam("storage_map_name", this->storageNameMapKind);
  n->getParam("storage_format_name", this->storageNameFormat);
  n->getParam("storage_unit_name", this->storageNameUnit);


  mapInitValueApplied = TValue(mapInitValue);
  listenerTf = new tf::TransformListener;
  Mapserver::topicRefinement(topicPrefix);

  mapSizeX = (this->maxX_m - this->minX_m) / resolution_mPerTile;
  mapSizeY = (this->maxY_m - this->minY_m) / resolution_mPerTile;
  mapSizeZ = (this->maxZ_m - this->minZ_m) / resolution_mPerTile;

  // Check whether we should get our tile information via a tuple or just via a name
  if (currentTupleTopic.empty()) {
//      subscriberTfTileName = Mapserver::n.subscribe<std_msgs::String>(currentTfNameTopic, 2, &Mapserver::tfTileNameHandler, this);
      this->subscriberTfTileName = n->subscribe(currentTfNameTopic, 2, &Mapserver::tfTileNameHandler, this);
  } else {
//      subscriberTuple = Mapserver::n.subscribe<mapserver_msgs::pnsTuple>(currentTupleTopic, 2, &Mapserver::tupleHandler, this);
    ROS_ERROR("----------------- THIS IS SUBSCRIBPTION");
      this->subscriberTuple = n->subscribe(currentTupleTopic, 2, &Mapserver::tupleHandler, this);
  }

  // Command scope to store the maps on demand
//  subscriberStoreMaps = Mapserver::n.subscribe<std_msgs::String>(storeMapsTopic, 1, &Mapserver::storeMaps, this);
  this->subscriberStoreMaps = n->subscribe(this->storeMapsTopic, 1, &Mapserver::storeMaps, this);

}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::mapRefreshAndStorage(
                          const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack,
                          const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStackShiftedResult,
                          const tf::StampedTransform transformRoiInWorld,
                          const std::string prefixString,
                          const std::string formatString,
                          const std::string formatUnitString,
                          const double resolution_meterPerTile,
                          const bool storeMapStack,
                          const bool shiftMapStack,
                          const bool clearMapStack,
                          TValue fillValue,
                          bool storeCurrentPosition,
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

template<typename TMapstack, typename TData, typename TValue, typename TChild>
Mapserver<TMapstack, TData, TValue, TChild>::~Mapserver() {
  delete listenerTf;
}



// Translate a map
template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::translateMap(cv::Mat &src, cv::Mat &dst, double offsetx, double offsety, TValue fillValue) {

  // Define a transformation for the image
  const cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);

  // Warp the image (which refers to the map)
  cv::warpAffine(src, dst, trans_mat,
                 cv::Size(src.rows, src.cols),
                 cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT,
                 cv::Scalar(static_cast<double>(fillValue)));

}


// Set all map tiles of all maps to the given value
template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::fillMapStack(std::vector<TMapstack> &mapStack, float value) {
  for (uint idx = 0; idx < mapStack.size(); ++idx) {
    mapStack[idx].fill (value);  // Fill every map with uncertainty
  }
}

// Translate a map
template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::translateMap(TMapstack &map, int offsetx, int offsety, float fillProbability) {

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
template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::translateMapStack(const std::shared_ptr<std::map<std::string, TMapstack*>> &mapStack, int offsetx, int offsety, float fillProbability) {
  for (auto it=mapStack->begin(); it!=mapStack->end(); ++it) {
    translateMap(*it->second, offsetx, offsety , fillProbability);
  }
}

template<typename TMapstack, typename TData, typename TValue, typename TChild>
void
Mapserver<TMapstack, TData, TValue, TChild>::getMapInitValue(const std::string paramName, TValue &mapInitValue, ros::NodeHandle *n) {
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
  } else if (std::is_same<TValue, short>::value) {
    int mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    ROS_ASSERT(mapInit_value < std::numeric_limits<short>::lowest() || mapInit_value > std::numeric_limits<short>::max());
    mapInitValue = short(mapInit_value);
  } else if (std::is_same<TValue, char>::value) {
    int mapInit_value;
    n->getParam("mapInit_value", mapInit_value);
    ROS_ASSERT(mapInit_value < std::numeric_limits<char>::lowest() || mapInit_value > std::numeric_limits<char>::max());
    mapInitValue = char(mapInit_value);
  } else {
    ROS_ERROR("No known conversion for TValue in mapserver");
    ROS_BREAK();
  }
}

