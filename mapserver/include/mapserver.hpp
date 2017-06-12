
#include "Constants.hpp"

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

template<typename TMapstack, typename TValue>
class Mapserver {

 public:
  Mapserver(const ros::NodeHandle& nh);

  void ~Mapserver();

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
  static void advertiseSubscribers(
      std::vector<ros::Subscriber> &subList,
      void (&f)(const boost::shared_ptr< T const>&, const std::string&),
      const std::string &superTopic,
      const int &debug,
      ros::NodeHandle &n = ros::NodeHandle("~"));

 protected:
  //! The node handle
  const ros::NodeHandle n;
  //! Holding the current tile name
  std::string currentTileTfName;
  //! Holding the former tile name
  std::string lastTileTfName;
  //! Discrete map size in x direction (s.t. width)
  int mapSizeX;
  //! Discrete map size in y direction (s.t. depth)
  int mapSizeY;
  //! Location to store the maps
  std::string mapStorageLocation;
  //! Indicator, if maps should be stored if currentTileTfName changes
  int dontStoreMaps = 0;
  //! Indicator, if maps should be shifted in the memory if currentTileTfName changes
  int shiftMap = 1;
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
  std::string storageNameKind;
  //! Storage name: Unit
  std::string storageNameUnit;



  //! Mutex to lock the mapstacks
  std::mutex mapRefresh;
  //! The current mapstack (Active mapstack in the double buffer)
  std::shared_ptr<TMapstack> currentMapStack;
  //! The last mapstack (Passive mapstack in the double buffer)
  std::shared_ptr<TMapstack> lastMapStack;

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
  /// \brief Store the current mapstack without shifting or swaping
  /// \param nameMsg Name of layer to store. Store all if string is empty
  ///
  void storeMaps (const std_msgs::String nameMsg);


  virtual void mapRefreshAndStorage(const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStack,
                            const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStackShiftedResult,
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

  //  std::string debugIsmTopic;
  //  std::string topicMap;
  //  std::string topicLaser;
//  std::string ismScopePrefix = scopes::map::super::ogm;
//  // Marging for non present information: 0.45 .. 0.55 -> 0.1
//  float uncertaintyBoundary = mapping::ogm::minDrawOccupancyUpdateCertainty;
//  float mapInitValue = mapping::ogm::unknownOccupancyUpdateCertainty;

//  //!
//  float maxOccupancyUpdateCertainty = mapping::ogm::maxOccupancyUpdateCertainty;

//  // Global variables
//    // The map stack
//    cv::Mat storageMapBuffer;
//    std::vector<mrpt::maps::COccupancyGridMap2D> mapStack;  // Current stack for fusion
//    std::vector<mrpt::maps::COccupancyGridMap2D> mapStackStorageTemp;  // Temp stack for hdd storage
//    std::vector<mrpt::maps::COccupancyGridMap2D> mapStackShiftTemp; // Temp stack for shifting
//    mrpt::maps::COccupancyGridMap2D::TMapDefinition def;

//  std::string debugTopic;
//  n.param<std::string>("debug_topic", debugTopic, "/amiro2/ism/cam"); // The topic of the fused map to show via opencv
//  n.param<float>("uncertainty_boundary", uncertaintyBoundary, 0.5); // Uncertainty boundary for displaying a feature (standard: 0.5 (unknown))
//  n.param<float>("max_occupancy_update_certainty", maxOccupancyUpdateCertainty);
//  n.param<std::string>("ism_scope_prefix", ismScopePrefix, scopes::map::super::ogm); // Scope prefix for the inverse sensor models


//  std::string topicDebugGridPrefix;
//  n.param<std::string>("topic_debug_grid_prefix", topicDebugGridPrefix, "/viz");
 public:





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
template<typename T>
static void Mapserver<void, void>::advertiseSubscribers(
    std::vector<ros::Subscriber> &subList,
    void (&f)(const boost::shared_ptr< T const>&, const std::string&),
    const std::string &superTopic,
    const int &debug,
    ros::NodeHandle &n = ros::NodeHandle("~")) {

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


template <typename TMapstack, typename TValue>
void Mapserver<TMapstack, TValue>::tfTileNameHandler(const std_msgs::String nameMsg) {
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
                          storageNameMapKind,                    // Kind of map
                          storageNameKind,                       // Format (empty: Take from type specifier)
                          storageNameUnit,                       // Unit
                          resolution_mPerTile,                   // Resolution per tile
                          !dontStoreMaps,                        // Info if maps should be stored
                          bool(shiftMap),                        // Info if maps should be shifted
                          !shiftMap,                             // If map is not shifted, reset the content of mapStack
                          mapInitValueApplied);                  // Fill-up value
  }

  mapRefresh.unlock();

}

template <typename TMapstack, typename TValue>
void Mapserver<TMapstack, TValue>::tupleHandler(const mapserver_msgs::pnsTuple msg) {
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
                           storageNameKind,                       // Format (empty: Take from type specifier)
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

template <typename TMapstack, typename TValue>
void Mapserver<TMapstack, TValue>::storeMaps (const std_msgs::String nameMsg) {

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
             storageNameKind,                       // Format (empty: Take from type specifier)
             storageNameUnit,                       // Unit
             resolution_mPerTile);                  // Resolution per tile
}


template <typename TMapstack, typename TValue>
Mapserver<TMapstack, TValue>::Mapserver(const ros::NodeHandle& nh) :
        n(nh),
        currentTileTfName(""),
        lastTileTfName(""),
        mapSizeX(1),
        mapSizeY(1),
        mapStorageLocation(ms::constants::mapping::ogm::mapStorageLocation),
        dontStoreMaps(0),
        shiftMap(1),
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
        mapInitValue(0.5),
        debug(0),
        doTest(0),
        rate(1.0),
        storageNameMapKind(""),
        storageNameKind(""),
        storageNameUnit("") {

  this->n.param<std::string>("tile_origin_tf_prefix", tileOriginTfPrefix);
  this->n.param<std::string>("tile_origin_tf_sufix_for_roi_origin", tileOriginTfSufixForRoiOrigin);
  this->n.param<std::string>("current_tf_name_topic", currentTfNameTopic);
  this->n.param<std::string>("current_tuple_topic", currentTupleTopic);
  this->n.param<std::string>("world_link", worldLink);
  this->n.param<std::string>("store_maps_topic", storeMapsTopic);
  this->n.param<std::string>("req_topic_map_stack", reqTopicMapStack);
  this->n.param<double>("idle_startup_time", idleStartupTime_s);
  this->n.param<int>("debug", debug);
  this->n.param<int>("test", doTest);
  this->n.param<double>("resolution", resolution_mPerTile);
  this->n.param<float>("max_distance_insertion", maxDistanceInsertion);
  this->n.param<float>("max_x_m", maxX_m);
  this->n.param<float>("min_x_m", minX_m);
  this->n.param<float>("max_y_m", maxY_m);
  this->n.param<float>("min_y_m", minY_m);
  this->n.param<float>("max_z_m", maxZ_m);
  this->n.param<float>("min_z_m", minZ_m);
  this->n.param<double>("mapInit_value", mapInitValue);
  this->n.param<std::string>("map_storage_location", mapStorageLocation);
  this->n.param<int>("shift_map", shiftMap);
  this->n.param<int>("dont_store_maps", dontStoreMaps);
  this->n.param<float>("rate", rate);

  mapInitValueApplied = TValue(mapInitValue);
  listenerTf = new tf::TransformListener;

  if (currentTupleTopic.empty()) {
      subscriberTfTileName = this->n.subscribe<std_msgs::String>(currentTfNameTopic, 2, tfTileNameHandler);
  } else {
      subscriberTuple = this->n.subscribe<mapserver_msgs::pnsTuple>(currentTupleTopic, 2, tupleHandler);
  }
  subscriberStoreMaps = this->n.subscribe<std_msgs::String>(storeMapsTopic, 1, storeMaps);

}

template <typename TMapstack, typename TValue>
Mapserver<TMapstack, TValue>::~Mapserver() {
  delete listenerTf;
}
