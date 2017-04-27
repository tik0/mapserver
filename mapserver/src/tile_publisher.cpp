#include <string>
#include <tuple>
#include <mutex>

#include <Constants.h>
using namespace ms::constants;
using namespace ms::constants::mappingLayers;
namespace msNumeric = ms::constants::numeric;

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <mapserver_msgs/pnsTuple.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/duration.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

ros::Subscriber subOdom;
tf::TransformBroadcaster *brTf;
typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::NavSatFix> syncPolicy;

static std::string tileParentTf("odom");
static std::string tileOriginTfPrefix("map_base_link_");
static std::string tileOriginTfSufixForRoiOrigin("_ROI_ORIGIN");
static std::string topicOdom("/bridge/Odom/S10Gps/filtered");
static std::string topicNavsat("");
static std::string currentTfNameTopic("/currentTfTile");
static std::string currentTupleTopic("/currentTuple");
static double idleStartupTime_s = -1.0;
static int tfTileHistory = -1;
static double tfPublishRate = 50, tfNamePublishDelay = 0.5;
static std::size_t tfNamePublishDelayCounter = 0;
static int zFix = 1; // Omit altitude
std::mutex mtx;
typedef std::tuple<geometry_msgs::Point, sensor_msgs::NavSatFix, std::string> pnsTuple;
static std::vector<pnsTuple> tileCenterHistory;
static double boundingBoxWidth;
static double boundingBoxHeight;
static double boundingBoxAltitude;
static double roiTooOriginTransWidth;
static double roiTooOriginTransHeight;
static double roiTooOriginTransAltitude;
Eigen::Matrix4d roi_roiOrigin;

// At last include the utils
#include <utils.h>

enum tupleEnum {
  pos = 0,
  nav = 1,
  name = 2
};

void messageHandler(const nav_msgs::Odometry odomMsg, const sensor_msgs::NavSatFix navsatMsg = sensor_msgs::NavSatFix()) {
  if (tileParentTf.compare(odomMsg.header.frame_id)) {
      ROS_WARN_ONCE("%s != %s, this might be OK if the two frames coincide",
                    tileParentTf.c_str(), odomMsg.header.frame_id.c_str());
  }
  const double diffX = fabs(odomMsg.pose.pose.position.x - std::get<tupleEnum::pos>(tileCenterHistory.back()).x);
  const double diffY = fabs(odomMsg.pose.pose.position.y - std::get<tupleEnum::pos>(tileCenterHistory.back()).y);
  const double diffZ = fabs(odomMsg.pose.pose.position.z - std::get<tupleEnum::pos>(tileCenterHistory.back()).z);

  ROS_DEBUG("Odom pose x:%f, y:%f, z:%f\n"
            "Tile pose x:%f, y:%f, z:%f\n"
            "Diff      x:%f, y:%f, z:%f",
      odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.position.z,
      std::get<tupleEnum::pos>(tileCenterHistory.back()).x, std::get<tupleEnum::pos>(tileCenterHistory.back()).y, std::get<tupleEnum::pos>(tileCenterHistory.back()).z,
      diffX, diffY, diffZ);

  if ( diffX > (boundingBoxWidth / 2.0) ) {
      // Do map reset
  } else if ( diffY > (boundingBoxHeight / 2.0) ) {
      // Do map reset
  } else if ( !zFix && (diffZ > (boundingBoxAltitude / 2.0)) ) {
      // Do map reset
  } else {
      return;
  }

  // Reset the map
  mtx.lock();
  tileCenterHistory.push_back(
      pnsTuple(
    odomMsg.pose.pose.position,
    navsatMsg,
    tileOriginTfPrefix + std::to_string(tileCenterHistory.size())));
  tfNamePublishDelayCounter = 0;
  mtx.unlock();
}

///
/// \brief Decides if a tf for a new tile should be created
/// \param Odometry message to process
///
void odomHandler(const nav_msgs::Odometry::ConstPtr odomMsg) {
  messageHandler(*odomMsg);
}

void odomNavsatHandler(const nav_msgs::Odometry::ConstPtr odomMsg, const sensor_msgs::NavSatFix::ConstPtr navsatMsg) {
  messageHandler(*odomMsg, *navsatMsg);
}

///
/// \brief Send the tile tf history and return
/// \return The current known tile position, navsat, and tf name as tuple
///
pnsTuple sendTfHistory(void) {
  tf::StampedTransform tileTf;
  tileTf.frame_id_ = tileParentTf;
  tileTf.setRotation(tf::Quaternion(.0, .0, .0, 1.0));
  tileTf.stamp_ = ros::Time::now();

  mtx.lock();
  auto currentTileCenterHistory = tileCenterHistory;
  mtx.unlock();

  // If a history size for sending is defined, the start index is set
  const std::size_t histSizeStart = tfTileHistory < 0 ? 0 : currentTileCenterHistory.size() - tfTileHistory - 1;
  // Send the tf's
  for (std::size_t idx = histSizeStart; idx < currentTileCenterHistory.size(); ++idx) {
    const tf::Vector3 translation(std::get<tupleEnum::pos>(currentTileCenterHistory.at(idx)).x,
				  std::get<tupleEnum::pos>(currentTileCenterHistory.at(idx)).y,
				  std::get<tupleEnum::pos>(currentTileCenterHistory.at(idx)).z);
    tileTf.setOrigin(translation);
    tileTf.child_frame_id_ = std::get<tupleEnum::name>(currentTileCenterHistory.at(idx));
    brTf->sendTransform(tileTf);
    brTf->sendTransform(tf::StampedTransform(utils::conversion::getTfFromEigen(roi_roiOrigin),
					     tileTf.stamp_,
					     tileTf.child_frame_id_,
					     tileTf.child_frame_id_ + tileOriginTfSufixForRoiOrigin));
    ROS_DEBUG("tileCenterHistory: %d/%d, x:%f, y:%f, z:%f, name:%s\n",
	      int(idx), int(currentTileCenterHistory.size()),
	      std::get<tupleEnum::pos>(currentTileCenterHistory.at(idx)).x,
	      std::get<tupleEnum::pos>(currentTileCenterHistory.at(idx)).y,
	      std::get<tupleEnum::pos>(currentTileCenterHistory.at(idx)).z,
	      std::get<tupleEnum::name>(currentTileCenterHistory.at(idx)).c_str());

  }

  return *(currentTileCenterHistory.end()-1);
}

int main(int argc, char **argv)
{
  // ROS
  ros::init(argc, argv, "tile_publisher");
  ros::NodeHandle n("~");

  n.param<std::string>("tile_parent_tf", tileParentTf, "odom");
  n.param<std::string>("tile_origin_tf_prefix", tileOriginTfPrefix, "map_base_link_");
  n.param<std::string>("tile_origin_tf_sufix_for_roi_origin", tileOriginTfSufixForRoiOrigin, machine::frames::names::ROI_ORIGIN);
  n.param<std::string>("odometry_topic", topicOdom, "/bridge/Odom/S10Gps/filtered");
  n.param<std::string>("navsat_topic", topicNavsat, "");
  n.param<std::string>("current_tf_name_topic", currentTfNameTopic, "/currentTfTile");
  n.param<std::string>("current_tuple_topic", currentTupleTopic, "/currentTuple");
  n.param<double>("idle_startup_time", idleStartupTime_s, -1.0); // Wait before starting publishing (< 0 to disable)
  n.param<int>("tf_tile_history", tfTileHistory, -1); // Number of tiles in the history to publish (< 0 for complete history)
  n.param<double>("tf_publish_rate", tfPublishRate, 50);
  n.param<double>("tf_name_publish_delay", tfNamePublishDelay, 0.5); // Delay time in seconds to tell everybody to use the new tf frame
  n.param<int>("z_ix", zFix, 1); // Omit altitude for measurements and resetting
  // Parameter for the resetting
  n.param<double>("bounding_box_width", boundingBoxWidth, mapping::roi::boundingbox::width); // (m) Reset boundary in width
  n.param<double>("bounding_box_height", boundingBoxHeight, mapping::roi::boundingbox::height); // (m) Reset boundary in height
  n.param<double>("bounding_box_altitude", boundingBoxAltitude, mapping::roi::boundingbox::altitude); // (m) Reset boundary in altitude
  n.param<double>("roi_too_origin_trans_width", roiTooOriginTransWidth, -mapping::roi::originWidth); // (m) Translation in width from the center to the origin of the map in width
  n.param<double>("roi_too_origin_trans_height", roiTooOriginTransHeight, -mapping::roi::originHeight); // (m) Translation in width from the center to the origin of the map in height
  n.param<double>("roi_too_origin_trans_altitude", roiTooOriginTransAltitude, -mapping::roi::originAltitude); // (m) Translation in width from the center to the origin of the map in altitude
  roi_roiOrigin = ms::constants::machine::tf::trans<double>(roiTooOriginTransWidth, roiTooOriginTransHeight, roiTooOriginTransAltitude);

  ROS_INFO("ROI offset x: %f, y: %f, z: %f", roiTooOriginTransWidth, roiTooOriginTransHeight, roiTooOriginTransAltitude);
  ROS_INFO("BB size x: %f, y: %f, z: %f", boundingBoxWidth, boundingBoxHeight, boundingBoxAltitude);

  if (tfNamePublishDelay < std::numeric_limits<double>::epsilon()) {
      ROS_WARN("It is recommended to set tf_name_publish_delay > 0 to defy race conditions between the new tf and advertisement of using it");
  }

  // Init the history
  tileCenterHistory.push_back(
      pnsTuple(
          geometry_msgs::Point(),
          sensor_msgs::NavSatFix(),
          tileOriginTfPrefix + std::string("0")));

  // Init tf, publisher, subscriber
  brTf = new tf::TransformBroadcaster;
  ros::Publisher currentTileTfNamePublisher = n.advertise<std_msgs::String>(currentTfNameTopic, 1);
  ros::Publisher currentTuplePublisher = n.advertise<mapserver_msgs::pnsTuple>(currentTupleTopic, 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, topicOdom, 1);
  message_filters::Subscriber<sensor_msgs::NavSatFix> navsat_sub(n, topicNavsat, 1);
  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), odom_sub, navsat_sub);

  if (topicNavsat.empty()) {
      ROS_WARN("Don't use NAVSAT messages");
      subOdom = n.subscribe<nav_msgs::Odometry>(topicOdom, 2, odomHandler);
  } else {
      sync.registerCallback(boost::bind(&odomNavsatHandler, _1, _2));
  }

  // Start the processing
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Do Stuff once a second
  ros::Rate rate(tfPublishRate);
  pnsTuple pns = sendTfHistory();
  std_msgs::String currentTfTileName;
  mapserver_msgs::pnsTuple currentTuple;
  while(ros::ok()) {

      // Send the tfs and only refresh the tuple when the delay is over
      if ((++tfNamePublishDelayCounter / tfPublishRate) >= tfNamePublishDelay) {
          pns = sendTfHistory();
      } else {
          sendTfHistory();
      }

      // Tell the world which frame to use
      currentTfTileName.data = std::get<tupleEnum::name>(pns);
      currentTileTfNamePublisher.publish(currentTfTileName);
      if (!topicNavsat.empty()) {
        currentTuple.header.frame_id = tileParentTf;
        currentTuple.header.stamp = ros::Time::now();
        currentTuple.string.data = std::get<tupleEnum::name>(pns);
        currentTuple.point = std::get<tupleEnum::pos>(pns);
        currentTuple.navsat = std::get<tupleEnum::nav>(pns);
        currentTuplePublisher.publish(currentTuple);
      }
    rate.sleep();
  }

  delete brTf;

  return 0;
}
