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
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/duration.h>

ros::Subscriber subOdom;
tf::TransformBroadcaster *brTf;
ros::Publisher currentTileTfNamePublisher;

static std::string tileParentTf("odom");
static std::string tileOriginTfPrefix("map_base_link_");
static std::string tileOriginTfSufixForRoiOrigin("_ROI_ORIGIN");
static std::string topicOdom("/bridge/Odom/S10Gps/filtered");
static std::string currentTfNameTopic("/currentTfTile");
static double idleStartupTime_s = -1.0;
static int tfTileHistory = 1, tfPublishRate = 50, tfNamePublishDelay = 10;
static std::size_t tfNamePublishDelayCounter = 0;
static int zFix = 1; // Omit altitude
std::mutex mtx;
static std::vector<std::tuple<geometry_msgs::Point, std::string> > tileCenterHistory;
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
  name = 1
};

///
/// \brief Decides if a tf for a new tile should be created
/// \param Odometry message to process
///
void odomHandler(const nav_msgs::Odometry odomMsg) {
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
      std::tuple<geometry_msgs::Point, std::string>(
	  odomMsg.pose.pose.position,
	  tileOriginTfPrefix + std::to_string(tileCenterHistory.size())));
  tfNamePublishDelayCounter = 0;
  mtx.unlock();
}

///
/// \brief Send the tile tf history and return
/// \return The current known tile tf name
///
std::string sendTfHistory(void) {
  tf::StampedTransform tileTf;
  tileTf.frame_id_ = tileParentTf;
  tileTf.setRotation(tf::Quaternion(.0, .0, .0, 1.0));
  tileTf.stamp_ = ros::Time::now();

  // If a history size for sending is defined, the start index is set
  const std::size_t histSizeStart = tfTileHistory < 0 ? 0 : tileCenterHistory.size() - tfTileHistory - 1;
  // Send the tf's
  mtx.lock();
  for (std::size_t idx = histSizeStart; idx < tileCenterHistory.size(); ++idx) {
    const tf::Vector3 translation(std::get<tupleEnum::pos>(tileCenterHistory.at(idx)).x,
				  std::get<tupleEnum::pos>(tileCenterHistory.at(idx)).y,
				  std::get<tupleEnum::pos>(tileCenterHistory.at(idx)).z);
    tileTf.setOrigin(translation);
    tileTf.child_frame_id_ = std::get<tupleEnum::name>(tileCenterHistory.at(idx));
    brTf->sendTransform(tileTf);
    brTf->sendTransform(tf::StampedTransform(utils::conversion::getTfFromEigen(roi_roiOrigin),
					     tileTf.stamp_,
					     tileTf.child_frame_id_,
					     tileTf.child_frame_id_ + tileOriginTfSufixForRoiOrigin));
    ROS_DEBUG("tileCenterHistory: %d/%d, x:%f, y:%f, z:%f, name:%s\n",
	      int(idx), int(tileCenterHistory.size()),
	      std::get<tupleEnum::pos>(tileCenterHistory.at(idx)).x,
	      std::get<tupleEnum::pos>(tileCenterHistory.at(idx)).y,
	      std::get<tupleEnum::pos>(tileCenterHistory.at(idx)).z,
	      std::get<tupleEnum::name>(tileCenterHistory.at(idx)).c_str());

  }
  mtx.unlock();
  return tileTf.child_frame_id_;
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
  n.param<std::string>("current_tf_name_topic", currentTfNameTopic, "/currentTfTile");
  n.param<double>("idle_startup_time", idleStartupTime_s, -1.0); // Wait before starting publishing (< 0 to disable)
  n.param<int>("tf_tile_history", tfTileHistory, -1); // Number of tiles in the history to publish (< 0 for complete history)
  n.param<int>("tf_publish_rate", tfPublishRate, 50);
  n.param<int>("tf_name_publish_delay", tfNamePublishDelay, 10);
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

  // Init the history
  tileCenterHistory.push_back(
      std::tuple<geometry_msgs::Point, std::string>(
	  geometry_msgs::Point(),
	  tileOriginTfPrefix + std::string("0")));

  // Init tf, publisher, subscriber
  brTf = new tf::TransformBroadcaster;
  currentTileTfNamePublisher = n.advertise<std_msgs::String>(currentTfNameTopic, 1);
  subOdom = n.subscribe<nav_msgs::Odometry>(topicOdom, 2, odomHandler);

  // Start the processing
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Do Stuff once a second
  ros::Rate rate(tfPublishRate);
  std_msgs::String currentTfTileName;
  while(ros::ok()) {
      currentTfTileName.data = sendTfHistory();
      if ((++tfNamePublishDelayCounter % tfNamePublishDelay) == 0) {
	      currentTileTfNamePublisher.publish(currentTfTileName);
	      tfNamePublishDelayCounter = 0;
      }
      rate.sleep();
  }

  delete brTf;

  return 0;
}
