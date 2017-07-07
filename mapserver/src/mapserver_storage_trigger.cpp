#include <ros/ros.h>
//#include <ros/spinner.h>
//#include <ros/console.h>
//#include <nav_msgs/OccupancyGrid.h>
//#include <std_msgs/String.h>
//#include <rosgraph_msgs/Log.h>
//#include <tf/transform_listener.h>
//#include <mapserver/rsm.h>
//#include <tf_conversions/tf_eigen.h>
//#include <rosapi/Topics.h>
#include <mapserver_msgs/StringStamped.h>
#include <xmlrpcpp/XmlRpc.h>

int main(int argc, char **argv) {

  ros::Publisher publisher; // The trigger publisher
  std::string triggerTopic;  // Scope for triggering the storage
  std::string topicsCsv;  // Topics for storage as CSV
  std::vector<ros::Time> timestamps;  // All timestamps to process
  int debug; // Send the whole vector regardless of time

  ros::init(argc, argv, "mapserver_storage_trigger");
  ros::NodeHandle n("~");
  n.param<std::string>("store_maps_topic", triggerTopic, "/storemaps");
  n.param<std::string>("topics", topicsCsv, "");
  n.param<int>("debug", debug, 0);

  // Load the timestamps
  try {
    XmlRpc::XmlRpcValue timestampList;
    n.getParam(std::string("timestamp"), timestampList);

    ROS_INFO("Check if param is an array");
    ROS_ASSERT(timestampList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    timestamps.resize(timestampList.size());

    ROS_INFO("Copy the parameters");
    for (std::size_t idx = 0; idx < timestampList.size(); ++idx) {
      ROS_ASSERT(
          timestampList[idx].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      timestamps.at(idx) = ros::Time(static_cast<double>(timestampList[idx]));
    }
    ROS_INFO_STREAM("Copied " << timestampList.size() << " entries");
  } catch (XmlRpc::XmlRpcException a) {
    std::cerr << "XmlRpc exception: " << a.getMessage() << std::endl;
    timestamps.resize(0);
  }

  publisher = n.advertise < mapserver_msgs::StringStamped > (triggerTopic, 1);
  mapserver_msgs::StringStamped msg;
  msg.data = topicsCsv;

  std::size_t idx = 0;
  if (timestamps.size() > 0) {
    while (ros::ok()) {
      const ros::Time currentTimestamp = timestamps.at(idx++);
      ros::Duration duration(currentTimestamp - ros::Time::now());
      if (duration > ros::Duration(0.0) || debug) {
        if (!debug) {
          ROS_DEBUG_STREAM("Sleep until " << currentTimestamp << " from now " << ros::Time::now());
          ros::Time::sleepUntil(currentTimestamp);
        }
        msg.header.stamp = currentTimestamp;
        publisher.publish(msg);
        ROS_INFO_STREAM("Published " << currentTimestamp);
      } else {
        ROS_INFO_STREAM("Skipped " << currentTimestamp);
      }

      if (idx >= timestamps.size()) {
        break;
      }
    }
  } else {
    ROS_ERROR("timestamps are empty");
  }

  ROS_INFO("Triggering finished");
  return 0;
}

