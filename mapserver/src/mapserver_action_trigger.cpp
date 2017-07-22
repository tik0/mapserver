#include <ros/ros.h>
#include <diagnostic_msgs/KeyValue.h>

int main(int argc, char **argv) {

  ros::Publisher publisher;
  std::string topic;
  std::string key;
  std::string value;
  int debug = 0;
  double rate;

  ros::init(argc, argv, "mapserver_storage_trigger");
  ros::NodeHandle n("~");
  n.param<std::string>("topic_action", topic, "/action");
  n.param<std::string>("key", key, "forget");
  n.param<std::string>("value", value, "0.5");
  n.param<double>("rate", rate, 1.0);
  n.param<int>("debug", debug, 0);

  publisher = n.advertise<diagnostic_msgs::KeyValue>(topic, 1);
  diagnostic_msgs::KeyValue msg;
  msg.key = key;
  msg.value = value;

  ROS_INFO_STREAM("Start action triggering with key/value: " << key << "/" << value);
  if (rate <= 0) {
    publisher.publish(msg);
  } else {
    ros::Rate _rate(rate);
    while (ros::ok()) {
      publisher.publish(msg);
      _rate.sleep();
    }
  }
  ROS_INFO_STREAM("Finish action triggering with key/value: " << key << "/" << value);

  return 0;
}

