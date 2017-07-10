#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <stdlib.h>
#include <Constants.hpp>

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

//OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//Mapserver
#include <mapserver_msgs/mapPrimitive.h>
#include <mapserver/ism.h>


using namespace constants;

ros::ServiceClient client;

static double rate = 1.0;

// Request values
static double yaw_deg = 90.0;
static std::string frameId = machine::frames::names::BASE_LINK;
static float resolution_meterPerTile = 0.1;
static float width_m = 10;
static float height_m = 1;
static float depth_m = 1;
static double transX_m = 0;
static double transY_m = 0;
static double transZ_m = 0;
static std::string serviceName = "";
static std::string reqInfo = "/ism/velodyne/svm/object";

void callService() {
  // Fill the request
  mapserver::ism::Request req;
  mapserver::ism::Response res;

  //Fill request for map server
  req.request.frame_id = frameId;
  req.request.header.frame_id = frameId;
  req.request.width = int(width_m / resolution_meterPerTile);
  req.request.height = int(height_m / resolution_meterPerTile);
  req.request.depth = int(depth_m / resolution_meterPerTile);
  req.request.pose.pose.position.x = transX_m;
  req.request.pose.pose.position.y = transY_m;
  req.request.pose.pose.position.z = transZ_m;
  req.request.resolution = resolution_meterPerTile;
  req.request.action = std::string(""); // can be multi or max
  req.request.req_info = reqInfo;

  tf::Quaternion q;
  double roll = 0.0, pitch = 0.0, yaw = yaw_deg * constants::deg2rad;
  q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, req.request.pose.pose.orientation);

  std::stringstream os;
  os << req.request;
  ROS_INFO("Request: %s\n%s\n", serviceName.c_str(), os.str().c_str());
  // Call the service
  ROS_INFO("Call mapserver service");
  if (client.call(req, res)) {
    ROS_INFO("SUCCESS\n");
    ROS_INFO("RPC: rows=%i, cols=%i", res.response.info.height, res.response.info.width);

    cv::Mat img = cv::Mat(res.response.info.height, res.response.info.width, CV_8SC1, res.response.data.data());

    // Show the RPC as image
    std::stringstream os;
    os << ros::Time::now();
    cv::imshow(std::string("mapRequest"), img);
    cv::setWindowTitle(std::string("mapRequest"),
                       std::string("mapRequest at ") + os.str());
    cv::waitKey(1);
  } else {
    ROS_ERROR("Failed to call mapserver service");
  }
}

/**
 * This program requests a view
 * rosrun mapserver mapserver_raw_rpc_test
 * rosrun mapserver mapserver_raw_rpc_test _req_service_name:=/rawServer/variancePulsewidth
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "mapserver_raw_stat_test");
  ros::NodeHandle node("~");
  node.param<double>("rate", rate, 1.0);
  node.param<double>("req_yaw_deg", yaw_deg, 0.0);
  node.param<float>("req_resolution_meterPerTile", resolution_meterPerTile,
                    0.1);
  node.param<std::string>("req_info", reqInfo,
                          "/ism/velodyne/svm/object");
  node.param<std::string>("req_frame_id", frameId,
                          constants::machine::frames::names::BASE_LINK);
  node.param<float>("req_width_m", width_m, 10);
  node.param<float>("req_height_m", height_m, 1);
  node.param<float>("req_depth_m", depth_m, 1);
  node.param<double>("req_transX_m", transX_m, 0.0);
  node.param<double>("req_transY_m", transY_m, 0.0);
  node.param<double>("req_transZ_m", transZ_m, 0.0);
  node.param<std::string>(
      "req_service_name",
      serviceName,
      constants::scopes::map::ogmServer::parent + std::string("/")
          + constants::scopes::map::ogmServer::requests::singleLayerOgm);

  //Get mapserver service
  client = node.serviceClient<mapserver::ism>(serviceName);

  ros::Rate _rate(rate);
  while (ros::ok()) {
    callService();
    _rate.sleep();
  }

  return 0;
}
