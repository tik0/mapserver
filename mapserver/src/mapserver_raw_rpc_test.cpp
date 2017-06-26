#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <stdlib.h>
#include <Constants.h>

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

//OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//Claas
#include <mapserver_msgs/mapPrimitive.h>
#include <mapserver/rsm.h>

using namespace ms::constants;

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

void callService() {
  // Fill the request
  mapserver::rsm::Request req;
  mapserver::rsm::Response res;

  //Fill request for map server
  req.request.frame_id = frameId;
  req.request.width = int(width_m / resolution_meterPerTile);
  req.request.height = int(height_m / resolution_meterPerTile);
  req.request.depth = int(depth_m / resolution_meterPerTile);
  req.request.pose.pose.position.x = transX_m;
  req.request.pose.pose.position.y = transY_m;
  req.request.pose.pose.position.z = transZ_m;
  req.request.resolution = resolution_meterPerTile;

  tf::Quaternion q;
  double roll = 0.0, pitch = 0.0, yaw = yaw_deg * deg2rad;
  q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, req.request.pose.pose.orientation);

  std::stringstream os;
  os << req.request;
  ROS_INFO("Request: %s\n%s\n", serviceName.c_str(), os.str().c_str());
  // Call the service
  ROS_INFO("Call mapserver service");
  if (client.call(req, res)) {
    ROS_INFO("SUCCESS\n");
    ROS_INFO("RPC: rows=%i, cols=%i", res.response.rows, res.response.cols);

    cv::Mat img = cv::Mat::zeros(res.response.rows, res.response.cols, CV_8UC3);

    //Convert response from map server to cv image
    for (int j = 0; j < res.response.cols; j++) {
      for (int i = 0; i < res.response.rows; i++) {
        const int idx = (j) + (i) * res.response.cols;

        const float resVal = res.response.map[idx];

        uint16_t val = 0;

        if (resVal >= 0.0f
            && resVal != ms::constants::numeric::invalidValue_float) {
          val = static_cast<uint16_t>(resVal) + 1;
        } else {
          // val=0;
        }

        const cv::Vec3b color(static_cast<uchar>(val & 0xFF),
                              static_cast<uchar>(val >> 8),
                              static_cast<uchar>(0));
        img.at<cv::Vec3b>(i, j) = color;
      }
    }

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
  ros::init(argc, argv, "mapserver_raw_rpc_test");
  ros::NodeHandle node("~");
  node.param<double>("rate", rate, 1.0);
  node.param<double>("req_yaw_deg", yaw_deg, 0.0);
  node.param<float>("req_resolution_meterPerTile", resolution_meterPerTile,
                    0.1);
  node.param<std::string>("req_frame_id", frameId,
                          machine::frames::names::BASE_LINK);
  node.param<float>("req_width_m", width_m, 10);
  node.param<float>("req_height_m", height_m, 1);
  node.param<float>("req_depth_m", depth_m, 1);
  node.param<double>("req_transX_m", transX_m, 0.0);
  node.param<double>("req_transY_m", transY_m, 0.0);
  node.param<double>("req_transZ_m", transZ_m, 0.0);
  node.param<std::string>(
      "req_service_name",
      serviceName,
      scopes::map::rawServer::parent + std::string("/")
          + scopes::map::rawServer::requests::meanHeight);

  //Get rawmapserver service
  client = node.serviceClient<mapserver::rsm>(serviceName);

  ros::Rate _rate(rate);
  while (ros::ok()) {
    callService();
    _rate.sleep();
  }

  return 0;
}
