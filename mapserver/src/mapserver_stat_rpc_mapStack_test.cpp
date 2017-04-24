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

#include <mapserver/ismStackFloat.h>

using namespace ms::constants;

ros::ServiceClient client;

static double rate = 1.0;

// Request values
static std::string reqServiceName = "";
static std::string reqString = "";


void callService() {

    // Fill the request
    mapserver::ismStackFloat::Request req;
    mapserver::ismStackFloat::Response res;

    if (!reqString.empty()) {
        req.request.strings.resize(1);
        req.request.strings.at(1) = reqString;
    }

    ROS_INFO("Call mapserver service");
    if (client.call(req,res)){
        ROS_INFO("SUCCESS\n");
        int idx = 0;
        auto itMap = res.response.mapStack.begin();
        for (auto it = res.response.mapNames.strings.begin(); it != res.response.mapNames.strings.end(); ++it, ++idx, ++itMap) {
            ROS_INFO("RPC: %d map name=%s", idx, it->c_str());

            cv::Mat mat = cv::Mat(itMap->info.height,itMap->info.width,CV_32FC1, (void*)itMap->map.data());
            cv::imshow(*it, mat);
            cv::waitKey(1);
        }
    } else{
        ROS_ERROR("Failed to call mapserver service");
    }
}

 /**
 * This program requests a view
 * rosrun mapserver mapserver_raw_rpc_test
 * rosrun mapserver mapserver_raw_rpc_test _req_service_name:=/rawServer/variancePulsewidth
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapserver_raw_rpc_test");
  ros::NodeHandle node("~");
  node.param<double>("rate", rate, 1.0);
  node.param<std::string>("req_string", reqString, "");
  node.param<std::string>("req_service_name", reqServiceName, "/reqMapStack");

   client = node.serviceClient<mapserver::ismStackFloat>(reqServiceName);


   ros::Rate _rate(rate);
   while(ros::ok()) {
       callService();
       _rate.sleep();
   }

  return 0;
}
