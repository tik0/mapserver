#include <mapserver_raw.hpp>

int main(int argc, char **argv) {

  ros::init(argc, argv, "mapserver_raw");
  ros::NodeHandle n("~");

  MapserverRaw ms(n);
  ms.spin();

  return 0;
}
