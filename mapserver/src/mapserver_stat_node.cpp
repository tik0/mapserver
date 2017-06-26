#include <mapserver_stat.hpp>

int main(int argc, char **argv) {

  ros::init(argc, argv, "mapserver_stat");
  ros::NodeHandle n("~");

  MapserverStat ms(n);
  ms.spin();

  return 0;
}
