#ifndef CONSTANTS_SCOPES_HPP_
#define CONSTANTS_SCOPES_HPP_

#include "common.hpp"

namespace constants {
namespace scopes {
const std::string root("");

namespace map {
namespace displayRoi {
const std::string displayRoi("/displayRoi/compressed");
}

namespace rawServer {
const std::string parent("/rawServer");
namespace requests {
const std::string height("height");
const std::string meanHeight("meanHeight");
const std::string meanHeightTest("meanHeightTest");
const std::string varianceHeight("varianceHeight");
const std::string varianceHeightTest("varianceHeightTest");
const std::string quantil90Height("quantil90Height");
const std::string quantil90HeightTest("quantil90HeightTest");
const std::string quantil95Height("quantil95Height");
const std::string quantil95HeightTest("quantil95HeightTest");
const std::string pulsewidth("pulsewidth");
const std::string meanPulsewidth("meanPulsewidth");
const std::string meanPulsewidthTest("meanPulsewidthTest");
const std::string variancePulsewidth("variancePulsewidth");
const std::string variancePulsewidthTest("variancePulsewidthTest");
const std::string quantil90Pulsewidth("quantil90Pulsewidth");
const std::string quantil90PulsewidthTest("quantil90PulsewidthTest");
const std::string quantil95Pulsewidth("quantil95Pulsewidth");
const std::string quantil95PulsewidthTest("quantil95PulsewidthTest");
}
}

namespace ogmServer {
const std::string parent("/mapServer");
namespace requests {
const std::string compressedMapImage("compressedMapImageReq");
const std::string singleLayerOgm("singleLayerOgmReq");
// These are the request strings in a singleLayerOgm-server request
const std::string stockDensity1("stockDensity1");
const std::string stockDensity2("stockDensity2");
const std::string stockDensity3("stockDensity3");
const std::string obstacleNonProcNonAcc("obstacleNonProcNonAcc");
const std::string obstacleProcNonAcc("obstacleProcNonAcc");
const std::string obstacleNonProcAcc("obstacleNonProcAcc");
const std::string stockFlattened("stockFlattened");
const std::string processed("processed");
const std::string stockEdge("stockEdge");
}
}
namespace super {
const std::string ogm("/ism");
const std::string raw("/map/raw");
}
namespace sub {
const std::string stockDensity1(
    std::string("/") + ogmServer::requests::stockDensity1);
const std::string stockDensity2(
    std::string("/") + ogmServer::requests::stockDensity2);
const std::string stockDensity3(
    std::string("/") + ogmServer::requests::stockDensity3);
const std::string obstacleNonProcNonAcc(
    std::string("/") + ogmServer::requests::obstacleNonProcNonAcc);
const std::string obstacleProcNonAcc(
    std::string("/") + ogmServer::requests::obstacleProcNonAcc);
const std::string obstacleNonProcAcc(
    std::string("/") + ogmServer::requests::obstacleNonProcAcc);
const std::string stockFlattened(
    std::string("/") + ogmServer::requests::stockFlattened);
const std::string processed(std::string("/") + ogmServer::requests::processed);
const std::string stockEdge(std::string("/") + ogmServer::requests::stockEdge);
}
const std::string stockDensity1(super::ogm + sub::stockDensity1);
const std::string stockDensity2(super::ogm + sub::stockDensity2);
const std::string stockDensity3(super::ogm + sub::stockDensity3);
const std::string obstacleNonProcNonAcc(
    super::ogm + sub::obstacleNonProcNonAcc);
const std::string obstacleProcNonAcc(super::ogm + sub::obstacleProcNonAcc);
const std::string obstacleNonProcAcc(super::ogm + sub::obstacleNonProcAcc);
const std::string stockFlattened(super::ogm + sub::stockFlattened);
const std::string processed(super::ogm + sub::processed);
const std::string stockEdge(super::ogm + sub::stockEdge);
}
}
}
#endif // CONSTANTS_SCOPES_HPP_
