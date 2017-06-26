#ifndef CONSTANTS_MAPPING_HPP_
#define CONSTANTS_MAPPING_HPP_

#include "common.hpp"
#include "scopes.hpp"

namespace constants {
namespace mapping {
/** \brief Map resolution */
const double discreteResolution = 0.1;  // meter per cell

namespace roi {

namespace boundingbox {
/** \brief Boundingbox width */
const double width = 8;  // meter

/** \brief Boundingbox height */
const double height = 8;  // meter

/** \brief Boundingbox altitude */
// WARNING is not checked in mapserver/tile_publisher
const double altitude = 0;  // meter
}

/** \brief Map width */
const double width = 35;  // meter

/** \brief Map height */
const double height = 35;  // meter

/** \brief Map altitude */
const double altitude = 35;  // meter

/** \brief Coordinate system origin in lateral (x) direction */
const double originWidth = width / 2.0;  // meter

/** \brief Coordinate system origin in vertical (y) direction */
const double originHeight = height / 2.0;  // meter

/** \brief Coordinate system origin in horizontal (z) direction */
const double originAltitude = altitude / 2.0;  // meter

/** \brief Minimum lateral coordinate (x) */
const double xMin = originWidth - width;  // meter

/** \brief Maximum lateral coordinate (x) */
const double xMax = width - originWidth;  // meter

/** \brief Minimum vertical coordinate (y) */
const double yMin = originHeight - height;  // meter

/** \brief Maximum vertical coordinate (y) */
const double yMax = height - originHeight;  // meter

/** \brief Minimum vertical coordinate (z) */
const double zMin = originAltitude - altitude;  // meter

/** \brief Maximum vertical coordinate (z) */
const double zMax = altitude - originHeight;  // meter

namespace discrete {
/** \brief Map width */
const size_t width = roi::width / discreteResolution;  // cells

/** \brief Map height */
const size_t height = roi::height / discreteResolution;  // cells

/** \brief Amount of cells in the map */
const size_t numCells = width * height;

/** \brief Coordinate system origin in lateral (x) direction */
const size_t originWidth = width / 2.0;

/** \brief Coordinate system origin in vertical (y) direction */
const size_t originHeight = height / 2.0;

/** \brief Minimum lateral coordinate (x) */
const size_t xMin = originWidth - width;

/** \brief Maximum lateral coordinate (x) */
const size_t xMax = width - originWidth;

/** \brief Minimum vertical coordinate (y) */
const size_t yMin = originHeight - height;

/** \brief Maximum vertical coordinate (y) */
const size_t yMax = height - originHeight;

/** \brief Minimum vertical coordinate (z) */
const double zMin = originAltitude - altitude;

/** \brief Maximum vertical coordinate (z) */
const double zMax = altitude - originHeight;
}
}

namespace ogm {

/** \brief Maximum allowed certainty value in the OGM server */
const double maxOccupancyUpdateCertainty = 0.8;

/** \brief Minimum allowed certainty value in the OGM server */
const double minOccupancyUpdateCertainty = 0.2;

/** \brief OGM value for unknown */
const double unknownOccupancyUpdateCertainty = 0.5;

/** \brief Minimum certainty value, to be drawn into the colored image */
const double minDrawOccupancyUpdateCertainty = 0.6;

/** \brief Standard storage location */
const std::string mapStorageLocation("/tmp/");
}
}

namespace mappingLayers {
const uint NUM_MAPS = 9;

enum maps {
  stockDensity1 = 0,
  stockDensity2,
  stockDensity3,
  obstacleNonProcNonAcc,
  obstacleProcNonAcc,
  obstacleNonProcAcc,
  stockFlattened,
  stockEdge,
  processed  // Overwrites all other properties for visualization and other reasoning
};

static std::string mapSubScopes[NUM_MAPS] { constants::scopes::map::sub::stockDensity1,
    constants::scopes::map::sub::stockDensity2, scopes::map::sub::stockDensity3,
    constants::scopes::map::sub::obstacleNonProcNonAcc,
    constants::scopes::map::sub::obstacleProcNonAcc, scopes::map::sub::obstacleNonProcAcc,
    constants::scopes::map::sub::stockFlattened, scopes::map::sub::processed,
    constants::scopes::map::sub::stockEdge };

static std::string mapRequestScopes[NUM_MAPS] {
    scopes::map::ogmServer::requests::stockDensity1,
    scopes::map::ogmServer::requests::stockDensity2,
    scopes::map::ogmServer::requests::stockDensity3,
    scopes::map::ogmServer::requests::obstacleNonProcNonAcc,
    scopes::map::ogmServer::requests::obstacleProcNonAcc,
    scopes::map::ogmServer::requests::obstacleNonProcAcc,
    scopes::map::ogmServer::requests::stockFlattened,
    scopes::map::ogmServer::requests::processed,
    scopes::map::ogmServer::requests::stockEdge };

static std::string ismScopes[NUM_MAPS] { scopes::map::stockDensity1,
    scopes::map::stockDensity2, scopes::map::stockDensity3,
    scopes::map::obstacleNonProcNonAcc, scopes::map::obstacleProcNonAcc,
    scopes::map::obstacleNonProcAcc, scopes::map::stockFlattened,
    scopes::map::processed, scopes::map::stockEdge };

const int NUM_BGR_CHANNELS = 3;

static unsigned char mapColorBGR[NUM_MAPS][NUM_BGR_CHANNELS] { { 255, 0, 0 },  // Light blue
    { 200, 0, 0 },  // Mid blue
    { 127, 0, 0 },  // Dark blue
    { 0, 0, 255 },  // Dark red
    { 42, 42, 165 },  // Brown
    { 30, 105, 210 },  // Chocolat
    { 200, 0, 200 },  // Purple
    { 0, 255, 0 },  // Green
    { 0, 255, 255 }  // Yellow
};

}
}
#endif // CONSTANTS_MAPPING_HPP_
