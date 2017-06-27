#include "mapserver.hpp"
#include <boost/math/special_functions/erf.hpp>
#include <Constants.hpp>
#include <nav_msgs/GridCells.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <utils.h>

using namespace constants;
using namespace constants::mappingLayers;
namespace numerics = constants::numeric;

class MapserverRaw : public Mapserver<short, nav_msgs::OccupancyGrid, short,
    MapserverRaw> {

 private:

  //! The node handle
  ros::NodeHandle n;

  std::mutex mtxSwap;
  std::mutex mtxShowRpc;

 public:

  ///
  /// \brief The constructor
  /// \param nh The node handle
  ///
  MapserverRaw(ros::NodeHandle& nh);

  ///
  /// \brief The destructor
  ///
  virtual ~MapserverRaw() {
  }
  ;

  ros::Publisher publisherMap;
  ros::Subscriber subscriberLaser, subscriberTfTileName;
  laser_geometry::LaserProjection projector;

  // Variables
  std::string topicMap;
  std::string topicLaser;
  float maxDistanceInsertion_m;
  int debugDrawRpc = 0;
  int sendTopLayerOfDistanceAsOgm;
  cv::Mat storageMapBuffer;
  int maxLayer;
  std::shared_ptr<std::vector<cv::Mat>> currentMapHeight_mm, lastMapHeight_mm;
  std::shared_ptr<std::vector<cv::Mat>> currentPulsewidth_ps, lastPulsewidth_ps;
  std::shared_ptr<std::vector<cv::Mat>> currentMapIterator, lastMapIterator;
  cv::Mat mapIterator_1, mapIterator_2;  // Pointer to the current position in the mapHeight map

  // Services
  ros::ServiceServer service_meanHeight;
  ros::ServiceServer service_varianceHeight;
  ros::ServiceServer service_quantil90Height;
  ros::ServiceServer service_quantil95Height;
  ros::ServiceServer service_meanPulsewidth;
  ros::ServiceServer service_variancePulsewidth;
  ros::ServiceServer service_quantil90Pulsewidth;
  ros::ServiceServer service_quantil95Pulsewidth;

  // Debug stuff for service RPC
  cv::Mat mapStackStatisticRequestDebug, mapStackStatisticDebug;
  cv::RotatedRect rect;

  // Translate a map
  template<typename T>
  void translateMap(cv::Mat &src, cv::Mat &dst, double offsetx = 0,
                    double offsety = 0, T fillValue =
                        numerics::invalidValue_int16) {

    // Define a transformation for the image
    const cv::Mat trans_mat =
        (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);

    // Warp the image (which refers to the map)
    cv::warpAffine(src, dst, trans_mat, cv::Size(src.rows, src.cols),
                   cv::INTER_NEAREST, cv::BORDER_CONSTANT,
                   cv::Scalar(static_cast<double>(fillValue)));

  }

  ///
  /// \brief Refresh or shift the map layers in the stack, plus the possibility to store the map stack as images on the hard drive
  /// \param mapStack The stack
  /// \param mapStackShiftedResult The result of the shifted map
  /// \param transformRoiInWorld The transformation refers the center of the ROI in the world, so that the information is stored in the map name
  /// \param prefixString A prefix for the filename
  /// \param formatString The string describing the the encoding format of the stack content (e.g. 8UC1: unsigned char (See OpenCV format for this string: http://docs.opencv.org/java/2.4.7/org/opencv/core/CvType.html)). If empty, OpenCV type is taken.
  /// \param formatUnitString Unit of the data content (eg. mm for millimeter)
  /// \param resolution_meterPerTile Size of quadratic tile in meter
  /// \param storeMapStack Indicator for storing the map stack to the hard drive (>0: store maps)
  /// \param shiftMapStack Indicator for shifting the map stack
  /// \param clearMapStack Set the whole map stack to the default value
  /// \param fillValue Value to fill up a cleared or shifted map
  ///
  template<typename T>
  void mapRefreshAndStorage(
      std::shared_ptr<std::vector<cv::Mat>> mapStack,
      std::shared_ptr<std::vector<cv::Mat>> mapStackShiftedResult,
      const tf::StampedTransform transformRoiInWorld,
      const std::string prefixString, const std::string formatString,
      const std::string formatUnitString, const double resolution_meterPerTile,
      const bool storeMapStack, const bool shiftMapStack,
      const bool clearMapStack, T fillValue = numerics::invalidValue_int16) {

    // The message from the last time the function was called (So it is the location of the center)
    static tf::StampedTransform transformRoiInWorldLast;

    if (storeMapStack) {
      // Get the timestamp in microseconds
      uint64_t timestamp = uint64_t(transformRoiInWorld.stamp_.sec)
          * uint64_t(1e6) + uint64_t(transformRoiInWorld.stamp_.nsec / 1e6);
      // Store each layer in the map
      for (int mapIdx = 0; mapIdx < maxLayer; ++mapIdx) {
        // Get the format string
        const std::string format =
            formatString.empty() ?
                utils::conversion::format2str(mapStack->at(mapIdx).type()) :
                formatString;
        // Get the filename
        std::ostringstream oss;
        oss << mapStorageLocation << "What_" << prefixString << "_"
            << "Timestamp_" << timestamp << "us_" << "Format_" << format << "_"
            << "Unit_" << formatUnitString << "_" << "LayerIdx_" << mapIdx
            << "_" << "GridRes_"
            << resolution_meterPerTile * geometry::millimeterPerMeter << "mm_"
            << "X_" << transformRoiInWorld.getOrigin().x() << "m_" << "Y_"
            << transformRoiInWorld.getOrigin().y() << "m_" << "Z_"
            << transformRoiInWorld.getOrigin().z() << "m_" << "rows_"
            << mapStack->at(mapIdx).rows << "_" << "cols_"
            << mapStack->at(mapIdx).cols << "_" << ".bin";

        ROS_DEBUG(
            "Store map to: %s\n"
            "With format: %s\n",
            oss.str().c_str(),
            utils::conversion::format2str(mapStack->at(mapIdx).type()).c_str());

        // Store the layer
        std::ofstream f;
        f.open(oss.str(), std::ofstream::out | std::ofstream::binary);
        if (f.is_open()) {
          f.write(
              (char*) mapStack->at(mapIdx).data,
              mapStack->at(mapIdx).rows * mapStack->at(mapIdx).cols
                  * utils::conversion::type2size(mapStack->at(mapIdx).type()));
          f.close();
        } else {
          ROS_ERROR("Unable to open file %s\n", oss.str().c_str());
        }
      }
    }

    // Shift the map
    if (shiftMapStack) {
      // Calculate the shift as indices
      ROS_DEBUG("World (x,y,z): %f, %f, %f",
                transformRoiInWorld.getOrigin().x(),
                transformRoiInWorld.getOrigin().y(),
                transformRoiInWorld.getOrigin().z());
      ROS_DEBUG("Last (x,y,z): %f, %f, %f",
                transformRoiInWorldLast.getOrigin().x(),
                transformRoiInWorldLast.getOrigin().y(),
                transformRoiInWorldLast.getOrigin().z());

      const double xshift_tiles = -(transformRoiInWorldLast.getOrigin().x()
          - transformRoiInWorld.getOrigin().x()) / resolution_meterPerTile;
      const double yshift_tiles = -(transformRoiInWorldLast.getOrigin().y()
          - transformRoiInWorld.getOrigin().y()) / resolution_meterPerTile;

      ROS_DEBUG("Shift of the map measured in tiles: x= %d, y= %d",
                int(xshift_tiles), int(yshift_tiles));

      for (std::size_t idx = 0; idx < mapStack->size(); ++idx) {
        translateMap(mapStack->at(idx), mapStackShiftedResult->at(idx),
                     xshift_tiles, yshift_tiles, fillValue);
      }

    }

    // Clear the map
    if (clearMapStack) {
      ROS_DEBUG("Clear the map");
      for (std::size_t idx = 0; idx < mapStack->size(); ++idx) {
        mapStack->at(idx).setTo(fillValue);
      }
    }

    transformRoiInWorldLast = transformRoiInWorld;  // Store the new location for the next time

  }

  ///
  /// \brief Transformes the received laser messages into the map frame and stores them in the next free map layer
  /// \param scanMsg Laser message to process
  ///
  void dataHandler(const sensor_msgs::LaserScan scanMsg) {

    sensor_msgs::PointCloud cloud, cloudRoi;  // Pointclouds for transformation

    mtxSwap.lock();
    const std::string frameTarget(
        currentTileTfName + tileOriginTfSufixForRoiOrigin);
    std::shared_ptr<std::vector<cv::Mat>> mapHeight_mm(currentMapHeight_mm);
    std::shared_ptr<std::vector<cv::Mat>> pulsewidth_ps(currentPulsewidth_ps);
    std::shared_ptr<std::vector<cv::Mat>> mapIterator(currentMapIterator);
    mtxSwap.unlock();

    // Handle the scan data in the ROI
    projector.projectLaser(scanMsg, cloud, -1.0,
                           laser_geometry::channel_option::Intensity);
    try {
      listenerTf->waitForTransform(frameTarget, cloud.header.frame_id,
                                   cloud.header.stamp, ros::Duration(0.3));
      listenerTf->transformPointCloud(frameTarget, cloud, cloudRoi);
    } catch (const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_WARN("%s, frameTarget:%s, cloud.header.frame_id:%s\n", excStr.c_str(),
               frameTarget.c_str(), cloud.header.frame_id.c_str());
      return;
    }

    for (std::size_t idx = 0; idx < cloudRoi.points.size(); ++idx) {

      // Skip if value if faulty (not needed because of projectLaser behavior)
      //    if (scanMsg.ranges.at(idx) > scanMsg.range_max || scanMsg.ranges.at(idx) < scanMsg.range_min ) {
      //      ROS_WARN("Skip value out of range (min < val < max): (%f < %f < %f)", scanMsg.range_min, scanMsg.ranges.at(idx), scanMsg.range_max);
      //      continue;
      //    }
      // Skip if value is out of the map
      if (cloudRoi.points.at(idx).x > mapSizeX_m
          || cloudRoi.points.at(idx).x < 0.0f
          || cloudRoi.points.at(idx).y > mapSizeY_m
          || cloudRoi.points.at(idx).y < 0.0f) {
        ROS_DEBUG("Skip value out of map");
        continue;
      }

      // Assigne the data
      const int x_px = static_cast<int>(floor(
          cloudRoi.points.at(idx).x / resolution_mPerTile));
      const int y_px = static_cast<int>(floor(
          cloudRoi.points.at(idx).y / resolution_mPerTile));
      const double z = cloudRoi.points.at(idx).z;  // The height lies in the z-axis of the ROI frame

      mapHeight_mm->at(mapIterator->at(0).at<uint8_t>(y_px, x_px)).at<int16_t>(
          y_px, x_px) = static_cast<int16_t>(z * constants::geometry::millimeterPerMeter);
      pulsewidth_ps->at(mapIterator->at(0).at<uint8_t>(y_px, x_px)).at<int16_t>(
          y_px, x_px) = static_cast<int16_t>(int16_t(
          cloudRoi.channels.at(0).values.at(idx)));
      ++mapIterator->at(0).at<uint8_t>(y_px, x_px);
      // Reset the layer counter if necessary
      if (mapIterator->at(0).at<uint8_t>(y_px, x_px) >= maxLayer) {
        mapIterator->at(0).at<uint8_t>(y_px, x_px) = 0;
      }
    }
  }

  ///////////////////////////////////////////SERVER///////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  enum statistics {
    mean,
    variance,
    quantil,
    quantil90,
    quantil95
  };

  ///
  /// \brief Calculates the requested statistic measure out of the map layer stack
  /// \param src Map layer stack
  /// \param dst Processed statistic as a single layer
  /// \param statisticReq The statistics which are stored into the return value
  /// \param quantil The quantil measure from 1 to 99 which is processed if quantil is requested
  ///
  void getStatistic(const std::vector<cv::Mat> &src, cv::Mat &dst,
                    const statistics statisticReq,
                    const uint8_t quantil = 0/*1 .. 99 %*/) {

    // Get the mean if necessary
    cv::Mat mean;
    if (statisticReq == statistics::variance) {
      getStatistic(src, mean, statistics::mean);
    } else if (statisticReq == statistics::mean) {
      // Go on
    } else if (statisticReq == statistics::quantil) {
      if (quantil < 1 || quantil > 99) {
        throw std::runtime_error(
            std::string("getStatistic: Unsupported quantil"));
      }
      getStatistic(src, mean, statistics::mean);
    } else {
      throw std::runtime_error(
          std::string("getStatistic: Unsupported statistic type"));
    }

    cv::Mat dstTmp = cv::Mat(src.at(0).rows, src.at(0).cols,
    CV_32SC1,
                             cv::Scalar_<int32_t>(0));

    dst = cv::Mat(src.at(0).rows, src.at(0).cols, src.at(0).type());

    cv::Mat validCounter = cv::Mat(src.at(0).rows, src.at(0).cols,
    CV_32SC1,
                                   cv::Scalar_<int32_t>(0));

    // The only thing that changes, is the calculation of dstTmp, but for speed reasons,
    // we spare the condition inside the loop and do it outside
    if (statisticReq == statistics::variance
        || statisticReq == statistics::quantil) {
      for (std::size_t idxMap = 0; idxMap < src.size(); ++idxMap) {
        for (int idxTile = 0;
            idxTile < src.at(idxMap).rows * src.at(idxMap).cols; ++idxTile) {
          const int16_t currentVal = src.at(idxMap).at<int16_t>(idxTile);
          if (currentVal != numerics::invalidValue_int16) {
            dstTmp.at<int32_t>(idxTile) += pow(
                int32_t(currentVal) - int32_t(mean.at<int16_t>(idxTile)), 2);
            ++validCounter.at<int32_t>(idxTile);
          }
        }
      }
    } else if (statisticReq == statistics::mean) {
      for (std::size_t idxMap = 0; idxMap < src.size(); ++idxMap) {
        for (int idxTile = 0;
            idxTile < src.at(idxMap).rows * src.at(idxMap).cols; ++idxTile) {
          const int16_t currentVal = src.at(idxMap).at<int16_t>(idxTile);
          if (currentVal != numerics::invalidValue_int16) {
            dstTmp.at<int32_t>(idxTile) += int32_t(currentVal);
            ++validCounter.at<int32_t>(idxTile);
          }
        }
      }
    }

    // Normalize
    for (int idxTile = 0; idxTile < src.at(0).rows * src.at(0).cols;
        ++idxTile) {
      if (validCounter.at<int32_t>(idxTile) != 0) {
        dstTmp.at<int32_t>(idxTile) /= validCounter.at<int32_t>(idxTile);
      } else {
        dstTmp.at<int32_t>(idxTile) = numerics::invalidValue_int32;
      }
    }

    // Calc the quantil assuming a standard deviation: http://matheguru.com/stochastik/31-normalverteilung.html
    if (statisticReq == statistics::quantil) {
      // Get the argument of for the inverse error function
      const float z = (static_cast<float>(quantil) / 100.0) * 2.0 - 1;
      const float erf_inv_z = boost::math::erf_inv<float>(z);
      // Get the quantil for every tile (dstTmp == sigma², mean == µ)
      for (int idxTile = 0; idxTile < dstTmp.rows * dstTmp.cols; ++idxTile) {
        const float mu = static_cast<float>(mean.at<int16_t>(idxTile));
        const float var = static_cast<float>(dstTmp.at<int32_t>(idxTile));
        dstTmp.at<int32_t>(idxTile) = static_cast<int32_t>(erf_inv_z
            * sqrt(2.0 * var) + mu);
      }
    }

    // Copy back the result
    for (int idxTile = 0; idxTile < src.at(0).rows * src.at(0).cols;
        ++idxTile) {
      dst.at<int16_t>(idxTile) = static_cast<int16_t>(dstTmp.at<int32_t>(
          idxTile));
    }
  }

  ///
  /// \brief Cuts the requested view out of the map stack and processes the statistics (Return the cropped statistic view of a std::vector<cv::Mat_<int16_t>> mapStack as cv::Mat_<float>)
  /// \param mapStack The map layer stack
  /// \param xView Horizontal translation in meter
  /// \param yView Lateral translation in meter
  /// \param wView Width of the requested view in meter (expansion along xView)
  /// \param dView Depth of the requested view in meter (expansion along yView)
  /// \param zRotView Rotation around the vertical axis in rad
  /// \param stat The statistics to be calculated out of the map layer stack
  /// \param targetFrame Target frame of the request
  /// \param sourceFrame Source frame of the request
  /// \return The requested and calculated view as a rectangle
  ///
  cv::Mat getStatisticView(const std::shared_ptr<std::vector<cv::Mat>> mapStack,
                           const double xView/*x*/, const double yView/*y*/,
                           const double wView/*w*/, const double dView/*h*/,
                           const double zRotView /*rotZ*/,
                           const statistics stat, const std::string targetFrame,
                           const std::string sourceFrame) {
    // Get the current data
    std::vector<cv::Mat> tmpMapStack(mapStack->size());

    tf::StampedTransform transformedViewInRoi;
    try {
      listenerTf->waitForTransform(targetFrame, sourceFrame, ros::Time(0),
                                   ros::Duration(3.0));
      listenerTf->lookupTransform(targetFrame, sourceFrame, ros::Time(0),
                                  transformedViewInRoi);
      ROS_DEBUG("getStatisticView: abs(x,y,z): %f",
                transformedViewInRoi.getOrigin().length());
    } catch (const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("%s", excStr.c_str());
      throw std::runtime_error(
          std::string(
              "doLaseTf: Won't perform any tf without valid machine model"));
    }

    for (std::size_t idx = 0; idx < mapStack->size(); ++idx) {
      mapStack->at(idx).copyTo(tmpMapStack.at(idx));
    }

    cv::Mat mapStackStatistic;  // Going to be int16_t in every cell
    switch (stat) {
      case statistics::mean:
        getStatistic(tmpMapStack, mapStackStatistic, statistics::mean);
        break;
      case statistics::variance:
        getStatistic(tmpMapStack, mapStackStatistic, statistics::variance);
        break;
      case statistics::quantil90:
        getStatistic(tmpMapStack, mapStackStatistic, statistics::quantil, 90);
        break;
      case statistics::quantil95:
        getStatistic(tmpMapStack, mapStackStatistic, statistics::quantil, 95);
        break;
      default:
        throw std::runtime_error(
            std::string("getStatisticView: Unsupported statistic type"));
        break;
    }

    // Get the odometry TF in the roi frame
    tf::Matrix3x3 m(transformedViewInRoi.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Eigen::Matrix4d roi_machineRoi = ctf::trans<double>(
        transformedViewInRoi.getOrigin().getX(),
        transformedViewInRoi.getOrigin().getY(), 0.0) * ctf::rotZ<double>(yaw);
    // Cut the image
    cv::Mat dst;
    rect = utils::cutView(mapStackStatistic, dst,
                          resolution_mPerTile /*m/px*/, xView/*x*/,
                          yView/*y*/, wView/*w*/, dView/*d*/, zRotView /*rotZ*/,
                          roi_machineRoi /*roi odometry*/,
                          mapStackStatistic.type());
    if (debug) {
      mtxShowRpc.lock();
      dst.copyTo(mapStackStatisticRequestDebug);
      mapStackStatistic.copyTo(mapStackStatisticDebug);
      mtxShowRpc.unlock();
    }

    return dst;
  }

  ///
  /// \brief Wrapper function for getStatisticView() which resizes the answer to the requested resolution
  /// \param view Request
  /// \param output Answer
  /// \param input Map layer stack from where the answer should be processed
  /// \param stat The statistic measure to calculate out of the map layer stack
  /// \param outputDimension An informative string describing the units stored in the answer
  ///
  void getStatisticOutput(const mapserver_msgs::mapPrimitive &view,
                          mapserver_msgs::rsm &output,
                          const std::shared_ptr<std::vector<cv::Mat>> &input,
                          const statistics stat,
                          const std::string &outputDimension) {

    cv::Mat statisticMat, croppedMat;

    std::stringstream os;
    os << view;
    ROS_INFO("view:\n%s\n", os.str().c_str());

    mtxSwap.lock();
    const std::shared_ptr<std::vector<cv::Mat>> mapHeight_mm = input;
    const std::string targetframe(currentTileTfName);
    mtxSwap.unlock();

    // Calculate the requested ROI
    const double xView = view.pose.pose.position.x;  // Meter
    const double yView = view.pose.pose.position.y;  // Meter
    const double wView = view.width * view.resolution;  // Tiles * meter/tiles
    const double dView = view.depth * view.resolution;  // Tiles * meter/tiles
    const std::string sourceframe =
        view.frame_id.empty() ?
            machine::frames::names::BASE_LINK : view.frame_id;
    tf::Quaternion q;
    tf::quaternionMsgToTF(view.pose.pose.orientation, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get the cropped statistic with respect to view
    output.header.stamp = ros::Time::now();
    statisticMat = getStatisticView(input, xView/*x*/, yView/*y*/, wView/*w*/,
                                    dView/*d*/, yaw /*rotZ*/, stat, targetframe,
                                    sourceframe);

    // Resize the resolution
    cv::Size size(view.width, view.depth);
    cv::resize(statisticMat, croppedMat, size, 0, 0, cv::INTER_NEAREST);

    // Copy the meta information
    output.header.frame_id = sourceframe;
    output.cols = croppedMat.cols;
    output.rows = croppedMat.rows;
    output.resolution = view.resolution;
    output.unit = outputDimension;
    const int arraySize = croppedMat.cols * croppedMat.rows;
    output.map.resize(arraySize);

    // Copy the payload
    cv::Mat croppedMatConverted;
    if (croppedMat.type() != CV_32FC1) {
      croppedMat.convertTo(croppedMatConverted, CV_32FC1);
    } else {
      croppedMatConverted = croppedMat;
    }
    // TODO replace by memcpy if memory is not fragmented
    for (int idx = 0; idx < arraySize; ++idx) {
      output.map.at(idx) = croppedMatConverted.at<float>(idx);
    }
  }

  bool mapRawServerMeanHeight(mapserver::rsm::Request &req,
                              mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-meanHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::mean, std::string("mm"));
    ROS_INFO("rawMapserver-meanHeight method called: Finish");
    return true;
  }

  bool mapRawServerVarianceHeight(mapserver::rsm::Request &req,
                                  mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::variance, std::string("mm^2"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil90Height(mapserver::rsm::Request &req,
                                   mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::quantil90, std::string("mm"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil95Height(mapserver::rsm::Request &req,
                                   mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentMapHeight_mm,
                       statistics::quantil95, std::string("mm"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerMeanPulsewidth(mapserver::rsm::Request &req,
                                  mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::mean, std::string("ps"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerVariancePulsewidth(mapserver::rsm::Request &req,
                                      mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::variance, std::string("ps^2"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil90Pulsewidth(mapserver::rsm::Request &req,
                                       mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::quantil90, std::string("ps"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  bool mapRawServerQuantil95Pulsewidth(mapserver::rsm::Request &req,
                                       mapserver::rsm::Response &res) {
    ROS_INFO("rawMapserver-varianceHeight method called");
    getStatisticOutput(req.request, res.response, currentPulsewidth_ps,
                       statistics::quantil95, std::string("ps"));
    ROS_INFO("rawMapserver-varianceHeight method called: Finish");
    return true;
  }

  void sendMap(void) {
    if (sendTopLayerOfDistanceAsOgm) {
      // Send the top layer as occupancy grid map
      mtxSwap.lock();
      const std::string frameTarget(currentTileTfName);
      std::shared_ptr<std::vector<cv::Mat>> mapHeight_mm = currentMapHeight_mm;
      mtxSwap.unlock();
      nav_msgs::OccupancyGrid ogm;
      ogm.header.frame_id = frameTarget + std::string("_")
          + machine::frames::names::ROI_ORIGIN;
      ogm.info.height = mapSizeY;
      ogm.info.width = mapSizeX;
      ogm.info.resolution = resolution_mPerTile;
      ogm.data.resize(mapSizeY * mapSizeX, 0);
      // HACK Needs parametrization for display: look at 1e-1
      for (int idx = 0; idx < ogm.data.size(); ++idx) {
        ogm.data.at(idx) = int8_t(
            float(mapHeight_mm->at(0).at<int16_t>(idx)) * 0.1f);
        //       std::cerr << ogm.data.at(idx) << " ";
        //       std::cerr << currentMapHeight_mm->at(0).at<int16_t>(idx) << " ";
      }
      //     std::cerr << std::endl;
      publisherMap.publish(ogm);
    }
  }

  ///
  /// \brief Store the current tf tile name
  /// \param nameMsg Name of the current tile tf
  ///
  void tfTileNameHandler(const std_msgs::String nameMsg) {
    bool currentTileTfNameChange = false;
    mtxSwap.lock();
    if ((nameMsg.data.back() != currentTileTfName.back())) {
      if (currentTileTfName.empty()) {
        // First round, we bootstrap
        currentTileTfName = nameMsg.data;
      } else {
        std::swap(currentMapHeight_mm, lastMapHeight_mm);
        std::swap(currentPulsewidth_ps, lastPulsewidth_ps);
        std::swap(currentMapIterator, lastMapIterator);
        lastTileTfName = currentTileTfName;
        currentTileTfName = nameMsg.data;
        currentTileTfNameChange = true;
      }
    }

    if (currentTileTfNameChange) {
      tf::StampedTransform transformRoiInWorld;
      try {
        listenerTf->waitForTransform(lastTileTfName, worldLink, ros::Time(0.0),
                                     ros::Duration(3.0));
        listenerTf->lookupTransform(lastTileTfName, worldLink, ros::Time(0.0),
                                    transformRoiInWorld);
      } catch (const std::exception &exc) {
        const std::string excStr(exc.what());
        ROS_ERROR("tfTileNameHandler: %s", excStr.c_str());
        mtxSwap.unlock();
        return;
      }
      ROS_DEBUG("NEW MAP");

      // Wait until all references are gone
      std::size_t lockCnt = 0;
      const std::size_t lockCntMax = 200000;  // 2 seconds if we sleep for 10 us

      const bool clearMap = !shiftMap;
      while ((!(lastMapHeight_mm.unique() && lastPulsewidth_ps.unique()
          && lastMapIterator.unique()) && !clearMap)  // The last map is hold by another process, but we don't care if we don't clear it
          || !(currentMapHeight_mm.unique() && currentPulsewidth_ps.unique()
              && currentMapIterator.unique()) && !bool(shiftMap)) {  // The current map needs to be filled with the old values if shifted, but we don't care if we don't shift it
        usleep(10);
        if (++lockCnt > lockCntMax) {
          ROS_ERROR(
              "tfTileNameHandler: Locked for to long, skip storage (maybe deadlock or out if resources?)");
          mtxSwap.unlock();
          return;
        }
      }

      mapRefreshAndStorage(lastMapHeight_mm, currentMapHeight_mm,
                           transformRoiInWorld, std::string("height"),
                           std::string(""), std::string("mm"),
                           resolution_mPerTile, !dontStoreMaps,
                           bool(shiftMap), clearMap,
                           numerics::invalidValue_int16);
      mapRefreshAndStorage(lastPulsewidth_ps,        // Map to shift/store/reset
          currentPulsewidth_ps,                 // The result of the shifted map
          transformRoiInWorld,                   // Transform
          std::string("pulsewidth"),             // Kind of map
          std::string(""),           // Format (empty: Take from type specifier)
          std::string("ps"),                     // Unit
          resolution_mPerTile,               // Tile resolution
          !dontStoreMaps,                       // Info if maps should be stored
          bool(shiftMap),                      // Info if maps should be shifted
          clearMap,      // If map is not shifted, reset the content of mapStack
          numerics::invalidValue_int16);     // Fill-up value
      mapRefreshAndStorage(lastMapIterator,          // Map to shift/store/reset
          currentMapIterator,                   // The result of the shifted map
          transformRoiInWorld,                   // Transform
          std::string("mapiterator"),            // Kind of map
          std::string(""),           // Format (empty: Take from type specifier)
          std::string("1"),                      // Unit
          resolution_mPerTile,               // Tile resolution
          false,                                // Info if maps should be stored
          bool(shiftMap),                      // Info if maps should be shifted
          clearMap,      // If map is not shifted, reset the content of mapStack
          0);                                    // Fill-up value
    }

    mtxSwap.unlock();

  }

  ///
  /// \brief Shows the current RPC if debug is on
  ///
  void showRpc() {
    if (debug) {
      ROS_INFO("DEBUG: Show the requests");
      if (mapStackStatisticDebug.empty() || mapStackStatisticDebug.empty()) {
        ROS_ERROR("mapStackStatistic is empty");
        return;
      }
      cv::Mat mapStackStatisticRequestDebugTmp, mapStackStatisticDebugTmp;
      mtxShowRpc.lock();
      mapStackStatisticRequestDebug.copyTo(mapStackStatisticRequestDebugTmp);
      mapStackStatisticDebug.copyTo(mapStackStatisticDebugTmp);
      mapStackStatisticDebugTmp.release();
      mapStackStatisticRequestDebugTmp.release();
      cv::RotatedRect rectTmp = rect;
      mtxShowRpc.unlock();

      std::stringstream os;
      os << ros::Time::now();
      {
        cv::imshow(std::string("mapStackStatisticRequest"),
                   mapStackStatisticRequestDebugTmp);
        cv::setWindowTitle(
            std::string("mapStackStatisticRequest"),
            std::string("mapStackStatisticRequest at ") + os.str());
      }
      {
        // Draw the request in pseudo-colors
        if (debugDrawRpc) {
          // TODO Make a cast which is possible to handle all data types
          utils::castCopyImage(mapStackStatisticDebugTmp,
                               mapStackStatisticDebugTmp, CV_16UC1);
          mapStackStatisticDebugTmp.convertTo(mapStackStatisticDebugTmp,
                                              CV_8UC3);
          cv::cvtColor(mapStackStatisticDebugTmp, mapStackStatisticDebugTmp,
                       CV_GRAY2BGR);
          utils::drawRotatedRectInImage(mapStackStatisticDebugTmp, rect,
                                        cv::Scalar(0, 0, 255));
        }

        cv::imshow(std::string("mapStackStatistic"), mapStackStatisticDebugTmp);
        cv::setWindowTitle(std::string("mapStackStatistic"),
                           std::string("mapStackStatistic at ") + os.str());
      }
      cv::waitKey(1);
    }
  }

  void spinOnce() {
    sendMap();
    showRpc();
  }

};

MapserverRaw::MapserverRaw(ros::NodeHandle &nh)
    : Mapserver(&nh),
      n(nh) {

  n.param<std::string>("topic_map", topicMap, "/map");
  n.param<std::string>("topic_laser", topicLaser, scopes::root);
  n.param<float>("max_distance_insertion_m", maxDistanceInsertion_m,
                 std::min(mapping::roi::width, mapping::roi::height));
  n.param<int>("max_layer", maxLayer, 20);
  n.param<int>("send_top_layer_of_distancestack_as_ogm",
               sendTopLayerOfDistanceAsOgm, 1);
  n.param<int>("debug_draw_rpc_in_view", debugDrawRpc, 0);

  publisherMap = n.advertise<nav_msgs::OccupancyGrid>(topicMap, 1);

  subscriberLaser = n.subscribe<sensor_msgs::LaserScan>(topicLaser, 100,
                                                        &MapserverRaw::dataHandler, this);
  subscriberTfTileName = n.subscribe<std_msgs::String>(currentTfNameTopic, 2,
                                                       &MapserverRaw::tfTileNameHandler, this);

  // Allocate space for the maps and init the double-buffer
  storageMapBuffer = cv::Mat(mapSizeX, mapSizeY, CV_8UC1);
  cv::Mat tmpTopLayerHeight = cv::Mat(mapSizeX, mapSizeY, CV_16SC1);

  {
    // Initiate the data for the maps
    cv::Mat mapInit(mapSizeX, mapSizeX, CV_16SC1,
                    cv::Scalar_<int16_t>(numerics::invalidValue_int16));
    cv::Mat iteratorInit(mapSizeX, mapSizeX, CV_8UC1, cv::Scalar_<uint8_t>(0));

    // Allocate memory for the maps
    currentMapHeight_mm = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    lastMapHeight_mm = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    currentPulsewidth_ps = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    lastPulsewidth_ps = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(maxLayer));
    currentMapIterator = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(1));
    lastMapIterator = std::shared_ptr<std::vector<cv::Mat>>(
        new std::vector<cv::Mat>(1));

    // Copy the initiated date to the map stacks
    for (std::size_t idx = 0; idx < maxLayer; ++idx) {
      mapInit.copyTo(currentMapHeight_mm->at(idx));
      mapInit.copyTo(lastMapHeight_mm->at(idx));
      mapInit.copyTo(currentPulsewidth_ps->at(idx));
      mapInit.copyTo(lastPulsewidth_ps->at(idx));
      ROS_DEBUG_STREAM(" Pointer: " << (int* )currentMapHeight_mm->at(idx).data);
    }
    iteratorInit.copyTo(currentMapIterator->at(0));
    iteratorInit.copyTo(lastMapIterator->at(0));
  }

  // Server
  // TODO Make request and scope changebale via program options concerning DisplayRoi
  const std::string s("/");
  service_meanHeight = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::meanHeight,
          &MapserverRaw::mapRawServerMeanHeight, this);
  service_varianceHeight = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::varianceHeight,
          &MapserverRaw::mapRawServerVarianceHeight, this);
  service_quantil90Height = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil90Height,
          &MapserverRaw::mapRawServerQuantil90Height, this);
  service_quantil95Height = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil95Height,
          &MapserverRaw::mapRawServerQuantil95Height, this);
  service_meanPulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::meanPulsewidth,
          &MapserverRaw::mapRawServerMeanPulsewidth, this);
  service_variancePulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::variancePulsewidth,
          &MapserverRaw::mapRawServerVariancePulsewidth, this);
  service_quantil90Pulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil90Pulsewidth,
          &MapserverRaw::mapRawServerQuantil90Pulsewidth, this);
  service_quantil95Pulsewidth = n.advertiseService(
      scopes::map::rawServer::parent + s
          + scopes::map::rawServer::requests::quantil95Pulsewidth,
          &MapserverRaw::mapRawServerQuantil95Pulsewidth, this);

}
