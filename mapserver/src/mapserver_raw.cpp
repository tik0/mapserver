#include <mapserver_raw.hpp>

cv::Mat MapserverRaw::getStatisticView(
    const std::shared_ptr<std::vector<cv::Mat>> mapStack,
    const double xView/*x*/, const double yView/*y*/, const double wView/*w*/,
    const double dView/*h*/, const double zRotView /*rotZ*/,
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
  rect = utils::cutView(mapStackStatistic, dst, resolution_mPerTile /*m/px*/,
                        xView/*x*/, yView/*y*/, wView/*w*/, dView/*d*/,
                        zRotView /*rotZ*/, roi_machineRoi /*roi odometry*/,
                        mapStackStatistic.type());
  if (debug) {
    mtxShowRpc.lock();
    dst.copyTo(mapStackStatisticRequestDebug);
    mapStackStatistic.copyTo(mapStackStatisticDebug);
    mtxShowRpc.unlock();
  }

  return dst;
}

void MapserverRaw::getStatisticOutput(
    const mapserver_msgs::mapPrimitive &view, mapserver_msgs::rsm &output,
    const std::shared_ptr<std::vector<cv::Mat>> &input, const statistics stat,
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
      view.frame_id.empty() ? machine::frames::names::BASE_LINK : view.frame_id;
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

void MapserverRaw::getStatistic(const std::vector<cv::Mat> &src, cv::Mat &dst,
                  const statistics statisticReq,
                  const uint8_t quantil) {

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
      for (int idxTile = 0; idxTile < src.at(idxMap).rows * src.at(idxMap).cols;
          ++idxTile) {
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
      for (int idxTile = 0; idxTile < src.at(idxMap).rows * src.at(idxMap).cols;
          ++idxTile) {
        const int16_t currentVal = src.at(idxMap).at<int16_t>(idxTile);
        if (currentVal != numerics::invalidValue_int16) {
          dstTmp.at<int32_t>(idxTile) += int32_t(currentVal);
          ++validCounter.at<int32_t>(idxTile);
        }
      }
    }
  }

  // Normalize
  for (int idxTile = 0; idxTile < src.at(0).rows * src.at(0).cols; ++idxTile) {
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
  for (int idxTile = 0; idxTile < src.at(0).rows * src.at(0).cols; ++idxTile) {
    dst.at<int16_t>(idxTile) =
        static_cast<int16_t>(dstTmp.at<int32_t>(idxTile));
  }
}

void MapserverRaw::laserDataHandler(const sensor_msgs::LaserScan::ConstPtr scanMsg) {
  sensor_msgs::PointCloud cloud;
  projector.projectLaser(*scanMsg, cloud, -1.0,
                         laser_geometry::channel_option::Intensity);
  this->dataHandler(boost::make_shared<sensor_msgs::PointCloud>(cloud));
}

void MapserverRaw::dataHandler(const sensor_msgs::PointCloud::Ptr cloud) {

  sensor_msgs::PointCloud cloudRoi;  // Point cloud in ROI

  mtxSwap.lock();
    const std::string frameTarget(
        currentTileTfName + tileOriginTfSufixForRoiOrigin);
    std::shared_ptr<std::vector<cv::Mat>> mapHeight_mm(currentMapHeight_mm);
    std::shared_ptr<std::vector<cv::Mat>> pulsewidth_ps(currentPulsewidth_ps);
    std::shared_ptr<std::vector<cv::Mat>> mapIterator(currentMapIterator);
  mtxSwap.unlock();

  // Handle the point cloud in the ROI
  try {
    listenerTf->waitForTransform(frameTarget, cloud->header.frame_id,
                                 cloud->header.stamp, ros::Duration(0.3));
    listenerTf->transformPointCloud(frameTarget, *cloud, cloudRoi);
  } catch (const std::exception &exc) {
    const std::string excStr(exc.what());
    ROS_WARN("%s, frameTarget:%s, cloud.header.frame_id:%s\n", excStr.c_str(),
             frameTarget.c_str(), cloud->header.frame_id.c_str());
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
        y_px, x_px) = static_cast<int16_t>(z
        * constants::geometry::millimeterPerMeter);
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

template<typename T>
void MapserverRaw::mapRefreshAndStorage(
    std::shared_ptr<std::vector<cv::Mat>> mapStack,
    std::shared_ptr<std::vector<cv::Mat>> mapStackShiftedResult,
    const tf::StampedTransform transformRoiInWorld,
    const std::string prefixString, const std::string formatString,
    const std::string formatUnitString, const double resolution_meterPerTile,
    const bool storeMapStack, const bool shiftMapStack,
    const bool clearMapStack, T fillValue) {

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
          << "Unit_" << formatUnitString << "_" << "LayerIdx_" << mapIdx << "_"
          << "GridRes_"
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
    ROS_DEBUG("World (x,y,z): %f, %f, %f", transformRoiInWorld.getOrigin().x(),
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

template<typename T>
void MapserverRaw::translateMap(cv::Mat &src, cv::Mat &dst, double offsetx,
                  double offsety,
                  T fillValue) {

  // Define a transformation for the image
  const cv::Mat trans_mat =
      (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);

  // Warp the image (which refers to the map)
  cv::warpAffine(src, dst, trans_mat, cv::Size(src.rows, src.cols),
                 cv::INTER_NEAREST, cv::BORDER_CONSTANT,
                 cv::Scalar(static_cast<double>(fillValue)));

}
