#include <mapserver_raw.hpp>

cv::Mat MapserverRaw::getStatisticView(
      const std::shared_ptr<std::vector<cv::Mat>> mapStack,
      const statistics stat, const std::string targetFrame_id,
      const mapserver_msgs::mapPrimitive &view) {
  // Get the current data
  std::vector<cv::Mat> tmpMapStack(mapStack->size());

  // Check if we have to perform the heavy processing
  try {
    listenerTf->waitForTransform(targetFrame_id, view.header.frame_id, view.header.stamp,
                                 ros::Duration(3.0));
  } catch (const std::exception &exc) {
    throw std::runtime_error(
        std::string(
            "getStatisticView: Won't perform without valid machine model"));
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

  cv::Mat dst;
  rect = this->cutView(mapStackStatistic, dst, resolution_mPerTile, targetFrame_id, view, mapStackStatistic.type());

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
  const std::string targetframe_id(currentTileTfName + tileOriginTfSufixForRoiOrigin);
  mtxSwap.unlock();

  // Get the cropped statistic with respect to view
  statisticMat = getStatisticView(input, stat, targetframe_id, view);

  // Resize the resolution
  resizeView(statisticMat, croppedMat, resolution_mPerTile, view.resolution);

  // Copy the meta information
  output.header.stamp = view.header.stamp;
  output.header.frame_id = view.header.frame_id;
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
    const bool clearMapStack, T fillValue, bool storeCurrentPosition /*unused*/,
    std::string additionalInformationString, ros::Time storageTime) {

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
          << mapStack->at(mapIdx).cols << "_"
          << additionalInformationString << "_" << ".bin";

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

void MapserverRaw::tfTileNameHandler(const std_msgs::String nameMsg) {
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
                           resolution_mPerTile, !dontStoreMaps, bool(shiftMap),
                           clearMap, numerics::invalidValue_int16);
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

void MapserverRaw::tupleHandler(const mapserver_msgs::pnsTuple msg) {
  bool currentTileTfNameChange = false;
  static mapserver_msgs::pnsTuple lastPnsTuple;
  mapRefresh.lock();
  if ((msg.string.data.back() != currentTileTfName.back())) {
    if (currentTileTfName.empty()) {
      // First round, we bootstrap
      currentTileTfName = msg.string.data;
      lastPnsTuple = msg;
    } else {
      std::swap(currentMapHeight_mm, lastMapHeight_mm);
      std::swap(currentPulsewidth_ps, lastPulsewidth_ps);
      std::swap(currentMapIterator, lastMapIterator);
      lastTileTfName = currentTileTfName;
      currentTileTfName = msg.string.data;
      currentTileTfNameChange = true;
    }
  }

  if (currentTileTfNameChange) {
    ROS_INFO("NEW MAP");
    tf::StampedTransform transformRoiInWorld;
    transformRoiInWorld.setOrigin(
        tf::Vector3(msg.point.x, msg.point.y, msg.point.z));
    transformRoiInWorld.setRotation(tf::Quaternion(0, 0, 0, 1));

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

    // Format the global location string
    std::stringstream navSatSs;
    navSatSs << std::setprecision(12) << "lat_"
             << lastPnsTuple.navsat.latitude << "_" << "lon_"
             << lastPnsTuple.navsat.longitude << "_" << "alt_"
             << lastPnsTuple.navsat.altitude;

    mapRefreshAndStorage(lastMapHeight_mm, currentMapHeight_mm,
                         transformRoiInWorld, std::string("height"),
                         std::string(""), std::string("mm"),
                         resolution_mPerTile, !dontStoreMaps, bool(shiftMap),
                         clearMap, numerics::invalidValue_int16, true, navSatSs.str());
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
        numerics::invalidValue_int16,      // Fill-up value
        true, navSatSs.str());
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
        0,                                     // Fill-up value
        true, navSatSs.str());
  }

  mtxSwap.unlock();
}
