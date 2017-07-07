#include <mapserver_stat.hpp>

// Add just some picture of each other
boost::shared_ptr<cv::Mat> MapserverStat::doColorMapCallback(
    std::vector<mrpt::maps::COccupancyGridMap2D> mapStack) {

  // TODO Define this as a global variable so that it dows not
  // have to allocate space on every call
  boost::shared_ptr<cv::Mat> dst(new cv::Mat(mapSizeX, mapSizeY, CV_8UC3));
  dst->setTo(cv::Scalar(127, 127, 127));  // to set all values to 127 (aka unknown)

  const float validCellValue = uncertaintyBoundary;
  for (uint mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    for (int idy = 0; idy < mapSizeY; ++idy) {
      for (int idx = 0; idx < mapSizeX; ++idx) {
        if (mapStack[mapIdx].getCell(idx, idy) > validCellValue) {
          dst->at<cv::Vec3b>(idy, idx)[0] =
              mappingLayers::mapColorBGR[mapIdx][0];
          dst->at<cv::Vec3b>(idy, idx)[1] =
              mappingLayers::mapColorBGR[mapIdx][1];
          dst->at<cv::Vec3b>(idy, idx)[2] =
              mappingLayers::mapColorBGR[mapIdx][2];
        }
      }
    }
  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
}

std::shared_ptr<cv::Mat> MapserverStat::mrptOggToGrayScale(
    mrpt::maps::COccupancyGridMap2D &map) {

  std::shared_ptr<cv::Mat> dst(
      new cv::Mat(map.getSizeY(), map.getSizeX(), CV_8UC1));
//  dst->setTo(cv::Scalar(127,127,127)); // to set all values to 127 (aka unknown)

  for (int idx = 0; idx < map.getRawMap().size(); ++idx) {
    dst->at<uchar>(idx) = mrpt::maps::COccupancyGridMap2D::l2p_255(
        map.getRawMap().at(idx));
  }
//  for (int idy = 0; idy < map.getSizeY(); ++idy) {
//    for (int idx = 0; idx < map.getSizeX(); ++idx) {
//      const uchar intensity = uchar(map.getCell(idx, idy) * 255);
//      dst->at<uchar>(idx,idy) = intensity;
//    }
//  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
}

std::shared_ptr<std::vector<tf::Point>> MapserverStat::ogmCellCoordinatesToPoints(
    const nav_msgs::OccupancyGrid::ConstPtr ogm, const std::string &targetFrame,
    const tf::TransformListener &tfListener, const std::size_t &idxStart,
    const std::size_t &idyStart, const std::size_t &idxWidth,
    const std::size_t &idyHeight) {

  std::shared_ptr<std::vector<tf::Point>> points;
  // Sanity checks
  if (!ogm) {
    ROS_ERROR("OGM is empty");
  } else if ((idxStart + idxWidth) > ogm->info.width
      || (idyStart + idyHeight) > ogm->info.height || idxWidth == 0
      || idyHeight == 0) {
    ROS_ERROR("Requested range out of bound");
  } else {
    // Get the pose in the target frame
    tf::Pose ogmOriginPose;
    tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
    std::shared_ptr<tf::Stamped<tf::Pose>> ogmOriginInTargetFrame =
        Mapserver::getPoseInFrame(
            tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp,
                                  ogm->header.frame_id),
            targetFrame, tfListener);
    // Start calculating the transform
    if (ogmOriginInTargetFrame) {
      points = std::shared_ptr<std::vector<tf::Point>>(
          new std::vector<tf::Point>(idxWidth * idyHeight));
      const float cornerToCenterOffset = ogm->info.resolution / 2.0f;

      auto pointsIter = points->begin();
      // tf::Stamped<tf::Point> ptInOgmFrame(tf::Point(0,0,0), ogm->header.stamp, ogm->header.frame_id); // The metric point of a OGM cell in the OGM frame
      for (int idy = idyStart; idy < idyStart + idyHeight; ++idy) {
        for (int idx = idxStart; idx < idxStart + idxWidth;
            ++idx, ++pointsIter) {
          const tf::Point ptInOgmOrigin(
              float(idx) * ogm->info.resolution + cornerToCenterOffset,
              float(idy) * ogm->info.resolution + cornerToCenterOffset, 0.0f);
          // ptInOgmOrigin.setZ(0.0f); // Don't need this
          // ptInOgmFrame = ogmOriginInSourceFrame * ptInOgmOrigin; // We don't need this
          const tf::Point ptInTargetFrame = *ogmOriginInTargetFrame
              * ptInOgmOrigin;
          *pointsIter = ptInTargetFrame;
        }
      }
    } else {
      ROS_WARN("TF unsuccessful");
    }
  }
  return points;
}

sensor_msgs::PointCloud::Ptr MapserverStat::ogmCellsToPointCloud(
    const nav_msgs::OccupancyGrid::ConstPtr ogm, const std::string &targetFrame,
    const tf::TransformListener &tfListener, const std::size_t &idxStart,
    const std::size_t &idyStart, const std::size_t &idxWidth,
    const std::size_t &idyHeight) {
  sensor_msgs::PointCloud::Ptr ogmAsPointCloud;
  // First get the metric coordinates
  std::shared_ptr<std::vector<tf::Point>> points = ogmCellCoordinatesToPoints(
      ogm, targetFrame, tfListener, idxStart, idyStart, idxWidth, idyHeight);
  // Sanity checks: We rely on ogmCellCoordinatesToPoints
  if (points) {
    ogmAsPointCloud = boost::shared_ptr<sensor_msgs::PointCloud>(
        new sensor_msgs::PointCloud);
    ogmAsPointCloud->header = ogm->header;
    ogmAsPointCloud->header.frame_id =
        targetFrame.empty() ? ogmAsPointCloud->header.frame_id : targetFrame;
    ogmAsPointCloud->points.resize(points->size());
    ogmAsPointCloud->channels.resize(1);
    ogmAsPointCloud->channels.at(0).name = "OGM";
    ogmAsPointCloud->channels.at(0).values.resize(points->size());
    auto pointCloudValueIter = ogmAsPointCloud->channels.at(0).values.begin();
    auto pointCloudPointIter = ogmAsPointCloud->points.begin();
    auto pointIter = points->begin();
    for (int idy = idyStart; idy < idyStart + idyHeight; ++idy) {
      for (int idx = idxStart; idx < idxStart + idxWidth;
          ++idx, ++pointCloudValueIter, ++pointCloudPointIter, ++pointIter) {
        const int index = idx + (idy * ogm->info.width);
        *pointCloudValueIter = float(ogm->data.at(index)) / 100.0f;
        pointCloudPointIter->x = float(pointIter->x());
        pointCloudPointIter->y = float(pointIter->y());
        pointCloudPointIter->z = float(pointIter->z());
      }
    }
  } else {
    ROS_WARN("ogmCellCoordinatesToPoints is empty");
  }
  return ogmAsPointCloud;

}

std::shared_ptr<std::vector<tf::Point>> MapserverStat::getOgmCornerPoints(
    const nav_msgs::OccupancyGrid::ConstPtr ogm, const std::string &targetFrame,
    const tf::TransformListener &tfListener) {

  std::shared_ptr<std::vector<tf::Point>> points;
  // Sanity checks
  if (!ogm) {
    ROS_ERROR("OGM is empty");
  } else {
    // Get the pose in the target frame
    tf::Pose ogmOriginPose;
    tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
    correctInvalidOrientation(ogmOriginPose);
    std::shared_ptr<tf::Stamped<tf::Pose>> ogmOriginInTargetFrame =
        Mapserver::getPoseInFrame(
            tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp,
                                  ogm->header.frame_id),
            targetFrame, tfListener);
    // Start calculating the transform
    if (ogmOriginInTargetFrame) {
      const std::size_t numCorners = 4;
      points = std::shared_ptr<std::vector<tf::Point>>(
          new std::vector<tf::Point>(numCorners));
      // pointInTargetFrame = transformMatrixFromTargetFrameToSourceFrame * pointInSourceFrame;
      points->at(0) = *ogmOriginInTargetFrame * tf::Point(0.0f, 0.0f, 0.0f);
      points->at(1) = *ogmOriginInTargetFrame
          * tf::Point(ogm->info.width * ogm->info.resolution, 0.0f, 0.0f);
      points->at(2) = *ogmOriginInTargetFrame
          * tf::Point(ogm->info.width * ogm->info.resolution,
                      ogm->info.height * ogm->info.resolution, 0.0f);
      points->at(3) = *ogmOriginInTargetFrame
          * tf::Point(0.0f, ogm->info.height * ogm->info.resolution, 0.0f);
    } else {
      ROS_WARN("TF unsuccessful");
    }
  }
  return points;
}

std::shared_ptr<std::vector<tf::Point>> MapserverStat::getOgmCornerPoints(
    const nav_msgs::OccupancyGrid::ConstPtr ogm,
    const tf::Stamped<tf::Pose> &targetPose,
    const tf::TransformListener &tfListener) {
  std::shared_ptr<std::vector<tf::Point>> points;
  // Get pose of the OGM in the target frame
  std::shared_ptr<tf::Stamped<tf::Pose>> targetPoseInOgmFrame =
      Mapserver::getPoseInFrame(targetPose, ogm->header.frame_id, tfListener);
  if (targetPoseInOgmFrame) {
    // First get the corner points in own frame
    points = getOgmCornerPoints(ogm, ogm->header.frame_id, tfListener);
    if (points) {
      points->at(0) = targetPoseInOgmFrame->inverse() * points->at(0);
      points->at(1) = targetPoseInOgmFrame->inverse() * points->at(1);
      points->at(2) = targetPoseInOgmFrame->inverse() * points->at(2);
      points->at(3) = targetPoseInOgmFrame->inverse() * points->at(3);
    }
  }
  return points;
}

nav_msgs::OccupancyGrid::ConstPtr MapserverStat::ogmResize(
    const nav_msgs::OccupancyGrid::ConstPtr ogm,
    const float &targetResolution) {

  nav_msgs::OccupancyGrid::Ptr ogmTf =
      boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);

  const float scaling = ogm->info.resolution / targetResolution;
  // Get data as OpenCV
  // HACK Type should be CV_8SC1, but resize function only works with CV_8UC1
  const cv::Mat ogmCv(ogm->info.height, ogm->info.width, CV_8UC1,
                      (void*) (ogm->data.data()));
  cv::Mat ogmCvResized;
  // Resize
  cv::resize(ogmCv, ogmCvResized, cv::Size(), scaling, scaling,
             cv::INTER_LINEAR);
  // Copy back
  ogmTf->header = ogm->header;
  ogmTf->info = ogm->info;
  ogmTf->info.height = ogmCvResized.rows;
  ogmTf->info.width = ogmCvResized.cols;
  ogmTf->info.resolution = targetResolution;
  ogmTf->data.resize(ogmCvResized.rows * ogmCvResized.cols);
  memcpy((void*) ogmTf->data.data(), (void*) ogmCvResized.data,
         ogmCvResized.rows * ogmCvResized.cols);

  return ogmTf;
}

nav_msgs::OccupancyGrid::ConstPtr MapserverStat::ogmTf(
    const nav_msgs::OccupancyGrid::ConstPtr ogm,
    const tf::Stamped<tf::Pose> &targetOrigin, const float &targetResolution,
    const tf::TransformListener &tfListener,
    const bool resetPoseToOgmBoundary) {

  nav_msgs::OccupancyGrid::Ptr ogmTf;

  // TODO Check for XY shift, which can be easily calculated (Just shift the pose in xy and maybe check for resolution)

  // Sanity check for invalid orientation
  tf::Pose ogmOriginPose;
  tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
  correctInvalidOrientation(ogmOriginPose);

  // First check, if pose and resolution are the same, to minimize workload
  if (ogmOriginPose == targetOrigin) {
    const bool resolutionIsSame = utils::compare(
        targetResolution, ogm->info.resolution,
        std::min(ogm->info.resolution, targetResolution) / 2);
    if (!resolutionIsSame) {  // Scale the OGM
      ROS_DEBUG("Pose is the same: Just change the resolution of the OGM");
      return ogmResize(ogm, targetResolution);
    } else {
      ROS_DEBUG(
          "Pose and resolution are the same: Just return the original OGM");
      return ogm;
    }
  } else {  // Do a full transform of the OGM
    // First get the metric coordinates of the four corner points in the target and source frame
    std::shared_ptr<std::vector<tf::Point>> pointsInTargetFrame =
        getOgmCornerPoints(ogm, targetOrigin, tfListener);
//    tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
    std::shared_ptr<std::vector<tf::Point>> pointsInSourceFrame =
        getOgmCornerPoints(
            ogm,
            tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp,
                                  ogm->header.frame_id),
            tfListener);
    // Calculate the homography
    if (pointsInTargetFrame && pointsInSourceFrame) {
      // Load OGM as OpenCV image
      // HACK Type should be CV_8SC1, but resize function only works with CV_8UC1
      cv::Mat ogmCv(ogm->info.height, ogm->info.width, CV_8UC1,
                    (void*) (ogm->data.data()));

      std::vector<cv::Point2f> pointsSource(4), pointsTarget(4);
      pointsSource.at(0) = cv::Point2f(float(0), float(0));
      pointsSource.at(1) = cv::Point2f(float(ogm->info.width), float(0));
      pointsSource.at(2) = cv::Point2f(float(ogm->info.width),
                                       float(ogm->info.height));
      pointsSource.at(3) = cv::Point2f(float(0), float(ogm->info.height));

      pointsTarget.at(0) = cv::Point2f(
          float(pointsInTargetFrame->at(0).x() / targetResolution),
          float(pointsInTargetFrame->at(0).y() / targetResolution));
      pointsTarget.at(1) = cv::Point2f(
          float(pointsInTargetFrame->at(1).x() / targetResolution),
          float(pointsInTargetFrame->at(1).y() / targetResolution));
      pointsTarget.at(2) = cv::Point2f(
          float(pointsInTargetFrame->at(2).x() / targetResolution),
          float(pointsInTargetFrame->at(2).y() / targetResolution));
      pointsTarget.at(3) = cv::Point2f(
          float(pointsInTargetFrame->at(3).x() / targetResolution),
          float(pointsInTargetFrame->at(3).y() / targetResolution));

      // If the pose will be reset to the boundary
      tf::Stamped<tf::Pose> targetOriginNew = targetOrigin;
      if (resetPoseToOgmBoundary) {
        const cv::Point2f xyMinPoint = utils::getXYMin(pointsTarget);
        tf::Pose resetPose(
            tf::Quaternion(0, 0, 0, 1),
            tf::Vector3(tfScalar(xyMinPoint.x * targetResolution),
                        tfScalar(xyMinPoint.y * targetResolution),
                        tfScalar(0)));
        pointsTarget.at(0) -= xyMinPoint;
        pointsTarget.at(1) -= xyMinPoint;
        pointsTarget.at(2) -= xyMinPoint;
        pointsTarget.at(3) -= xyMinPoint;
        targetOriginNew *= resetPose;
      }

      // Do the warping to get the PGM in the new pose
      cv::Mat H = cv::findHomography(pointsSource, pointsTarget, 0);
      const cv::Point2f xyMaxPoint = utils::getXYMax(pointsTarget);
      cv::Mat ogmCvWarped(xyMaxPoint.y, xyMaxPoint.x, CV_8SC1);
      // Warp the OGM with linear interpolation (Boarders are set to unknown s.t. 50)
      cv::warpPerspective(ogmCv, ogmCvWarped, H, ogmCvWarped.size(),
                          cv::INTER_NEAREST, cv::BORDER_CONSTANT,
                          cv::Scalar(50));

      // Copy back
      ogmTf = boost::shared_ptr<nav_msgs::OccupancyGrid>(
          new nav_msgs::OccupancyGrid);
      ogmTf->header = ogm->header;
      ogmTf->header.frame_id = targetOriginNew.frame_id_;
      geometry_msgs::PoseStamped poseMsg;
      tf::poseStampedTFToMsg(targetOriginNew, poseMsg);
      ogmTf->info.origin = poseMsg.pose;
      ogmTf->info.height = ogmCvWarped.rows;
      ogmTf->info.width = ogmCvWarped.cols;
      ogmTf->info.resolution = targetResolution;
      ogmTf->data.resize(ogmCvWarped.rows * ogmCvWarped.cols);
      memcpy((void*) ogmTf->data.data(), (void*) ogmCvWarped.data,
             ogmCvWarped.rows * ogmCvWarped.cols);
    } else {
      ROS_WARN("pointsInTargetFrame or pointsInSourceFrame are empty");
    }
  }
  return ogmTf;

}

void MapserverStat::calcIsm4Mapserver(const double xIsmCenter_m,
                                      const double yIsmCenter_m,
                                      const double phiIsm_rad,
                                      const double ismResolution,
                                      cv::Mat &ismInRoi, cv::Mat &ism) {

  const double roiResolution = resolution_mPerTile;
  double ismMagnificationFactor = ismResolution / roiResolution;
  const int32_t x_dis_r = xIsmCenter_m / roiResolution;
  const int32_t y_dis_r = yIsmCenter_m / roiResolution;

  // Handle axis flipping
//  utils::rot90(ism,ismFlipcode);

  // Allocate space
  const cv::Size roiSizePx = ismInRoi.size();
  const cv::Point2f centerRoi(roiSizePx.height / 2.0f, roiSizePx.width / 2.0f);
  ismInRoi = cv::Mat(roiSizePx, ismInRoi.type(), cv::Scalar(50/*% Unknown*/));  // Flush input image

  // Put the ISM in a bigger rectangular image, to handle rotation without cropping
  int newIsmRowsCols = sqrt(pow(ism.rows, 2) + pow(ism.cols, 2));
  const cv::Point2f ismOldCenter(ism.size().height / 2.0f,
                                 ism.size().width / 2.0f);
  cv::Mat ismContainer(newIsmRowsCols, newIsmRowsCols, ism.type(),
                       cv::Scalar(50 /*% Unknown*/));
  cv::Rect ismInNewIsm((newIsmRowsCols - ism.cols) / 2,
                       (newIsmRowsCols - ism.rows) / 2, ism.cols, ism.rows);
  ism.copyTo(ismContainer(ismInNewIsm));

  // Resize ISM
  cv::Size newIsmSize(ismContainer.size().width * ismMagnificationFactor,
                      ismContainer.size().height * ismMagnificationFactor);
  cv::Mat ismResized;
  cv::resize(ismContainer, ismResized, newIsmSize, 0, 0, cv::INTER_NEAREST);

  // Rotate the ISM around its center
  const cv::Point2f centerISM(ismResized.size().height / 2.0f,
                              ismResized.size().width / 2.0f);
  cv::Mat rotation(2, 3, CV_32FC1);
  rotation = cv::getRotationMatrix2D(centerISM, -phiIsm_rad * rad2deg,
                                     1.0/*ZoomFactor*/);  // Negative angle, because OpenCV is CW regarding our convention
  cv::Mat ismRotated;
  cv::warpAffine(ismResized, ismRotated, rotation, ismResized.size(),
                 cv::INTER_NEAREST, cv::BORDER_CONSTANT,
                 cv::Scalar(50 /*% Unknown*/));

  // Copy the rotated ISM to its translated position in the ROI
  const cv::Point2f centerRotatedISM(ismRotated.size().height / 2.0f,
                                     ismRotated.size().width / 2.0f);
  cv::Rect ismInRoiRect = cv::Rect(centerRoi.x + x_dis_r - centerRotatedISM.x,
                                   centerRoi.y + y_dis_r - centerRotatedISM.y,
                                   ismRotated.cols, ismRotated.rows);
  ismRotated.copyTo(ismInRoi(ismInRoiRect));

//  // Store the first ISM to be converted
//  if (storeFirstIsm) {
//    static bool once = false;
//    if (!once) {
//      cv::imwrite(std::string("/tmp/") + storeFirstIsmName + std::string(".bmp"),ismInRoi); // RSB ISM
//      cv::imwrite(std::string("/tmp/") + storeFirstIsmName + std::string("Ism.bmp"),ism); // Native ROS ISM
//      once = true;
//    }
//  }
}

// Todo Do we some mutex around here ?
nav_msgs::OccupancyGrid::Ptr MapserverStat::oggTf(
    const std::string &targetFrame,
    const nav_msgs::OccupancyGrid::ConstPtr ismRos, const double targetRes) {

  nav_msgs::OccupancyGrid::Ptr oggTrans;

  // Get the transform between the frames
  tf::StampedTransform tfOcc;
  try {
    listenerTf->waitForTransform(ismRos->header.frame_id, targetFrame,
                                 ros::Time(0.0), ros::Duration(3.0));
    listenerTf->lookupTransform(ismRos->header.frame_id, targetFrame,
                                ros::Time(0.0), tfOcc);
  } catch (const std::exception &exc) {
    const std::string excStr(exc.what());
    ROS_ERROR("tfTileNameHandler: %s", excStr.c_str());
    return oggTrans;
  }

  // Get the arguments to pack the current ISM into the ROI frame
  // We need to respect Z and also non aligned occupancy maps
//  int32_t x_dis_r_copy = x_dis_r;
//  int32_t y_dis_r_copy = y_dis_r;
//  int32_t z_dis_r_copy = z_dis_r;
  double x = tfOcc.getOrigin().getX();
  double y = tfOcc.getOrigin().getY();
//  double z = tfOcc.getOrigin().getZ();
  double phi_rad = utils::conversion::quaternion2eulerYaw(
      tfOcc.getRotation().getW(), tfOcc.getRotation().getX(),
      tfOcc.getRotation().getY(), tfOcc.getRotation().getZ());

  // Allocate the new Occ TODO: This is to sloppy, need propper definition
  oggTrans = boost::shared_ptr<nav_msgs::OccupancyGrid>(
      new nav_msgs::OccupancyGrid);
  oggTrans->header = ismRos->header;
  oggTrans->header.frame_id = targetFrame;
  oggTrans->info.resolution =
      targetRes < 0.0 ? ismRos->info.resolution : targetRes;
//  oggTrans->info.origin = ???
  const int width = (maxX_m - minX_m) / oggTrans->info.resolution;
  const int height = (maxY_m - minY_m) / oggTrans->info.resolution;
  oggTrans->info.width = width;
  oggTrans->info.height = height;

  // Allocate opencv images to do the work
  cv::Mat ismInRoi(height, width, CV_8SC1);
  cv::Mat ism(ismRos->info.height, ismRos->info.width, CV_8SC1,
              (void*) (&ismRos->data[0]));

  ROS_DEBUG("TF.xyz to %s: %f %f %f", ismRos->header.frame_id.c_str(),
            tfOcc.getOrigin().x(), tfOcc.getOrigin().y(),
            tfOcc.getOrigin().z());
  ROS_DEBUG("ismRos->info.origin.position.x,y: %f %f",
            ismRos->info.origin.position.x, ismRos->info.origin.position.y);

  // Calculate the tranlation from the base_footprint to the ISM origin
  // assuming that the coordinate systems are aligned after REP-103, and
  // that the height does not care
  const double xBasefootprint2Ism_m = tfOcc.getOrigin().x()
      + ismRos->info.origin.position.x;
  const double yBasefootprint2Ism_m = tfOcc.getOrigin().y()
      + ismRos->info.origin.position.y;
  const double ismWidthX_m = ismRos->info.width * ismRos->info.resolution;
  const double ismHeightY_m = ismRos->info.height * ismRos->info.resolution;
  double fz_rad = utils::conversion::quaternion2eulerYaw(
      ismRos->info.origin.orientation.w, ismRos->info.origin.orientation.x,
      ismRos->info.origin.orientation.y, ismRos->info.origin.orientation.z);
  ROS_DEBUG("Euler orientation z (deg) %f", fz_rad * rad2deg);

  // Get the Center of the ISM after the rotation, assuming that we only operate in the plane
  double ismCenterX_m = ismWidthX_m / 2.0;
  double ismCenterY_m = ismHeightY_m / 2.0;
  utils::conversion::rotXY(ismCenterX_m, ismCenterY_m, fz_rad);
  ROS_DEBUG("ismCenterXY_m %f %f", ismCenterX_m, ismCenterY_m);

  // Rotate the center of the ISM in the footprint
  double xBasefootprint2IsmCenter_m = xBasefootprint2Ism_m + ismCenterX_m;
  double yBasefootprint2IsmCenter_m = yBasefootprint2Ism_m + ismCenterY_m;
  utils::conversion::rotXY(xBasefootprint2IsmCenter_m,
                           yBasefootprint2IsmCenter_m, phi_rad);

  // Add the position inside the ROI
  const double xRoi2IsmCenter = xBasefootprint2IsmCenter_m + x;
  const double yRoi2IsmCenter = yBasefootprint2IsmCenter_m + y;
  // Get the final orientation of the ISM
  double ismOrientation = fz_rad + phi_rad;  // TODO Check if fz needs to be assigned with minus

  // Merge ROS-ISM into RSB-ISM
  calcIsm4Mapserver(xRoi2IsmCenter, yRoi2IsmCenter, ismOrientation,
                    ismRos->info.resolution, ismInRoi, ism);

  const size_t size = width * height;
  oggTrans->data.resize(size);
  memcpy((void*) oggTrans->data.data(), (void*) ismInRoi.data, size);

  return oggTrans;

}

void MapserverStat::doIsmFusion(const nav_msgs::OccupancyGrid::ConstPtr &msg,
                                const std::string &topic) {

  // The current OGM (map) with which the ISM (msg) is fused
  mrpt::maps::COccupancyGridMap2D *map = NULL;
  // The current map center tf name
  std::string tileTfName;
  // The current map origin tf name
  std::string tileOriginTfName;

  // Sanity checks
  if (currentTileTfName.empty()) {
    ROS_WARN("currentTileTfName is empty: Skipping sensor fusion");
    return;
  }

  // Get the current map stack, or allocate new space
  mapRefresh.lock();
  std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack =
      currentMapStack;
  try {
//      for (auto& x: *mapStack) {
//        std::cout << x.first << ": " << x.second << '\n';
//      }
    map = mapStack->at(topic.c_str());
  } catch (...) {  // It only throws if the topic is not in the map
    ROS_INFO_STREAM("Add new map for topic " << topic);
    map = mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def);
    currentMapStack->insert(
        std::pair<std::string, mrpt::maps::COccupancyGridMap2D*>(topic, map));
    lastMapStack->insert(
        std::pair<std::string, mrpt::maps::COccupancyGridMap2D*>(
            topic,
            mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def)));
  }
  tileTfName = currentTileTfName;
  mapRefresh.unlock();
  tileOriginTfName = tileTfName + tileOriginTfSufixForRoiOrigin;

// Some debug stuff
//  tf::Pose ogmPose; tf::poseMsgToTF(msg->info.origin, ogmPose);
//  tf::Stamped<tf::Pose> originAsTargetPose(ogmPose, msg->header.stamp, msg->header.frame_id);
//  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, originAsTargetPose, msg->info.resolution, *listenerTf); // Check minimal function
//  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, originAsTargetPose, msg->info.resolution / 2, *listenerTf); // Check only resizing

  // Target pose is the origin of the OGM frame
  tf::Stamped<tf::Pose> targetPose(tf::Pose(tf::Quaternion(0, 0, 0, 1)),
                                   msg->header.stamp, tileOriginTfName);
  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, targetPose,
                                                           resolution_mPerTile,
                                                           *listenerTf);  // Check homography

  // Sanity check to speed up
  if (!ogmTransformed) {
    ROS_ERROR("ISM as OGM not valid");
    return;
  } else if (ogmTransformed->info.height <= 0
      || ogmTransformed->info.width <= 0) {
    ROS_ERROR("ISM has no size");
    return;
  } else if (map == NULL) {
    ROS_ERROR("OGM is empty");
    return;
  } else if (map->getSizeY() <= 0 || map->getSizeX() <= 0) {
    ROS_ERROR("OGM has no size");
    return;
  } else {
    ROS_DEBUG("OK: Do the sensor fusion");
  }

  // Update a sensor scan:
  // The transformation already points to the lower left edge of the sensor scan
  // Rotation is not allowed for now
  // TODO Check with pose to speed up everything regarding the parameter ogmTf(..., true)
  // Assumptions are made for ISM and OGM: origin pose coincide, resolution is same
  const float max_x = ogmTransformed->info.height
      * ogmTransformed->info.resolution;
  const float min_x = 0.0f;
  const float max_y = ogmTransformed->info.width
      * ogmTransformed->info.resolution;
  const float min_y = 0.0f;
  const float transX = ogmTransformed->info.origin.position.x;
  const float transY = ogmTransformed->info.origin.position.y;
  const float transZ = ogmTransformed->info.origin.position.z;

  ROS_DEBUG(
      "\n  Everything should be OK, if the pose is 0,0,0 and the frames are the same:"
      "  ISM coordinate x, y, z, frame_id: %f, %f, %f, %s\n"
      "  OGM frame name: %s\n\n",
      transX, transY, transZ, ogmTransformed->header.frame_id.c_str(),
      targetPose.frame_id_.c_str());

//  const int mapCenterX = int((def.max_x - def.min_x) / def.resolution / 2.0);
//  const int mapCenterY = int((def.max_y - def.min_y) / def.resolution / 2.0);
//  const int mapOffsetXStart = transX / def.resolution + mapCenterX;
//  const int mapOffsetXEnd   = (transX + max_x - min_x) / def.resolution + mapCenterX;
//  const int mapOffsetYStart = transY / def.resolution + mapCenterY;
//  const int mapOffsetYEnd   = (transY + max_y - min_y) / def.resolution + mapCenterY;

  // Add blind spots to the ISM
  MapserverStat::addBlindSpotsToOgm(ogmTransformed, blindSpots,
                                    *this->listenerTf);

  if (debug) {
    if (!topic.compare(debugIsmTopic)) {
      publisherIsmAsOgm.publish(ogmTransformed);
    }
    // Send OGM as point cloud
    //      sensor_msgs::PointCloudPtr ogmPtCloud = ogmCellsToPointCloud(msg, "world", *listenerTf, 0, 0, msg->info.width, msg->info.height);
    //      if(ogmPtCloud) {
    //          publisherIsmAsPointCloud.publish(ogmPtCloud);
    //      } else {
    //          ROS_ERROR("Debug: ISM as point cloud not valid");
    //      }
  }

  for (int idy = 0;
      idy < std::min(ogmTransformed->info.height, map->getSizeY()); ++idy) {
    for (int idx = 0;
        idx < std::min(ogmTransformed->info.width, map->getSizeX()); ++idx) {
      // Get the index to get the correct cell out of the update
//      const int y = idx-mapOffsetXStart;
//      const int x = idy-mapOffsetYStart;
//      const int index = x + (y * msg->info.width);
      const int idxOgm = idx + (idy * map->getSizeX());
      const int idxIsm = idx + (idy * ogmTransformed->info.width);
      // Update the cell
      // There are only int values in the update:
      // -1 (unknown), 0 - 100 (occupancyness in percent)
//      if (msg->data.at(index) >= 0) {
//      if (ogmTransformed->data.at(idxIsm) >= 0) {
//      if (idx > 20) {
//        if(debug)INFO_MSG("Before: " << mapStack[mapIdx].getCell(idx,idy))
//        INFO_MSG("Before: " << map->getCell(idx,idy));
      map->updateCell(idx, idy,
                      float(ogmTransformed->data.at(idxIsm)) / 100.0f);
//        map->updateCell(idx,idy,60.0f / 100.0f);
//        INFO_MSG("After: " << map->getCell(idx,idy));
//        if(debug)INFO_MSG("After: " << mapStack[mapIdx].getCell(idx,idy))
//      }
    }
  }

  // TEST: Draw the ISM in the upper left corner of the Occ
//  for (int idy = 0; idy < msg->info.height ; ++idy) {
//    for (int idx = 0; idx < msg->info.width ; ++idx) {
//      // Get the index to get the correct cell out of the update
//      const int index = idx + (idy * msg->info.width);
//      map->updateCell(idx,idy,float(msg->data.at(index)) / 100.0f);
//    }
//  }

  if (debug) {
    this->msgDebug = msg;
  }
}

nav_msgs::OccupancyGrid MapserverStat::getIsmOutput(
    const mapserver_msgs::mapPrimitive &view,
    const mrpt::maps::COccupancyGridMap2D &input) {

  cv::Mat statisticMat, croppedMat; /*float matrices*/
  nav_msgs::OccupancyGrid output;

  std::stringstream os;
  os << view;
  ROS_INFO("view:\n%s\n", os.str().c_str());

  // Calculate the requested ROI
  const double xView = view.pose.pose.position.x;  // Meter
  const double yView = view.pose.pose.position.y;  // Meter
  const double wView = view.width * view.resolution;  // Tiles * meter/tiles
  const double hView = view.height * view.resolution;  // Tiles * meter/tiles
  const double dView = view.depth * view.resolution;  // Tiles * meter/tiles
  const std::string sourceframe =
      view.frame_id.empty() ? machine::frames::names::BASE_LINK : view.frame_id;
  tf::Quaternion q;
  tf::quaternionMsgToTF(view.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  const double zRotView = yaw;

  // Resize the resolution
  cv::Size size(view.width, view.depth);
  cv::resize(statisticMat, croppedMat, size, 0, 0, cv::INTER_NEAREST);

  // Copy the meta information
  output.header.stamp = ros::Time::now();
  output.header.frame_id = sourceframe;
  output.info.resolution = view.resolution;
  output.info.width = croppedMat.cols;
  output.info.height = croppedMat.rows;
  output.info.origin = view.pose.pose;
  const int arraySize = croppedMat.cols * croppedMat.rows;

  // Copy everything to output
  output.data.resize(arraySize);
  for (int idx = 0; idx < arraySize; ++idx) {
    output.data.at(idx) = croppedMat.at<char>(idx);
  }

  return output;
}

void MapserverStat::formatAndSendGrid(
    std::vector<std::string> &list,
    std::string &frame,
    const std::shared_ptr<
        std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack,
    ros::NodeHandle &n, const std::string topicPrefixGrid,
    const std::shared_ptr<tf::Pose> tfPose, std::string topicSuffixGrid) {
  const char fName[] = "formatAndSendGrid";
  const float gridSpacing_m = 0.01;
  const float minDrawOccupancyUpdateCertainty = 0.5;

  // Sanity checks
  if (frame.empty()) {
    ROS_WARN("%s: formatAndSendGrid: frame is empty", fName);
    return;
  } else if (!mapStack) {
    ROS_WARN("%s: mapStack pointer is empty", fName);
    return;
  }
  if (list.empty()) {
    ROS_WARN_ONCE("%s: list is empty -> sending all maps", fName);
  }

  // Initialize the iterator and decide which one to use
  // If the list is empty, process the whole map stack, otherwise
  // send the content defined in list
  std::map<std::string, mrpt::maps::COccupancyGridMap2D*>::iterator mapIt;
  std::vector<std::string>::iterator listIt;
  int idxIt = 0;
  if (list.size() > 0) {
    listIt = list.begin();
  }
  if (mapStack->size() > 0) {
    mapIt = mapStack->begin();
  } else {
    ROS_WARN("%s: mapStack is empty", fName);
    return;
  }

  // Allocate the maximal size for the points to send
  std::vector<geometry_msgs::Point> points(
      mapIt->second->getSizeY() * mapIt->second->getSizeX());

  while (true) {
    // Check if we have to work with the list iterator, if not ...
    if (list.size() > 0) {
      if (listIt == list.end()) {
        break;
      }
      mapIt = mapStack->find(*listIt);
      if (mapIt == mapStack->end()) {
        ROS_WARN("%s: No entry for '%s'", fName, listIt->c_str());
        continue;
      } else if (mapIt->second == NULL) {
        ROS_WARN("%s: Map pointer for '%s' is empty", fName, listIt->c_str());
        continue;
      }
      ++listIt;
    } else {  // ... just process the full map stack
      if (idxIt > 0) {
        ++mapIt;
      }  // else, it is the first round
      if (mapIt == mapStack->end()) {
        break;
      }
    }

    // Check for publisher and advertise one, if missing
    static std::map<std::string, ros::Publisher> mapPublisher;
    auto publisherIt = mapPublisher.find(mapIt->first);
    if (publisherIt == mapPublisher.end()) {
      ROS_INFO("%s: Advertise map publisher with topic %s", fName,
               mapIt->first.c_str());
      mapPublisher.insert(
          std::pair<std::string, ros::Publisher>(
              mapIt->first,
              n.advertise<nav_msgs::GridCells>(
                  topicPrefixGrid + mapIt->first + topicSuffixGrid, 1)));
      publisherIt = --mapPublisher.end();
    }

    // Format the map
    tf::Point pointTf;
    geometry_msgs::Point pointMsg;
    nav_msgs::GridCells msg;
    msg.cell_height = resolution_mPerTile;
    msg.cell_width = resolution_mPerTile;
    auto pointsIt = points.begin();
    for (int idy = 0; idy < mapIt->second->getSizeY(); ++idy) {
      for (int idx = 0; idx < mapIt->second->getSizeX(); ++idx) {
        // We get values from 0 .. 100
        if (mapIt->second->getCell(idx, idy)
            > minDrawOccupancyUpdateCertainty) {
          pointTf[0] = (float(idx) * resolution_mPerTile);
          pointTf[1] = (float(idy) * resolution_mPerTile);
          pointTf[2] = float(idxIt) * gridSpacing_m;  // Overlay each layer by some distance

          if (tfPose) {  // Check for transformation
            pointTf = *tfPose * pointTf;
          }
          tf::pointTFToMsg(pointTf, pointMsg);
          *pointsIt = pointMsg;
          if (pointsIt != points.end()) {
            ++pointsIt;
          }
        }
      }
    }
    msg.cells.assign(points.begin(), pointsIt);

    // Publish the map
    msg.header.frame_id = frame;
    msg.header.stamp = ros::Time::now();
    publisherIt->second.publish(msg);

    ++idxIt;
  }

}

void MapserverStat::addBlindSpotsToOgm(
    nav_msgs::OccupancyGrid::ConstPtr ogm, const BlindSpots &blindSpots,
    const tf::TransformListener &tfListener) {

  // Test
//  tf::Pose p1 = tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(tfScalar(0), tfScalar(0), tfScalar(0)));
//  tf::Pose p2 = tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(tfScalar(1), tfScalar(1), tfScalar(0)));
//  std::string pf1 = "base_link";
//  tf::Stamped<tf::Pose> ps1 =  tf::Stamped<tf::Pose>(p1, msg->header.stamp, pf1);
//  tf::Stamped<tf::Pose> ps2 =  tf::Stamped<tf::Pose>(p2, msg->header.stamp, pf1);
//  BlindSpot pt = std::make_tuple(ps1, ps2);
//  BlindSpots blindSpots;
//  pts.push_back(pt);

  // We assume that the OGM lies in its origin (s.t. ogmTransformed->info.origin)
  for (auto pt = blindSpots.begin(); pt < blindSpots.end(); ++pt) {

    // Expand the two poses to all four points of the rectangular
    // points which lie straight in the source frame
    // p2-----p3
    // |       |
    // p1-----p4
    const tfScalar p1SrcX = std::min(std::get<0>(*pt).getOrigin().getX(),
                                     std::get<1>(*pt).getOrigin().getX());
    const tfScalar p1SrcY = std::min(std::get<0>(*pt).getOrigin().getY(),
                                     std::get<1>(*pt).getOrigin().getY());
    const tfScalar p2SrcX = p1SrcX;
    const tfScalar p2SrcY = std::max(std::get<0>(*pt).getOrigin().getY(),
                                     std::get<1>(*pt).getOrigin().getY());
    const tfScalar p3SrcX = std::max(std::get<0>(*pt).getOrigin().getX(),
                                     std::get<1>(*pt).getOrigin().getX());
    const tfScalar p3SrcY = p2SrcY;
    const tfScalar p4SrcX = p3SrcX;
    const tfScalar p4SrcY = p1SrcY;
    const tf::Quaternion q = tf::Quaternion(0, 0, 0, 1);
    const tf::Stamped<tf::Pose> ps1Src = tf::Stamped<tf::Pose>(
        tf::Pose(q, tf::Vector3(p1SrcX, p1SrcY, tfScalar(0))),
        ogm->header.stamp, std::get<0>(*pt).frame_id_);
    const tf::Stamped<tf::Pose> ps2Src = tf::Stamped<tf::Pose>(
        tf::Pose(q, tf::Vector3(p2SrcX, p2SrcY, tfScalar(0))),
        ogm->header.stamp, std::get<0>(*pt).frame_id_);
    const tf::Stamped<tf::Pose> ps3Src = tf::Stamped<tf::Pose>(
        tf::Pose(q, tf::Vector3(p3SrcX, p3SrcY, tfScalar(0))),
        ogm->header.stamp, std::get<0>(*pt).frame_id_);
    const tf::Stamped<tf::Pose> ps4Src = tf::Stamped<tf::Pose>(
        tf::Pose(q, tf::Vector3(p4SrcX, p4SrcY, tfScalar(0))),
        ogm->header.stamp, std::get<0>(*pt).frame_id_);

    // Get the points in the destination frame
    std::shared_ptr<tf::Stamped<tf::Pose>> ps1Dst = Mapserver::getPoseInFrame(
        ps1Src, ogm->header.frame_id, tfListener);
    std::shared_ptr<tf::Stamped<tf::Pose>> ps2Dst = Mapserver::getPoseInFrame(
        ps2Src, ogm->header.frame_id, tfListener);
    std::shared_ptr<tf::Stamped<tf::Pose>> ps3Dst = Mapserver::getPoseInFrame(
        ps3Src, ogm->header.frame_id, tfListener);
    std::shared_ptr<tf::Stamped<tf::Pose>> ps4Dst = Mapserver::getPoseInFrame(
        ps4Src, ogm->header.frame_id, tfListener);

    ROS_DEBUG_STREAM(
        std::setprecision(2) << "Blind spot poses [p1 -- p4] in " << std::get<0>(*pt).frame_id_ << " frame : \n" << "[x: " << ps1Src.getOrigin().getX() << ", y: " << ps1Src.getOrigin().getY() << "]\n" << "[x: " << ps2Src.getOrigin().getX() << ", y: " << ps2Src.getOrigin().getY() << "]\n" << "[x: " << ps3Src.getOrigin().getX() << ", y: " << ps3Src.getOrigin().getY() << "]\n" << "[x: " << ps4Src.getOrigin().getX() << ", y: " << ps4Src.getOrigin().getY() << "]\n");
    ROS_DEBUG_STREAM(
        std::setprecision(2) << "Blind spot poses [p1 -- p4] in " << ogm->header.frame_id << " frame : \n" << "[x: " << ps1Dst->getOrigin().getX() << ", y: " << ps1Dst->getOrigin().getY() << "]\n" << "[x: " << ps2Dst->getOrigin().getX() << ", y: " << ps2Dst->getOrigin().getY() << "]\n" << "[x: " << ps3Dst->getOrigin().getX() << ", y: " << ps3Dst->getOrigin().getY() << "]\n" << "[x: " << ps4Dst->getOrigin().getX() << ", y: " << ps4Dst->getOrigin().getY() << "]\n");

    // Draw if possible
    if (ps1Dst != NULL && ps2Dst != NULL && ps3Dst != NULL && ps4Dst != NULL) {
      // Get the raw pointer
      std::shared_ptr<cv::Mat> map = Mapserver::rosOccToImage(ogm);
      // Draw the blind spot
      std::vector<cv::Point> points(4);
      points.at(0) = cv::Point(
          ps1Dst->getOrigin().getX() / ogm->info.resolution,
          ps1Dst->getOrigin().getY() / ogm->info.resolution);
      points.at(1) = cv::Point(
          ps2Dst->getOrigin().getX() / ogm->info.resolution,
          ps2Dst->getOrigin().getY() / ogm->info.resolution);
      points.at(2) = cv::Point(
          ps3Dst->getOrigin().getX() / ogm->info.resolution,
          ps3Dst->getOrigin().getY() / ogm->info.resolution);
      points.at(3) = cv::Point(
          ps4Dst->getOrigin().getX() / ogm->info.resolution,
          ps4Dst->getOrigin().getY() / ogm->info.resolution);

      cv::fillConvexPoly(
          *map, points,
          cv::Scalar(50.0 /*Unknown value in ROS occupancy grid map*/),
          8 /*8-connected line*/);
    } else {
      ROS_WARN_STREAM("Blind spot not transformable");
    }
  }
}

///
/// \brief Shows the current RPC if debug is on
///
//void showRpc() {
//    if (debug) {
//        ROS_INFO("DEBUG: Show the requests");
//        if(mapStackStatisticDebug.empty() || mapStackStatisticDebug.empty()) {
//            ROS_ERROR("mapStackStatistic is empty");
//            return;
//        }
//        cv::Mat mapStackStatisticRequestDebugTmp, mapStackStatisticDebugTmp;
//        mtxShowRpc.lock();
//            mapStackStatisticRequestDebug.copyTo(mapStackStatisticRequestDebugTmp);
//            mapStackStatisticDebug.copyTo(mapStackStatisticDebugTmp);
//            mapStackStatisticDebugTmp.release();
//            mapStackStatisticRequestDebugTmp.release();
//            cv::RotatedRect rectTmp = rect;
//        mtxShowRpc.unlock();
//
//        std::stringstream os;
//        os << ros::Time::now();
//        {
//            cv::imshow(std::string("mapStackStatisticRequest"), mapStackStatisticRequestDebugTmp);
//            cv::setWindowTitle(std::string("mapStackStatisticRequest"),
//                               std::string("mapStackStatisticRequest at ") + os.str());
//        }
//        {
//            // Draw the request in pseudo-colors
//            if (debugDrawRpc) {
//                // TODO Make a cast which is possible to handle all data types
//                utils::castCopyImage(mapStackStatisticDebugTmp, mapStackStatisticDebugTmp, CV_16UC1);
//                mapStackStatisticDebugTmp.convertTo(mapStackStatisticDebugTmp, CV_8UC3);
//                cv::cvtColor( mapStackStatisticDebugTmp, mapStackStatisticDebugTmp, CV_GRAY2BGR );
//                utils::drawRotatedRectInImage(mapStackStatisticDebugTmp, rect, cv::Scalar(0,0,255));
//            }
//
//            cv::imshow(std::string("mapStackStatistic"), mapStackStatisticDebugTmp);
//            cv::setWindowTitle(std::string("mapStackStatistic"),
//                               std::string("mapStackStatistic at ") + os.str());
//        }
//        cv::waitKey(1);
//    }
//}

void MapserverStat::addMapToResponse(
    const std::string &name, mrpt::maps::COccupancyGridMap2D* map,
    mapserver::ismStackFloat::Response &response) {
  if (map == NULL) {
    ROS_WARN("Map pointer empty");
    return;
  }

  response.response.mapNames.strings.resize(
      response.response.mapNames.strings.size() + 1);
  response.response.mapStack.resize(response.response.mapStack.size() + 1);

  auto mapName = response.response.mapNames.strings.end() - 1;
  *mapName = name;

  const std::size_t numCells = map->getSizeX() * map->getSizeY();
  // Copy the cells
  auto mapRes = (response.response.mapStack.end() - 1);
  mapRes->map.resize(numCells);
  for (int idx = 0; idx < numCells; ++idx) {
    int xIdx = idx % map->getSizeX();
    int yIdx = idx / map->getSizeX();
    mapRes->map.at(idx) = map->getCell(xIdx, yIdx);
  }

  mapRes->header.stamp = ros::Time::now();
  mapRes->info.height = map->getSizeY();
  mapRes->info.width = map->getSizeX();
  const tf::Pose pose = tf::Pose(tf::Quaternion(0, 0, 0, 1));
  tf::poseTFToMsg(pose, mapRes->info.origin);
  mapRes->info.resolution = map->getResolution();
}

bool MapserverStat::mapStatServerMapStack(
    mapserver::ismStackFloat::Request &req,
    mapserver::ismStackFloat::Response &res) {
  mapRefresh.lock();
  auto mapStack = currentMapStack;
  std::string tileName = currentTileTfName;
  mapRefresh.unlock();

  ROS_INFO("statMapserver-mapStack method called");
  if (req.request.strings.empty()) {
    ROS_INFO("Respond the full map stack");
    for (auto it = mapStack->begin(); it != mapStack->end(); ++it) {
      addMapToResponse(it->first, it->second, res);
    }
  } else {
    for (auto it = req.request.strings.begin(); it != req.request.strings.end();
        ++it) {
      auto itMapStack = mapStack->find(*it);
      if (itMapStack != mapStack->end()) {
        addMapToResponse(itMapStack->first, itMapStack->second, res);
      }
    }
  }

  res.response.header.frame_id = tileName + tileOriginTfSufixForRoiOrigin;
  res.response.header.stamp = ros::Time::now();
  for (auto it = res.response.mapStack.begin();
      it != res.response.mapStack.end(); ++it) {
    it->header.frame_id = res.response.header.frame_id;
  }
  ROS_INFO("statMapserver-mapStack method called: Finish");
  return true;
}

std::shared_ptr<mrpt::maps::COccupancyGridMap2D> MapserverStat::maxPooling(
    std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> maps) {
  // Allocate the result and copy the first map
  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> map = std::shared_ptr<
      mrpt::maps::COccupancyGridMap2D>(new mrpt::maps::COccupancyGridMap2D);
  *map = *(maps->begin()->second);
  for (auto it = ++maps->begin(); it != maps->end(); ++it) {
    ROS_ASSERT(map->getRawMap().size() != map->getRawMap().size());
    for (size_t idx = 0; idx < map->getRawMap().size(); ++idx) {
      const_cast<std::vector<mrpt::maps::COccupancyGridMap2D::cellType>&>(map
          ->getRawMap()).at(idx) = std::max(map->getRawMap().at(idx),
                                            it->second->getRawMap().at(idx));
    }
  }
  return map;
}

std::shared_ptr<mrpt::maps::COccupancyGridMap2D> MapserverStat::multiPooling(
    std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> maps) {
  // Allocate the result and copy the first map
  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> map = std::shared_ptr<
      mrpt::maps::COccupancyGridMap2D>(new mrpt::maps::COccupancyGridMap2D);
  *map = *(maps->begin()->second);
  for (auto it = ++maps->begin(); it != maps->end(); ++it) {
    ROS_ASSERT(map->getRawMap().size() != map->getRawMap().size());
    for (size_t idy = 0; idy < map->getSizeY(); ++idy) {
      for (size_t idx = 0; idx < map->getSizeX(); ++idx) {
        map->updateCell(idx, idy, it->second->getCell(idx, idy));
      }
    }
  }
  return map;
}

bool MapserverStat::mapStatServerSingleLayerOgm(mapserver::ism::Request &req,
                                                mapserver::ism::Response &res) {

  // !!! Treat all requests in the base_link frame of the map !!!
  // TODO Fix this!!!
  const std::string frameId("base_link");  // should be req.request.header.frame_id
  mapRefresh.lock();
  auto mapStack = currentMapStack;
  std::string tileName = currentTileTfName;
  mapRefresh.unlock();

  // Perform sanity checks
  // Rules: There can be two "actions": multi-opinion pooling "multi" or max pooling "max" between layers
  // Action can only be performed, iff the req_info field holds multiple comma-seperated maps
  const std::vector<std::string> actionsAvailable = { "multi", "max" };
  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> map;
  auto it = actionsAvailable.begin();
  std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> reqMaps =
      std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>>(
          new std::map<std::string, mrpt::maps::COccupancyGridMap2D*>);
  std::vector<std::string> topics;
  if (!req.request.action.empty()) {  // Perform pooling and return a single layer map
    // CHeck if action is available
    for (; it != actionsAvailable.end(); ++it) {
      req.request.action.compare(*it);
    }
    if (it == actionsAvailable.end()) {
      ROS_ERROR_STREAM(
          "mapStatServerSingleLayerOgm: No available action: " << req.request.action << "\n");
      return false;
    }

    // Check if the req_info holds a CSV list of at least two available maps
    boost::split(topics, req.request.req_info, boost::is_any_of(","));
    for (auto topic = topics.begin(); topic < topics.end(); ++topic) {
      auto it = mapStack->find(*topic);
      if (it == mapStack->end()) {
        ROS_ERROR_STREAM( "mapStatServerSingleLayerOgm: Unknown map identifier: " << *topic << "\n");
        return false;
      } else {
        reqMaps->insert(std::make_pair(it->first, it->second));
      }
    }
    // Apply the pooling
    // TODO Use function look up in actionsAvailable
    if (req.request.action.compare("multi") == 0) {
      map = multiPooling(reqMaps);
    } else if (req.request.action.compare("max") == 0) {
      map = maxPooling(reqMaps);
    }
  } else {  // Copy the single requested map
    auto it = mapStack->find(req.request.req_info);
    if (it == mapStack->end()) {
      ROS_ERROR_STREAM(
          "mapStatServerSingleLayerOgm: Unknown map identifier: " << req.request.req_info << "\n");
      return false;
    } else {
      map = std::shared_ptr<mrpt::maps::COccupancyGridMap2D>(
          new mrpt::maps::COccupancyGridMap2D);
      *map = *(it->second);
    }
  }

  // Get the requested view
  try {
    // Get the map as an image
    std::shared_ptr<cv::Mat> src = mrptOggToGrayScale(*map);

    // Request the current odometry
    tf::StampedTransform transformedViewInRoi;
    try {
      listenerTf->waitForTransform(tileName, std::string("base_link"),
                                   ros::Time(0), ros::Duration(3.0));
      listenerTf->lookupTransform(tileName, std::string("base_link"),
                                  ros::Time(0), transformedViewInRoi);
      ROS_DEBUG("mapStatServerSingleLayerOgm: abs(x,y,z): %f",
                transformedViewInRoi.getOrigin().length());
    } catch (const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("%s", excStr.c_str());
      throw std::runtime_error(
          std::string(
              "mapStatServerSingleLayerOgm: Won't perform any tf without valid machine model"));
    }

    // Get the desired rotation
    tf::Matrix3x3 m(transformedViewInRoi.getRotation());
    double rollMachine, pitchMachine, yawMachine;
    m.getRPY(rollMachine, pitchMachine, yawMachine);
    Eigen::Matrix4d roi_machineRoi = ctf::trans<double>(
        transformedViewInRoi.getOrigin().getX(),
        transformedViewInRoi.getOrigin().getY(), 0.0)
        * ctf::rotZ<double>(yawMachine);

    // Request the view
    cv::Mat dst;
    utils::cutView(*src, dst, resolution_mPerTile /*m/px*/,
                   req.request.pose.pose.position.x,
                   req.request.pose.pose.position.y,
                   req.request.width * req.request.resolution,
                   req.request.depth * req.request.resolution, yawMachine,
                   roi_machineRoi, src->type());

    // Resize the resolution
    // TODO Check if all the other do the same
    cv::Size size(req.request.width, req.request.depth);
    if (req.request.resolution > map->getResolution()) { // Downscaling
      // Do averaging over pixel (INTER_AREA should be the desired method for
      // down scaling regarding https://stackoverflow.com/questions/29572347/interpolation-for-smooth-downscale-of-image-in-opencv)
      cv::resize(dst, dst, size, 0, 0, cv::INTER_AREA);
    } else { // Upscaling
      cv::resize(dst, dst, size, 0, 0, cv::INTER_NEAREST);
    }

    // Copy the meta information
    res.response.header.frame_id = frameId;
    res.response.info.width = dst.cols;
    res.response.info.height = dst.rows;
    res.response.info.resolution = req.request.resolution;
    const int arraySize = dst.cols * dst.rows;
    res.response.data.resize(arraySize);

    // Copy the payload
    // TODO replace by memcpy if memory is not fragmented
    for (int idx = 0; idx < arraySize; ++idx) {
      res.response.data.at(idx) = dst
          .at<mrpt::maps::COccupancyGridMap2D::cellType>(idx);
    }
  } catch (...) {
    ROS_ERROR_STREAM("mapStatServerSingleLayerOgm: Something happened");
    return false;
  }
  return true;
}

MapserverStat::MapserverStat(ros::NodeHandle& nh)
    : Mapserver(&nh),
      n(nh) {

  // Print the MRPT look-up table values
  if (debug) {
    MapserverStat::printMrptLookUpTable();
  }

  // Check the other parameters
  this->n.param<std::string>("debug_ism_topic", debugIsmTopic,
                             "/ism/radar/tracking/radar_return");  // The topic of the ISM which is resend as transformed ISM in the mapserver frame
  this->n.param<std::string>("debug_topic", debugTopic, "/amiro2/ism/cam");  // The topic of the fused map to show via opencv
  this->n.param<float>("max_occupancy_update_certainty",
                       maxOccupancyUpdateCertainty, 0);  // Maximum update uncertainty
  this->n.param<float>("max_distance_insertion", maxDistanceInsertion, 0);  // Maximum distance insertion
  this->n.param<float>("uncertainty_boundary", uncertaintyBoundary, 0.5);  // Uncertainty boundary for displaying a feature (standard: 0.5 (unknown))
  this->n.param<std::string>("topic_debug_grid_prefix", topicDebugGridPrefix,
                             "/viz");

  if (mapInitValue > uncertaintyBoundary)
    ROS_WARN_STREAM(
        "'uncertaintyBoundary' is less than 'mapInitValue', display might be corrupted");

  // Define the properties of one occupancy map
  def.resolution = this->resolution_mPerTile;
  def.insertionOpts.maxOccupancyUpdateCertainty = this
      ->maxOccupancyUpdateCertainty;
  def.insertionOpts.maxDistanceInsertion = this->maxDistanceInsertion;
  def.max_x = this->maxX_m;
  def.min_x = this->minX_m;
  def.max_y = this->maxY_m;
  def.min_y = this->minY_m;

  // Parse the blind spots
  MapserverStat::getBlindSpots(n, blindSpots);

  // Allocate space for the map and mapstacks
  this->currentMapStack = std::shared_ptr<
      std::map<std::string, mrpt::maps::COccupancyGridMap2D*>>(
      new std::map<std::string, mrpt::maps::COccupancyGridMap2D*>);
  this->lastMapStack = std::shared_ptr<
      std::map<std::string, mrpt::maps::COccupancyGridMap2D*>>(
      new std::map<std::string, mrpt::maps::COccupancyGridMap2D*>);

  publisherIsmAsPointCloud = this->n.advertise<sensor_msgs::PointCloud>("foo",
                                                                        1);
  publisherIsmAsOgm = this->n.advertise<nav_msgs::OccupancyGrid>(
      std::string("debug") + debugIsmTopic, 1);

  // Prepare ROS service
  // TODO: Do this on demand for given subscribed topics
  const std::string s("/");

  //  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::compressedMapImage , mapServerCompressedMap);
  //  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::stockEdge , mapServerStockEdge);
  //  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::singleLayerOgm , mapServerSingleLayerOgm);
//      service_mapStack = this->n.advertiseService(reqTopicMapStack/*scopes::map::statServer::parent + s + scopes::map::statServer::requests::mapStack*/, &MapserverStat::mapStatServerMapStack);
  service_mapStack =
      n.advertiseService(
          reqTopicMapStack/*scopes::map::statServer::parent + s + scopes::map::statServer::requests::mapStack*/,
          &MapserverStat::mapStatServerMapStack, this);
  service_singleLayerOgm = n.advertiseService(
      scopes::map::ogmServer::parent + s
          + scopes::map::ogmServer::requests::singleLayerOgm,
      &MapserverStat::mapStatServerSingleLayerOgm, this);
}

void MapserverStat::getBlindSpots(ros::NodeHandle &n, BlindSpots &blindSpots) {

  ROS_INFO_STREAM("Start blindspot parsing");
  std::string blinspotPrefix("blindspot_");
  std::vector<std::string> keys;
  n.getParamNames(keys);
  for (auto key = keys.begin(); key < keys.end(); ++key) {

    // Continue if the parameter is not in our namespace
    try {
      if (key->compare(0, n.getNamespace().size(), n.getNamespace()) != 0) {
        ROS_DEBUG_STREAM(
            *key << " is not a parameter in the " << n.getNamespace() << " namespace => continue");
        continue;
      }
    } catch (...) {
      continue;
    }
    // Get rid of the namespace and check for the correct prefix
    std::vector<std::string> strs;
    boost::split(strs, *key, boost::is_any_of("/"));
    if (strs.back().size() < blinspotPrefix.size()) {
      ROS_DEBUG_STREAM(*key << " to short");
      continue;
    } else {
      ROS_DEBUG_STREAM(*key << " matches in length");
    }

    // Parse the blindspot parameters
    if (strs.back().compare(0, blinspotPrefix.size(), blinspotPrefix) == 0) {
      try {
        ROS_DEBUG_STREAM(*key << " matches");
        XmlRpc::XmlRpcValue blindSpotList;
        n.getParam(*key, blindSpotList);
        ROS_ASSERT(blindSpotList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(blindSpotList.size() == 5);  // s.t. 5 is the number of possible entries [p1x, p1y, p2x, p2y, frame_id]
        ROS_ASSERT(
            blindSpotList[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(
            blindSpotList[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        tf::Pose p1 = tf::Pose(
            tf::Quaternion(0, 0, 0, 1),
            tf::Vector3(tfScalar(static_cast<double>(blindSpotList[0])),
                        tfScalar(static_cast<double>(blindSpotList[1])),
                        tfScalar(0)));
        ROS_ASSERT(
            blindSpotList[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(
            blindSpotList[3].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        std::cerr << std::endl;
        tf::Pose p2 = tf::Pose(
            tf::Quaternion(0, 0, 0, 1),
            tf::Vector3(tfScalar(static_cast<double>(blindSpotList[2])),
                        tfScalar(static_cast<double>(blindSpotList[3])),
                        tfScalar(0)));
        ROS_ASSERT(
            blindSpotList[4].getType() == XmlRpc::XmlRpcValue::TypeString);
        tf::Stamped<tf::Pose> ps1 = tf::Stamped<tf::Pose>(
            p1, ros::Time(0.0), static_cast<std::string>(blindSpotList[4]));
        tf::Stamped<tf::Pose> ps2 = tf::Stamped<tf::Pose>(
            p2, ros::Time(0.0), static_cast<std::string>(blindSpotList[4]));
        BlindSpot pt = std::make_tuple(ps1, ps2);
        ROS_INFO_STREAM(
            "Parsing: " << "[p1x: " << ps1.getOrigin().getX() << "], " << "[p1y: " << ps1.getOrigin().getY() << "], " << "[p2x: " << ps2.getOrigin().getX() << "], " << "[p2y: " << ps2.getOrigin().getY() << "], " << "[frame_id: " << ps1.frame_id_ << "]");
        blindSpots.push_back(pt);
      } catch (XmlRpc::XmlRpcException a) {
        std::cerr << "XmlRpc exception: " << a.getMessage() << std::endl;
      }
    } else {
      ROS_DEBUG_STREAM(*key << " does not match");
    }
  }
  ROS_INFO_STREAM("End blindspot parsing");
}

void MapserverStat::spinOnce() {
  // Plot map
  //      boost::shared_ptr<cv::Mat> image(doColorMapCallback());
  //      cv::imshow( "Current View", *image );                    // Show our image inside it.
  //      cv::waitKey(1); // Update the window
  if (debug) {
    try {
      //            std::shared_ptr<cv::Mat> image(mrptOggToGrayScale(*currentMapStack->at(debugTopic)));
      //            std::shared_ptr<cv::Mat> image(Mapserver::rosOccToGrayScale(this->msgDebug));
      //              if (image) {
      //                  cv::flip(*image, *image, 0);
      //                  cv::imshow( "Current View", *image);
      //                  cv::waitKey(1); // Update the window
      //              }
    } catch (...) {
      ROS_WARN("Debug visualization: No such topic '%s' in currentMapStack",
               debugTopic.c_str());
    }
    // Publish the maps
    std::vector<std::string> foo;
    tf::Vector3 trans = tf::Vector3(tfScalar(-(maxX_m - minX_m) / 2.0f),
                                    tfScalar(-(maxY_m - minY_m) / 2.0f),
                                    tfScalar(0.0f));
    std::shared_ptr<tf::Pose> poseOffset = std::shared_ptr<tf::Pose>(
        new tf::Pose(tf::Quaternion(0, 0, 0, 1), trans));
    std::string reference = currentTileTfName + tileOriginTfSufixForRoiOrigin;
    formatAndSendGrid(foo, reference, currentMapStack, n, topicDebugGridPrefix);
  }

  //      showRpc();

  // Add new subscribers
  this->advertiseSubscribers(subIsmList, &MapserverStat::doIsmFusion, this,
                             topicPrefix, debug);
}

// Translate a map
void MapserverStat::translateMap(
    mrpt::maps::COccupancyGridMap2D &map, int offsetx, int offsety,
    mrpt::maps::COccupancyGridMap2D::cellType fillProbability_logodds) {

  // Define a transformation for the image
  cv::Mat trans_mat =
      (cv::Mat_<double>(2, 3) << 1, 0, static_cast<double>(offsetx), 0, 1, static_cast<double>(offsety));

  // Get the raw data as an image
  cv::Mat mapAsImage(static_cast<int>(map.getSizeY()),
                     static_cast<int>(map.getSizeX()),
#if defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS)
                     CV_8SC1
#else // defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
                     CV_16SC1
#endif
                     ,
                     (void*) &map.getRawMap()[0]);

  // Warp the image (which refers to the map)
  cv::warpAffine(
      mapAsImage,
      mapAsImage,
      trans_mat,
      cv::Size(static_cast<int>(map.getSizeX()),
               static_cast<int>(map.getSizeY())),
      cv::INTER_NEAREST, cv::BORDER_CONSTANT,
      cv::Scalar(static_cast<double>(fillProbability_logodds)));
}

// Set all map tiles of all maps to the given value
void MapserverStat::fillMapStack(
    std::vector<mrpt::maps::COccupancyGridMap2D> &mapStack,
    mrpt::maps::COccupancyGridMap2D::cellType fillValue) {
  for (auto idx = mapStack.begin(); idx < mapStack.end(); ++idx) {
    idx->fill(mrpt::maps::COccupancyGridMap2D::l2p(fillValue));
  }
}

// Set all map tiles of all maps to the given value
void MapserverStat::fillMapStack(
    std::vector<mrpt::maps::COccupancyGridMap2D> &mapStack, float fillValue) {
  fillMapStack(mapStack, mrpt::maps::COccupancyGridMap2D::p2l(fillValue));
}

// Set map tiles of maps to the given value
void MapserverStat::fillMap(
    mrpt::maps::COccupancyGridMap2D &map,
    mrpt::maps::COccupancyGridMap2D::cellType fillValue) {
  map.fill(mrpt::maps::COccupancyGridMap2D::l2p(fillValue));
}

void* MapserverStat::getRawData(mrpt::maps::COccupancyGridMap2D *map) {
  return (void*) map->getRawMap().data();
}

void MapserverStat::mapStorage(
    const std::shared_ptr<
        std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> &mapStack,
    const std::string prefixString, const std::string formatString,
    const std::string formatUnitString, const double resolution_meterPerTile,
    const ros::Time timestamp) {

  const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> dummyMap;
  const tf::StampedTransform dummyTf;
  const mrpt::maps::COccupancyGridMap2D::cellType dummy = 0;

  mapRefreshAndStorage(mapStack,                     // Map to shift/store/reset
      dummyMap,                                 // Nothing at all
      dummyTf,                                  // Transform
      std::string("OGM"),                       // Kind of map
      std::string(""),               // Format (empty: Take from type specifier)
      std::string("logodds"),                   // Unit
      resolution_meterPerTile,                  // Resolution per tile
      true,                                     // Maps should be stored
      false,                                    // Maps should be shifted
      false,                                    // Map should be reseted reset
      dummy,                                    // Some float value
      false,                                 // Don't store the current position
      std::string(""), timestamp);
}
