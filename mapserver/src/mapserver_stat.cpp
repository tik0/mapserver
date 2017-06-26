#include <mapserver_stat.hpp>


// Add just some picture of each other
boost::shared_ptr<cv::Mat> MapserverStat::doColorMapCallback(std::vector<mrpt::maps::COccupancyGridMap2D> mapStack) {

  // TODO Define this as a global variable so that it dows not
  // have to allocate space on every call
  boost::shared_ptr<cv::Mat> dst(new cv::Mat(mapSizeX, mapSizeY, CV_8UC3));
  dst->setTo(cv::Scalar(127,127,127)); // to set all values to 127 (aka unknown)

  const float validCellValue = uncertaintyBoundary;
  for (uint mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    for (int idy = 0; idy < mapSizeY; ++idy) {
      for (int idx = 0; idx < mapSizeX; ++idx) {
        if (mapStack[mapIdx].getCell(idx, idy) > validCellValue) {
          dst->at<cv::Vec3b>(idy,idx)[0] = mappingLayers::mapColorBGR[mapIdx][0];
          dst->at<cv::Vec3b>(idy,idx)[1] = mappingLayers::mapColorBGR[mapIdx][1];
          dst->at<cv::Vec3b>(idy,idx)[2] = mappingLayers::mapColorBGR[mapIdx][2];
        }
      }
    }
  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
}


std::shared_ptr<cv::Mat> MapserverStat::mrptOggToGrayScale(mrpt::maps::COccupancyGridMap2D &map) {

  std::shared_ptr<cv::Mat> dst(new cv::Mat(map.getSizeY(), map.getSizeX(), CV_8UC1));
//  dst->setTo(cv::Scalar(127,127,127)); // to set all values to 127 (aka unknown)


  for (int idx = 0; idx < map.getRawMap().size(); ++idx) {
    dst->at<uchar>(idx) = mrpt::maps::COccupancyGridMap2D::l2p_255(map.getRawMap().at(idx));
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

std::shared_ptr<cv::Mat> MapserverStat::rosOggToGrayScale(nav_msgs::OccupancyGrid::ConstPtr map) {

  std::shared_ptr<cv::Mat> dst;

  if (map) {
    if (map->info.width > 0 && map->info.height > 0) {
      dst = std::shared_ptr<cv::Mat>(new cv::Mat(map->info.height, map->info.width, CV_8UC1));
//      dst->setTo(cv::Scalar(127,127,127)); // to set all values to 127 (aka unknown)

      for (int idx = 0; idx < map->data.size(); ++idx) {
        const int oggValue = int(map->data.at(idx));
        const int intensity = oggValue < 0 ? 50 : oggValue;
        dst->at<uchar>(idx) = uchar(float(intensity) * 2.55);
      }
    }
  }

//  DEBUG_MSG("Server returns map")
//   cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
}

///
/// \brief Returns the pose in the desired frame
/// \param poseInSourceFrame Stamped pose in the source frame
/// \param targetFrame The target frame in which the returned pose will reside
/// \param tfListener A transformer
/// \return Transformed pose. Shared pointer is empty if an error occurs
///
std::shared_ptr<tf::Stamped<tf::Pose>> MapserverStat::getPoseInFrame(const tf::Stamped<tf::Pose> &poseInSourceFrame, const std::string &targetFrame, const tf::TransformListener &tfListener) {

  std::shared_ptr<tf::Stamped<tf::Pose>> poseInTargetFrameReturn;
  tf::Stamped<tf::Pose> poseInTargetFrame;
  // Use source frame as target frame if empty
  const bool useSrcFrameAsDstFrame = targetFrame.empty() || !targetFrame.compare(poseInSourceFrame.frame_id_) ? true : false;
  const std::string srcFrame = poseInSourceFrame.frame_id_;
  const std::string dstFrame = useSrcFrameAsDstFrame ? srcFrame : targetFrame;
  // Get the origin of the pose in the target frame
  bool tfSuccess = true;
  if (!useSrcFrameAsDstFrame) { // We don't need this, if we stay in the same frame
    try {
      // HACK We don't wait for tf messages in the past, because they will never arrive at all
      std::string errorMsg;
      const std::string errorMsgWorthitToWaitFor("Lookup would require extrapolation into the future");
      tfListener.canTransform(dstFrame, srcFrame, poseInSourceFrame.stamp_, &errorMsg);
      std::size_t found = errorMsg.find(errorMsgWorthitToWaitFor);
      if (found != std::string::npos || errorMsg.empty()) {
          if (found != std::string::npos) {
              ROS_DEBUG_STREAM("getPoseInFrame: We'll wait for tf transform messages in the future: " << errorMsg);
              tfListener.waitForTransform(dstFrame, srcFrame, poseInSourceFrame.stamp_, ros::Duration(3.0));
          }
          tfListener.transformPose(dstFrame, poseInSourceFrame, poseInTargetFrame);
      } else {
          throw std::runtime_error(errorMsg);
      }
    } catch(const std::exception &exc) {
      const std::string excStr(exc.what());
      ROS_ERROR("getPoseInFrame (%s -> %s): %s", srcFrame.c_str(), dstFrame.c_str(), excStr.c_str());
      tfSuccess = false;
    }
  } else {
      poseInTargetFrame = poseInSourceFrame;
  }

  // Return a filled shared pointer if tf was successful
  if (tfSuccess) {
      poseInTargetFrameReturn = std::make_shared<tf::Stamped<tf::Pose>>(poseInTargetFrame);
  }
  return poseInTargetFrameReturn;
}

///
/// \brief Calculate the metric coordinates of OGM cell centroids in the target frame
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetFrame The target frame in which the the point is calculated (target frame equals OGM frame if empty)
/// \param tfListener A transformer
/// \param idxStart Start index in width
/// \param idyStart Start index in height
/// \param idxWidth Amount of cells in x direction
/// \param idyHeight Amount of cells in y direction
/// \return Vector of points in row major order. Shared pointer is empty if an error occurs
///
std::shared_ptr<std::vector<tf::Point>> MapserverStat::ogmCellCoordinatesToPoints(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                                   const std::string &targetFrame,
                                                                   const tf::TransformListener &tfListener,
                                                                   const std::size_t &idxStart,
                                                                   const std::size_t &idyStart,
                                                                   const std::size_t &idxWidth,
                                                                   const std::size_t &idyHeight) {

  std::shared_ptr<std::vector<tf::Point>> points;
  // Sanity checks
  if(!ogm) {
      ROS_ERROR("OGM is empty");
  } else if((idxStart + idxWidth) > ogm->info.width || (idyStart + idyHeight) > ogm->info.height || idxWidth == 0 || idyHeight == 0) {
      ROS_ERROR("Requested range out of bound");
  } else {
      // Get the pose in the target frame
      tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
      std::shared_ptr<tf::Stamped<tf::Pose>> ogmOriginInTargetFrame = getPoseInFrame(tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp, ogm->header.frame_id), targetFrame, tfListener);
      // Start calculating the transform
      if (ogmOriginInTargetFrame) {
          points = std::shared_ptr<std::vector<tf::Point>>(new std::vector<tf::Point>(idxWidth * idyHeight));
          const float cornerToCenterOffset = ogm->info.resolution / 2.0f;

          auto pointsIter = points->begin();
          // tf::Stamped<tf::Point> ptInOgmFrame(tf::Point(0,0,0), ogm->header.stamp, ogm->header.frame_id); // The metric point of a OGM cell in the OGM frame
          for (int idy = idyStart; idy < idyStart + idyHeight ; ++idy) {
              for (int idx = idxStart; idx < idxStart + idxWidth; ++idx, ++pointsIter) {
                  const tf::Point ptInOgmOrigin(float(idx) * ogm->info.resolution + cornerToCenterOffset,
                                                float(idy) * ogm->info.resolution + cornerToCenterOffset,
                                                0.0f);
                  // ptInOgmOrigin.setZ(0.0f); // Don't need this
                  // ptInOgmFrame = ogmOriginInSourceFrame * ptInOgmOrigin; // We don't need this
                  const tf::Point ptInTargetFrame = *ogmOriginInTargetFrame * ptInOgmOrigin;
                  *pointsIter = ptInTargetFrame;
              }
          }
      } else {
          ROS_WARN("TF unsuccessful");
      }
  }
  return points;
}

///
/// \brief Converts a OGM to a point cloud Calculate the metric coordinates of OGM cell centroids in the target frame
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetFrame The target frame in which the point is calculated (target frame equals OGM frame if empty)
/// \param tfListener A transformer
/// \param idxStart Start index in width
/// \param idyStart Start index in height
/// \param idxWidth Amount of cells in x direction
/// \param idyHeight Amount of cells in y direction
/// \return OGM as point cloud. The OGM range [-1, 0 .. 100] is converted to [-0.01f, 0 .. 1.0f]. If message pointer is empty, an error occurred
///
sensor_msgs::PointCloud::Ptr MapserverStat::ogmCellsToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                                   const std::string &targetFrame,
                                                                   const tf::TransformListener &tfListener,
                                                                   const std::size_t &idxStart,
                                                                   const std::size_t &idyStart,
                                                                   const std::size_t &idxWidth,
                                                                   const std::size_t &idyHeight) {
  sensor_msgs::PointCloud::Ptr ogmAsPointCloud;
  // First get the metric coordinates
  std::shared_ptr<std::vector<tf::Point>> points = ogmCellCoordinatesToPoints(ogm, targetFrame, tfListener, idxStart, idyStart, idxWidth , idyHeight);
  // Sanity checks: We rely on ogmCellCoordinatesToPoints
  if (points) {
      ogmAsPointCloud = boost::shared_ptr<sensor_msgs::PointCloud>(new sensor_msgs::PointCloud);
      ogmAsPointCloud->header = ogm->header;
      ogmAsPointCloud->header.frame_id = targetFrame.empty() ? ogmAsPointCloud->header.frame_id : targetFrame;
      ogmAsPointCloud->points.resize(points->size());
      ogmAsPointCloud->channels.resize(1);
      ogmAsPointCloud->channels.at(0).name = "OGM";
      ogmAsPointCloud->channels.at(0).values.resize(points->size());
      auto pointCloudValueIter = ogmAsPointCloud->channels.at(0).values.begin();
      auto pointCloudPointIter = ogmAsPointCloud->points.begin();
      auto pointIter = points->begin();
      for (int idy = idyStart; idy < idyStart + idyHeight ; ++idy) {
          for (int idx = idxStart; idx < idxStart + idxWidth; ++idx, ++pointCloudValueIter, ++pointCloudPointIter, ++pointIter) {
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

void MapserverStat::correctInvalidOrientation(tf::Pose &pose) {
  if (pose.getRotation().x() < tf::QUATERNION_TOLERANCE &&
      pose.getRotation().y() < tf::QUATERNION_TOLERANCE &&
      pose.getRotation().z() < tf::QUATERNION_TOLERANCE &&
      pose.getRotation().w() < tf::QUATERNION_TOLERANCE) {
      ROS_WARN_ONCE("correctInvalidOrientation: Pose with quaternion(0,0,0,0) detected. Interpretation as (0,0,0,1)");
      pose.setRotation(tf::Quaternion(0,0,0,1));
    }
}

///
/// \brief Returns the coordinates of the four corner points of the OGM in the given frame
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetFrame The target frame in which the point is calculated (target frame equals OGM frame if empty)
/// \param tfListener A transformer
/// \return Vector of points starting top-left. Shared pointer is empty if an error occurs
///
std::shared_ptr<std::vector<tf::Point>> MapserverStat::getOgmCornerPoints(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                           const std::string &targetFrame,
                                                           const tf::TransformListener &tfListener) {

  std::shared_ptr<std::vector<tf::Point>> points;
  // Sanity checks
  if(!ogm) {
      ROS_ERROR("OGM is empty");
  } else {
      // Get the pose in the target frame
      tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
      correctInvalidOrientation(ogmOriginPose);
      std::shared_ptr<tf::Stamped<tf::Pose>> ogmOriginInTargetFrame = getPoseInFrame(tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp, ogm->header.frame_id), targetFrame, tfListener);
      // Start calculating the transform
      if (ogmOriginInTargetFrame) {
          const std::size_t numCorners = 4;
          points = std::shared_ptr<std::vector<tf::Point>>(new std::vector<tf::Point>(numCorners));
          // pointInTargetFrame = transformMatrixFromTargetFrameToSourceFrame * pointInSourceFrame;
          points->at(0) = *ogmOriginInTargetFrame * tf::Point(0.0f, 0.0f, 0.0f);
          points->at(1) = *ogmOriginInTargetFrame * tf::Point(ogm->info.width * ogm->info.resolution, 0.0f, 0.0f);
          points->at(2) = *ogmOriginInTargetFrame * tf::Point(ogm->info.width * ogm->info.resolution, ogm->info.height * ogm->info.resolution, 0.0f);
          points->at(3) = *ogmOriginInTargetFrame * tf::Point(0.0f, ogm->info.height * ogm->info.resolution, 0.0f);
      } else {
          ROS_WARN("TF unsuccessful");
      }
  }
  return points;
}

///
/// \brief Returns the coordinates of the four corner points of the OGM in the given pose
/// \param ogm The OGM which cell coordinates are converted to points
/// \param targetPose The target Pose in which the point is calculated
/// \param tfListener A transformer
/// \return Vector of points starting top-left. Shared pointer is empty if an error occurs
///
std::shared_ptr<std::vector<tf::Point>> MapserverStat::getOgmCornerPoints(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                                           const tf::Stamped<tf::Pose> &targetPose,
                                                           const tf::TransformListener &tfListener) {
  std::shared_ptr<std::vector<tf::Point>> points;
  // Get pose of the OGM in the target frame
  std::shared_ptr<tf::Stamped<tf::Pose>> targetPoseInOgmFrame = getPoseInFrame(targetPose, ogm->header.frame_id, tfListener);
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


///
/// \brief Resize a OGM to the desired resolution
/// \param ogm The OGM which should be resized
/// \param targetResolution The target resolution of the OGM
/// \return Resized OGM
///
nav_msgs::OccupancyGrid::ConstPtr MapserverStat::ogmResize(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                   const float &targetResolution) {

  nav_msgs::OccupancyGrid::Ptr ogmTf = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);

  const float scaling = ogm->info.resolution / targetResolution;
  // Get data as OpenCV
  // HACK Type should be CV_8SC1, but resize function only works with CV_8UC1
  const cv::Mat ogmCv(ogm->info.height, ogm->info.width, CV_8UC1, (void*)(ogm->data.data()));
  cv::Mat ogmCvResized;
  // Resize
  cv::resize(ogmCv, ogmCvResized, cv::Size(), scaling, scaling, cv::INTER_LINEAR);
  // Copy back
  ogmTf->header = ogm->header;
  ogmTf->info = ogm->info;
  ogmTf->info.height = ogmCvResized.rows;
  ogmTf->info.width = ogmCvResized.cols;
  ogmTf->info.resolution = targetResolution;
  ogmTf->data.resize(ogmCvResized.rows * ogmCvResized.cols);
  memcpy((void*)ogmTf->data.data(), (void*)ogmCvResized.data,ogmCvResized.rows * ogmCvResized.cols);

  return ogmTf;
}

///
/// \brief Warps a OGM to the desired pose and resolution such that the new OGM is aligned with this pose
/// \param ogm The OGM which should be transformed
/// \param targetPose The target pose in which the new OGM resides
/// \param targetResolution The target resolution of the OGM
/// \param tfListener A transformer
/// \param resetPoseToOgmBoundary The origin of the new OGM will not reside in targetOrigin, but with respect to the minimum possible size of the which pose lies in the XY-plane of targetOrigin.
/// \return OGM in the new frame. If message pointer is empty, an error occurred
///
nav_msgs::OccupancyGrid::ConstPtr MapserverStat::ogmTf(const nav_msgs::OccupancyGrid::ConstPtr ogm,
                                   const tf::Stamped<tf::Pose> &targetOrigin,
                                   const float &targetResolution,
                                   const tf::TransformListener &tfListener,
                                   const bool resetPoseToOgmBoundary) {

  nav_msgs::OccupancyGrid::Ptr ogmTf;

  // TODO Check for XY shift, which can be easily calculated (Just shift the pose in xy and maybe check for resolution)



  // Sanity check for invalid orientation
  tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
  correctInvalidOrientation(ogmOriginPose);

  // First check, if pose and resolution are the same, to minimize workload
  if (ogmOriginPose == targetOrigin) {
      const bool resolutionIsSame = utils::compare(targetResolution, ogm->info.resolution, std::min(ogm->info.resolution, targetResolution) / 2 );
      if (!resolutionIsSame) { // Scale the OGM
          ROS_DEBUG("Pose is the same: Just change the resolution of the OGM");
          return ogmResize(ogm, targetResolution);
      } else {
          ROS_DEBUG("Pose and resolution are the same: Just return the original OGM");
          return ogm;
      }
  } else { // Do a full transform of the OGM
    // First get the metric coordinates of the four corner points in the target and source frame
    std::shared_ptr<std::vector<tf::Point>> pointsInTargetFrame = getOgmCornerPoints(ogm, targetOrigin, tfListener);
//    tf::Pose ogmOriginPose; tf::poseMsgToTF(ogm->info.origin, ogmOriginPose);
    std::shared_ptr<std::vector<tf::Point>> pointsInSourceFrame = getOgmCornerPoints(ogm, tf::Stamped<tf::Pose>(ogmOriginPose, ogm->header.stamp, ogm->header.frame_id), tfListener);
    // Calculate the homography
    if (pointsInTargetFrame && pointsInSourceFrame) {
        // Load OGM as OpenCV image
        // HACK Type should be CV_8SC1, but resize function only works with CV_8UC1
        cv::Mat ogmCv(ogm->info.height, ogm->info.width, CV_8UC1, (void*)(ogm->data.data()));

        std::vector<cv::Point2f> pointsSource(4), pointsTarget(4);
        pointsSource.at(0) = cv::Point2f(float(0),float(0));
        pointsSource.at(1) = cv::Point2f(float(ogm->info.width),float(0));
        pointsSource.at(2) = cv::Point2f(float(ogm->info.width),float(ogm->info.height));
        pointsSource.at(3) = cv::Point2f(float(0),float(ogm->info.height));

        pointsTarget.at(0) = cv::Point2f(float(pointsInTargetFrame->at(0).x() / targetResolution),float(pointsInTargetFrame->at(0).y() / targetResolution));
        pointsTarget.at(1) = cv::Point2f(float(pointsInTargetFrame->at(1).x() / targetResolution),float(pointsInTargetFrame->at(1).y() / targetResolution));
        pointsTarget.at(2) = cv::Point2f(float(pointsInTargetFrame->at(2).x() / targetResolution),float(pointsInTargetFrame->at(2).y() / targetResolution));
        pointsTarget.at(3) = cv::Point2f(float(pointsInTargetFrame->at(3).x() / targetResolution),float(pointsInTargetFrame->at(3).y() / targetResolution));

        // If the pose will be reset to the boundary
        tf::Stamped<tf::Pose> targetOriginNew = targetOrigin;
        if (resetPoseToOgmBoundary) {
            const cv::Point2f xyMinPoint = utils::getXYMin(pointsTarget);
            tf::Pose resetPose(tf::Quaternion(0,0,0,1), tf::Vector3(tfScalar(xyMinPoint.x * targetResolution), tfScalar(xyMinPoint.y * targetResolution), tfScalar(0)));
            pointsTarget.at(0) -= xyMinPoint;
            pointsTarget.at(1) -= xyMinPoint;
            pointsTarget.at(2) -= xyMinPoint;
            pointsTarget.at(3) -= xyMinPoint;
            targetOriginNew *= resetPose;
        }

        // Do the warping to get the PGM in the new pose
        cv::Mat H = cv::findHomography(  pointsSource,pointsTarget,0 );
        const cv::Point2f xyMaxPoint = utils::getXYMax(pointsTarget);
        cv::Mat ogmCvWarped(xyMaxPoint.y, xyMaxPoint.x, CV_8SC1);
        // Warp the OGM with linear interpolation (Boarders are set to unknown s.t. 50)
        cv::warpPerspective(ogmCv,ogmCvWarped,H,ogmCvWarped.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(50));

        // Copy back
        ogmTf = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);
        ogmTf->header = ogm->header;
        ogmTf->header.frame_id = targetOriginNew.frame_id_;
        geometry_msgs::PoseStamped poseMsg;
        tf::poseStampedTFToMsg(targetOriginNew, poseMsg);
        ogmTf->info.origin = poseMsg.pose;
        ogmTf->info.height = ogmCvWarped.rows;
        ogmTf->info.width = ogmCvWarped.cols;
        ogmTf->info.resolution = targetResolution;
        ogmTf->data.resize(ogmCvWarped.rows * ogmCvWarped.cols);
        memcpy((void*)ogmTf->data.data(), (void*)ogmCvWarped.data,ogmCvWarped.rows * ogmCvWarped.cols);
    } else {
        ROS_WARN("pointsInTargetFrame or pointsInSourceFrame are empty");
    }
  }
  return ogmTf;

}

void MapserverStat::calcIsm4Mapserver(const double xIsmCenter_m, const double yIsmCenter_m, const double phiIsm_rad,
                const double ismResolution, cv::Mat &ismInRoi, cv::Mat &ism) {

  const double roiResolution = resolution_mPerTile;
  double ismMagnificationFactor = ismResolution / roiResolution;
  const int32_t x_dis_r = xIsmCenter_m / roiResolution;
  const int32_t y_dis_r = yIsmCenter_m / roiResolution;

  // Handle axis flipping
//  utils::rot90(ism,ismFlipcode);

  // Allocate space
  const cv::Size roiSizePx = ismInRoi.size();
  const cv::Point2f centerRoi(roiSizePx.height / 2.0f, roiSizePx.width /2.0f);
  ismInRoi = cv::Mat(roiSizePx, ismInRoi.type(), cv::Scalar(50/*% Unknown*/));  // Flush input image

  // Put the ISM in a bigger rectangular image, to handle rotation without cropping
  int newIsmRowsCols = sqrt(pow(ism.rows,2) + pow(ism.cols,2));
  const cv::Point2f ismOldCenter(ism.size().height / 2.0f, ism.size().width /2.0f);
  cv::Mat ismContainer(newIsmRowsCols,newIsmRowsCols, ism.type(), cv::Scalar(50 /*% Unknown*/));
  cv::Rect ismInNewIsm((newIsmRowsCols - ism.cols) / 2, (newIsmRowsCols - ism.rows) / 2, ism.cols, ism.rows);
  ism.copyTo(ismContainer(ismInNewIsm));

  // Resize ISM
  cv::Size newIsmSize(ismContainer.size().width * ismMagnificationFactor,
                      ismContainer.size().height * ismMagnificationFactor);
  cv::Mat ismResized;
  cv::resize(ismContainer,ismResized, newIsmSize, 0, 0, cv::INTER_NEAREST);

  // Rotate the ISM around its center
  const cv::Point2f centerISM(ismResized.size().height / 2.0f, ismResized.size().width /2.0f);
  cv::Mat rotation( 2, 3, CV_32FC1 );
  rotation = cv::getRotationMatrix2D( centerISM, - phiIsm_rad * rad2deg , 1.0f/*ZoomFactor*/ ); // Negative angle, because OpenCV is CW regarding our convention
  cv::Mat ismRotated;
  cv::warpAffine( ismResized, ismRotated, rotation, ismResized.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(50 /*% Unknown*/));

  // Copy the rotated ISM to its translated position in the ROI
  const cv::Point2f centerRotatedISM(ismRotated.size().height / 2.0f, ismRotated.size().width /2.0f);
  cv::Rect ismInRoiRect = cv::Rect(centerRoi.x + x_dis_r - centerRotatedISM.x, centerRoi.y  + y_dis_r - centerRotatedISM.y, ismRotated.cols, ismRotated.rows);
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
nav_msgs::OccupancyGrid::Ptr MapserverStat::oggTf(const std::string &targetFrame, const nav_msgs::OccupancyGrid::ConstPtr ismRos, const double targetRes)
{

  nav_msgs::OccupancyGrid::Ptr oggTrans;

  // Get the transform between the frames
  tf::StampedTransform tfOcc;
  try {
    listenerTf->waitForTransform(ismRos->header.frame_id, targetFrame, ros::Time(0.0), ros::Duration(3.0));
    listenerTf->lookupTransform(ismRos->header.frame_id, targetFrame, ros::Time(0.0), tfOcc);
  } catch(const std::exception &exc) {
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
  double phi_rad = utils::conversion::quaternion2eulerYaw(tfOcc.getRotation().getW(), tfOcc.getRotation().getX(), tfOcc.getRotation().getY(), tfOcc.getRotation().getZ());

  // Allocate the new Occ TODO: This is to sloppy, need propper definition
  oggTrans = boost::shared_ptr<nav_msgs::OccupancyGrid>(new nav_msgs::OccupancyGrid);
  oggTrans->header = ismRos->header;
  oggTrans->header.frame_id = targetFrame;
  oggTrans->info.resolution = targetRes < 0.0 ? ismRos->info.resolution : targetRes;
//  oggTrans->info.origin = ???
  const int width = (maxX_m - minX_m) / oggTrans->info.resolution;
  const int height = (maxY_m - minY_m) / oggTrans->info.resolution;
  oggTrans->info.width = width;
  oggTrans->info.height = height;

  // Allocate opencv images to do the work
  cv::Mat ismInRoi(height,width,CV_8SC1);
  cv::Mat ism(ismRos->info.height, ismRos->info.width, CV_8SC1, (void*)(&ismRos->data[0]));


  ROS_DEBUG("TF.xyz to %s: %f %f %f", ismRos->header.frame_id.c_str(), tfOcc.getOrigin().x(), tfOcc.getOrigin().y(), tfOcc.getOrigin().z());
  ROS_DEBUG("ismRos->info.origin.position.x,y: %f %f", ismRos->info.origin.position.x, ismRos->info.origin.position.y);

  // Calculate the tranlation from the base_footprint to the ISM origin
  // assuming that the coordinate systems are aligned after REP-103, and
  // that the height does not care
  const double xBasefootprint2Ism_m = tfOcc.getOrigin().x() + ismRos->info.origin.position.x;
  const double yBasefootprint2Ism_m = tfOcc.getOrigin().y() + ismRos->info.origin.position.y;
  const double ismWidthX_m = ismRos->info.width * ismRos->info.resolution;
  const double ismHeightY_m = ismRos->info.height * ismRos->info.resolution;
  double fz_rad = utils::conversion::quaternion2eulerYaw(ismRos->info.origin.orientation.w, ismRos->info.origin.orientation.x,
                                                  ismRos->info.origin.orientation.y, ismRos->info.origin.orientation.z);
  ROS_DEBUG("Euler orientation z (deg) %f",fz_rad*rad2deg);

  // Get the Center of the ISM after the rotation, assuming that we only operate in the plane
  double ismCenterX_m = ismWidthX_m / 2.0;
  double ismCenterY_m = ismHeightY_m / 2.0;
  utils::conversion::rotXY(ismCenterX_m, ismCenterY_m, fz_rad);
  ROS_DEBUG("ismCenterXY_m %f %f", ismCenterX_m, ismCenterY_m);

  // Rotate the center of the ISM in the footprint
  double xBasefootprint2IsmCenter_m = xBasefootprint2Ism_m + ismCenterX_m;
  double yBasefootprint2IsmCenter_m = yBasefootprint2Ism_m + ismCenterY_m;
  utils::conversion::rotXY(xBasefootprint2IsmCenter_m, yBasefootprint2IsmCenter_m, phi_rad);

  // Add the position inside the ROI
  const double xRoi2IsmCenter = xBasefootprint2IsmCenter_m + x;
  const double yRoi2IsmCenter = yBasefootprint2IsmCenter_m + y;
  // Get the final orientation of the ISM
  double ismOrientation = fz_rad + phi_rad; // TODO Check if fz needs to be assigned with minus

  // Merge ROS-ISM into RSB-ISM
  calcIsm4Mapserver(xRoi2IsmCenter, yRoi2IsmCenter, ismOrientation,
                     ismRos->info.resolution, ismInRoi, ism);

  const size_t size = width * height;
  oggTrans->data.resize(size);
  memcpy((void*) oggTrans->data.data(), (void*) ismInRoi.data, size );


  return oggTrans;

}

//void setMask () {
//
//}
//
//bool cellNotInMask(int idx, int idy) {
//
//}

// Get topic name with callback: http://answers.ros.org/question/68434/get-topic-name-in-callback/?answer=68545#post-id-68545
// Using bind function: http://en.cppreference.com/w/cpp/utility/functional/bind
//static std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> currentMapStack;
//static std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> lastMapStack;
void MapserverStat::doIsmFusion(const nav_msgs::OccupancyGrid::ConstPtr &msg, const std::string &topic) {


  // The current OGM (map) with which the ISM (msg) is fused
  mrpt::maps::COccupancyGridMap2D *map = NULL;

  // Sanity checks
  if (currentTileTfName.empty()) {
      ROS_WARN("currentTileTfName is empty: Skipping sensor fusion");
      return;
  }

  // Get the current map stack, or allocate new space
  mapRefresh.lock();
  std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack = currentMapStack;
  try {
//      for (auto& x: *mapStack) {
//        std::cout << x.first << ": " << x.second << '\n';
//      }
      map = mapStack->at(topic.c_str());
  } catch (...) { // It only throws if the topic is not in the map
      ROS_INFO_STREAM("Add new map for topic " << topic);
      map = mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def);
      currentMapStack->insert(
          std::pair<std::string,mrpt::maps::COccupancyGridMap2D*>(topic,map) );
      lastMapStack->insert(
          std::pair<std::string,mrpt::maps::COccupancyGridMap2D*>(topic,mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def)) );
  }
  std::string tileTfName = currentTileTfName;
  mapRefresh.unlock();

// Some debug stuff
//  tf::Pose ogmPose; tf::poseMsgToTF(msg->info.origin, ogmPose);
//  tf::Stamped<tf::Pose> originAsTargetPose(ogmPose, msg->header.stamp, msg->header.frame_id);
//  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, originAsTargetPose, msg->info.resolution, *listenerTf); // Check minimal function
//  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, originAsTargetPose, msg->info.resolution / 2, *listenerTf); // Check only resizing

  // Target pose is the origin of the OGM frame
  tf::Stamped<tf::Pose> targetPose(tf::Pose(tf::Quaternion(0,0,0,1)), msg->header.stamp, tileTfName + tileOriginTfSufixForRoiOrigin);
  nav_msgs::OccupancyGrid::ConstPtr ogmTransformed = ogmTf(msg, targetPose, resolution_mPerTile, *listenerTf); // Check homography


  // Sanity check to speed up
  if (!ogmTransformed) {
      ROS_ERROR("ISM as OGM not valid");
      return;
  } else if (ogmTransformed->info.height <= 0 || ogmTransformed->info.width <= 0) {
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

  // Update a sensor scan:
  // The transformation already points to the lower left edge of the sensor scan
  // Rotation is not allowed for now
  // TODO Check with pose to speed up everything regarding the parameter ogmTf(..., true)
  // Assumptions are made for ISM and OGM: origin pose coincide, resolution is same
  const float max_x = ogmTransformed->info.height * ogmTransformed->info.resolution;
  const float min_x = 0.0f;
  const float max_y = ogmTransformed->info.width * ogmTransformed->info.resolution;
  const float min_y = 0.0f;
  const float transX = ogmTransformed->info.origin.position.x;
  const float transY = ogmTransformed->info.origin.position.y;
  const float transZ = ogmTransformed->info.origin.position.z;

  ROS_DEBUG("\n  Everything should be OK, if the pose is 0,0,0 and the frames are the same:"
            "  ISM coordinate x, y, z, frame_id: %f, %f, %f, %s\n"
            "  OGM frame name: %s\n\n",
            transX, transY, transZ, ogmTransformed->header.frame_id.c_str(), targetPose.frame_id_.c_str());

//  const int mapCenterX = int((def.max_x - def.min_x) / def.resolution / 2.0);
//  const int mapCenterY = int((def.max_y - def.min_y) / def.resolution / 2.0);
//  const int mapOffsetXStart = transX / def.resolution + mapCenterX;
//  const int mapOffsetXEnd   = (transX + max_x - min_x) / def.resolution + mapCenterX;
//  const int mapOffsetYStart = transY / def.resolution + mapCenterY;
//  const int mapOffsetYEnd   = (transY + max_y - min_y) / def.resolution + mapCenterY;

  // Do a point wise sensor fusion
//  for (int idy = mapOffsetYStart; idy < mapOffsetYEnd ; ++idy) {
//    for (int idx = mapOffsetXStart; idx < mapOffsetXEnd ; ++idx) {

  for (int idy = 0; idy < std::min(ogmTransformed->info.height, map->getSizeY()) ; ++idy) {
    for (int idx = 0; idx < std::min(ogmTransformed->info.width, map->getSizeX()) ; ++idx) {
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
      map->updateCell(idx,idy,float(ogmTransformed->data.at(idxIsm)) / 100.0f);
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
    msgTmp = msg;
  }
}

nav_msgs::OccupancyGrid MapserverStat::getIsmOutput(const mapserver_msgs::mapPrimitive &view, const mrpt::maps::COccupancyGridMap2D &input) {

  cv::Mat statisticMat , croppedMat; /*float matrices*/
  nav_msgs::OccupancyGrid output;

  std::stringstream os;
  os << view;
  ROS_INFO("view:\n%s\n", os.str().c_str());

  // Calculate the requested ROI
  const double xView = view.pose.pose.position.x; // Meter
  const double yView = view.pose.pose.position.y; // Meter
  const double wView = view.width * view.resolution; // Tiles * meter/tiles
  const double hView = view.height * view.resolution; // Tiles * meter/tiles
  const double dView = view.depth * view.resolution; // Tiles * meter/tiles
  const std::string sourceframe = view.frame_id.empty() ? machine::frames::names::BASE_LINK : view.frame_id;
  tf::Quaternion q;
  tf::quaternionMsgToTF(view.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  const double zRotView = yaw;


  // Resize the resolution
  cv::Size size(view.width, view.depth);
  cv::resize(statisticMat,croppedMat,size,0,0,cv::INTER_NEAREST);

  // Copy the meta information
  output.header.stamp = ros::Time::now();
  output.header.frame_id = sourceframe;
  output.info.resolution = view.resolution;
  output.info.width  = croppedMat.cols;
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

void MapserverStat::formatAndSendGrid(std::vector<std::string> &list,
                       std::string &frame,
                       const std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>> mapStack,
                       ros::NodeHandle &n,
                       const std::string topicPrefixGrid,
                       const std::shared_ptr<tf::Pose> tfPose,
                       std::string topicSufixGrid,
                       std::string topicSufixPointCloud) {
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
  std::vector<geometry_msgs::Point> points(mapIt->second->getSizeY() * mapIt->second->getSizeX());

  while (true) {
      // Check if we have to work with the list iterator, if not ...
      if (list.size() > 0) {
          if (listIt == list.end()) {
              break;
          }
          mapIt = mapStack->find(*listIt);
          if (mapIt == mapStack->end()) {
              ROS_WARN("%s: No entry for '%s'", fName,  listIt->c_str());
              continue;
          } else if (mapIt->second == NULL) {
              ROS_WARN("%s: Map pointer for '%s' is empty", fName, listIt->c_str());
              continue;
          }
          ++listIt;
      } else { // ... just process the full map stack
          if (idxIt > 0) {
              ++mapIt;
          } // else, it is the first round
          if(mapIt == mapStack->end()) {
              break;
          }
      }

      // Check for publisher and advertise one, if missing
      static std::map<std::string, ros::Publisher> mapPublisher;
      auto publisherIt = mapPublisher.find(mapIt->first);
      if (publisherIt == mapPublisher.end()) {
          ROS_INFO("%s: Advertise map publisher with topic %s", fName, mapIt->first.c_str());
          mapPublisher.insert(
                    std::pair<std::string,ros::Publisher>(mapIt->first,
                                                          n.advertise<nav_msgs::GridCells>(topicPrefixGrid + mapIt->first + topicSufixGrid, 1)) );
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
            if (mapIt->second->getCell(idx, idy) >  minDrawOccupancyUpdateCertainty) {
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


void MapserverStat::addMapToResponse(const std::string &name, mrpt::maps::COccupancyGridMap2D* map, mapserver::ismStackFloat::Response &response) {
  if (map == NULL) {
      ROS_WARN("Map pointer empty");
      return;
  }

  response.response.mapNames.strings.resize(response.response.mapNames.strings.size() + 1);
  response.response.mapStack.resize(response.response.mapStack.size() + 1);

  auto mapName = response.response.mapNames.strings.end()-1;
  *mapName = name;

  const std::size_t numCells = map->getSizeX() * map->getSizeY();
  // Copy the cells
  auto mapRes = (response.response.mapStack.end()-1);
  mapRes->map.resize(numCells);
  for (int idx = 0; idx < numCells; ++idx) {
      int xIdx = idx % map->getSizeX();
      int yIdx = idx / map->getSizeX();
      mapRes->map.at(idx) = map->getCell(xIdx, yIdx);
  }

  mapRes->header.stamp = ros::Time::now();
  mapRes->info.height = map->getSizeY();
  mapRes->info.width = map->getSizeX();
  const tf::Pose pose = tf::Pose(tf::Quaternion(0,0,0,1));
  tf::poseTFToMsg(pose, mapRes->info.origin);
  mapRes->info.resolution = map->getResolution();
}

bool MapserverStat::mapStatServerMapStack(mapserver::ismStackFloat::Request &req, mapserver::ismStackFloat::Response &res) {
  mapRefresh.lock();
  auto mapStack = currentMapStack;
  std::string tileName = currentTileTfName;
  mapRefresh.unlock();

  ROS_INFO( "statMapserver-mapStack method called" );
  if (req.request.strings.empty()) {
      ROS_INFO( "Respond the full map stack" );
      for (auto it=mapStack->begin(); it!=mapStack->end(); ++it) {
          addMapToResponse(it->first, it->second, res);
      }
  } else {
      for (auto it=req.request.strings.begin(); it!=req.request.strings.end(); ++it) {
          auto itMapStack = mapStack->find(*it);
          if (itMapStack != mapStack->end()) {
              addMapToResponse(itMapStack->first, itMapStack->second, res);
          }
      }
  }


  res.response.header.frame_id = tileName + tileOriginTfSufixForRoiOrigin;
  res.response.header.stamp = ros::Time::now();
  for (auto it = res.response.mapStack.begin(); it != res.response.mapStack.end(); ++it) {
      it->header.frame_id = res.response.header.frame_id;
  }
  ROS_INFO( "statMapserver-mapStack method called: Finish" );
  return true;
}

MapserverStat::MapserverStat(ros::NodeHandle& nh) : Mapserver(&nh, this), n(nh){

    this->n.param<std::string>("debug_ism_topic", debugIsmTopic, "/ism/radar/tracking/radar_return"); // The topic of the ISM which is resend as transformed ISM in the mapserver frame
    this->n.param<std::string>("debug_topic", debugTopic, "/amiro2/ism/cam"); // The topic of the fused map to show via opencv
    this->n.param<float>("max_occupancy_update_certainty", maxOccupancyUpdateCertainty, 0); // Maximum update uncertainty
    this->n.param<float>("max_distance_insertion", maxDistanceInsertion, 0); // Maximum distance insertion
    this->n.param<float>("uncertainty_boundary", uncertaintyBoundary, 0.5); // Uncertainty boundary for displaying a feature (standard: 0.5 (unknown))
    this->n.param<std::string>("topic_debug_grid_prefix", topicDebugGridPrefix, "/viz");



    if (mapInitValue > uncertaintyBoundary)
      ROS_WARN_STREAM("'uncertaintyBoundary' is less than 'mapInitValue', display might be corrupted");

    // Define the properties of one occupancy map
    def.resolution = this->resolution_mPerTile;
    def.insertionOpts.maxOccupancyUpdateCertainty = this->maxOccupancyUpdateCertainty;
    def.insertionOpts.maxDistanceInsertion = this->maxDistanceInsertion;
    def.max_x = this->maxX_m;
    def.min_x = this->minX_m;
    def.max_y = this->maxY_m;
    def.min_y = this->minY_m;

    // Allocate space for the map and mapstacks
      this->currentMapStack = std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>>(new std::map<std::string, mrpt::maps::COccupancyGridMap2D*>);
      this->lastMapStack = std::shared_ptr<std::map<std::string, mrpt::maps::COccupancyGridMap2D*>>(new std::map<std::string, mrpt::maps::COccupancyGridMap2D*>);

      publisherIsmAsPointCloud = this->n.advertise<sensor_msgs::PointCloud>("foo",1);
      publisherIsmAsOgm = this->n.advertise<nav_msgs::OccupancyGrid>(std::string("debug") + debugIsmTopic,1);

      // Prepare ROS service
      // TODO: Do this on demand for given subscribed topics
      const std::string s("/");

    //  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::compressedMapImage , mapServerCompressedMap);
    //  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::stockEdge , mapServerStockEdge);
    //  ros::ServiceServer service_singleLayerOgm = n.advertiseService(scopes::map::ogmServer::parent + s + scopes::map::ogmServer::requests::singleLayerOgm , mapServerSingleLayerOgm);
//      service_mapStack = this->n.advertiseService(reqTopicMapStack/*scopes::map::statServer::parent + s + scopes::map::statServer::requests::mapStack*/, &MapserverStat::mapStatServerMapStack);
      service_mapStack = n.advertiseService(reqTopicMapStack/*scopes::map::statServer::parent + s + scopes::map::statServer::requests::mapStack*/, &MapserverStat::mapStatServerMapStack, this);
  }

  void MapserverStat::spinOnce() {
        // Plot map
  //      boost::shared_ptr<cv::Mat> image(doColorMapCallback());
  //      cv::imshow( "Current View", *image );                    // Show our image inside it.
  //      cv::waitKey(1); // Update the window
        if (debug) {
          try {
    //            std::shared_ptr<cv::Mat> image(mrptOggToGrayScale(*currentMapStack->at(debugTopic)));
    //            std::shared_ptr<cv::Mat> image(rosOggToGrayScale(msgTmp));
  //              if (image) {
  //                  cv::flip(*image, *image, 0);
  //                  cv::imshow( "Current View", *image);
  //                  cv::waitKey(1); // Update the window
  //              }
          } catch (...) {
              ROS_WARN("Debug visualization: No such topic '%s' in currentMapStack", debugTopic.c_str());
          }
          // Publish the maps
  //        std::vector<std::string> foo;
  //        tf::Vector3 trans = tf::Vector3(tfScalar(-(max_x - min_x)/2.0f), tfScalar(-(max_y - min_y)/2.0f), tfScalar(0.0f));
  //        std::shared_ptr<tf::Pose> poseOffset = std::shared_ptr<tf::Pose>(new tf::Pose(tf::Quaternion(0,0,0,1), trans));
  //        std::string reference = currentTileTfName + tileOriginTfSufixForRoiOrigin;
  //        formatAndSendGrid(foo, reference, currentMapStack, n, topicDebugGridPrefix);
        }

  //      showRpc();

        // Add new subscribers
        this->advertiseSubscribers(subIsmList, &MapserverStat::doIsmFusion, this, topicPrefix, debug);
  }

  void MapserverStat::spin() {
    ros::AsyncSpinner spinner(5);
    spinner.start();
    // Do stuff periodically
    ros::Rate _rate(rate);
    ROS_ERROR("Mapserver starts spinning");
    while(ros::ok()) {
      this->spinOnce();
      _rate.sleep();
    }
  }

