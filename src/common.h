//
// Created by henry on 7/04/21.
//

#ifndef STEREO_CALIBRATION_COMMON_H
#define STEREO_CALIBRATION_COMMON_H

#include "ros/ros.h"
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sstream>
#include <termio.h>
#include <limits>
#include <opencv2/aruco/charuco.hpp>

namespace CARES {
    namespace Calibration {
        enum CalibrationMethod{
            CHECKER,
            CHARUCO
        };

        Mat convertToMat(const sensor_msgs::ImageConstPtr &msg) {
          try {
            //Get the time_start image in opencv Mat format
            Mat bgr_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

            //Convert RGB image to bgr if required
            if (msg->encoding == "rgb8") {//TODO put static type in here instead
              cvtColor(bgr_image, bgr_image, CV_RGB2BGR);
            }
            return bgr_image;

          }
          catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit(1);
          }
        }
    }
}

#endif //STEREO_CALIBRATION_COMMON_H
