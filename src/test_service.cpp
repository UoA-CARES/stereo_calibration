//
// Created by henry on 7/04/21.
//

#include "ros/ros.h"
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sstream>
#include <termio.h>
#include <limits>
#include <opencv2/aruco/charuco.hpp>
#include <boost/filesystem/operations.hpp>
#include <ctime>
#include <math.h>
#include "CharucoCalibrator.h"
#include "CheckerCalibrator.h"
#include "parameters.h"
#include "common.h"
#include "cares_msgs/StereoCameraInfo.h"
#include "cares_msgs/CalibrationService.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace CARES::Calibration;


int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::ServiceClient client = nh.serviceClient<cares_msgs::CalibrationService>("stereo_calibration");

  const char *env = getenv("HOME");
  std::string home_path = "";
  home_path.append(env);

  std::string image_directory = home_path + "/calibration_test/charuco/";
  cares_msgs::CalibrationService srv;
  srv.request.image_directory = image_directory;
  client.call(srv);
}