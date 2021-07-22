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

std::string save_directory = "";
StereoCalibrator* calibrator;

void collectImagesFromFolder(std::string image_load_file_path){
  ROS_INFO("Calibrating from Files - %s", image_load_file_path.c_str());

  std::vector<cv::String> filenames;
  glob(image_load_file_path+"*_left_image_color.png", filenames);

  int number_of_image_pairs = filenames.size();
  ROS_INFO("Loading %d pairs of images", number_of_image_pairs);
  for(int i = 1; i < number_of_image_pairs-1; ++i) {
    Mat left_image;
    Mat right_image;

    std::string left_filepath  = image_load_file_path + std::to_string(i) + "_left_image_color.png";
    std::string right_filepath = image_load_file_path + std::to_string(i) + "_right_image_color.png";

    if (boost::filesystem::exists(left_filepath) && boost::filesystem::exists(right_filepath)) {
      left_image = imread(left_filepath);
      right_image = imread(right_filepath);
      calibrator->processImages(left_image, right_image);
    }
  }
}

bool calibrationCallback(cares_msgs::CalibrationServiceRequest &request, cares_msgs::CalibrationServiceResponse &response) {
  std::string image_directory = request.image_directory;

  collectImagesFromFolder(image_directory);

  cares_msgs::StereoCameraInfo stereo_info = calibrator->calibrate(image_directory);
  response.stereo_info = stereo_info;
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "stereo_calibration_service");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //Load calibration parameters
  Size board_size;
  if(!nh_private.getParam(CARES::Calibration::BOARD_WIDTH_I, board_size.width) || !nh_private.getParam(CARES::Calibration::BOARD_HEIGHT_I, board_size.height)){
    ROS_ERROR("Undefined board size");
    return 1;
  }
  ROS_INFO("Board size - width %i height %i", board_size.width, board_size.height);

  Size image_size;
  if(!nh_private.getParam(CARES::Calibration::IMAGE_WIDTH_I, image_size.width) || !nh_private.getParam(CARES::Calibration::IMAGE_HEIGHT_I, image_size.height)){
    ROS_ERROR("Undefined image size");
    return 1;
  }
  ROS_INFO("Image - width: %i height %i", image_size.width, image_size.height);

  double square_length;
  if(!nh_private.getParam(CARES::Calibration::SQUARE_LENGTH_D, square_length)){
    ROS_ERROR("Undefined square length");
    return 1;
  }
  square_length /= 1000.0;
  ROS_INFO("Square length: %f m", square_length);

  //Determine calibration method
  CalibrationMethod calibration_method;
  int method;
  if(!nh_private.getParam(CARES::Calibration::CALIBRATION_METHOD_S, method)){
    ROS_ERROR("Undefined calibration method");
    return 1;
  }
  calibration_method = static_cast<CalibrationMethod>(method);

  double average_min_difference_threshold = 0;
  nh_private.param(CARES::Calibration::THRESHOLD_D, average_min_difference_threshold, average_min_difference_threshold);
  ROS_INFO("Difference Threshold: %f", average_min_difference_threshold);

  bool display;
  nh_private.param(CARES::Calibration::DISPLAY_B, display, true);

  switch(calibration_method){
    case CHECKER:
      ROS_INFO("Checker board calibration");
      calibrator = new CheckerCalibrator(board_size, square_length, image_size, average_min_difference_threshold, display);
      break;
    case CHARUCO:
      ROS_INFO("Charuco board calibration");
      double marker_length;
      if(!nh_private.getParam(CARES::Calibration::MARKER_LENGTH_D, marker_length)){
        ROS_ERROR("Undefined marker length");
        return 1;
      }
      marker_length /= 1000.0;
      ROS_INFO("Marker length: %f m", marker_length);
      int dictionary_id = aruco::DICT_4X4_50;
      nh_private.param(CARES::Calibration::DICTIONARY_ID_I, dictionary_id, dictionary_id);
      ROS_INFO("Using dictionary ID: %i", dictionary_id);
      calibrator = new CharucoCalibrator(board_size, square_length, marker_length, dictionary_id, image_size, average_min_difference_threshold, display);
      break;
  }

  ROS_INFO("Setup calibration service");
  ros::ServiceServer service = nh.advertiseService("stereo_calibration", calibrationCallback);

  ros::spin();
}

