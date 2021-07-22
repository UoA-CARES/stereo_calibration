//
// Created by henry on 1/04/21.
//

#ifndef STEREO_CALIBRATION_STEREOCALIBRATOR_H
#define STEREO_CALIBRATION_STEREOCALIBRATOR_H

#include "ros/ros.h"
#include <ros/package.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/aruco/charuco.hpp>

#include "sensor_msgs/CameraInfo.h"
#include "cares_msgs/StereoCameraInfo.h"

#include <boost/filesystem/operations.hpp>

using namespace cv;

class StereoCalibrator{
protected:
    bool display;
private:
    std::vector< std::vector<cv::Point2f > > all_corners[2];
    std::vector< cv::Mat > all_images[2];
    std::vector<std::vector<cv::Point3f> > object_points;

public:
  StereoCalibrator(Size board_size, double square_length, Size image_size, double average_min_difference_threshold, bool display){
    this->display = display;

    this->board_size = board_size;
    this->square_length = square_length;
    this->image_size = image_size;
    this->average_min_difference_threshold = average_min_difference_threshold;

    //Setup image windows
    if(this->display) {
      std::string left_window = "Left-image";
      cv::namedWindow(left_window, cv::WINDOW_NORMAL);
      cv::resizeWindow(left_window, 600, 600);
      cv::moveWindow(left_window, 20, 20);

      std::string right_window = "Right-image";
      cv::namedWindow(right_window, cv::WINDOW_NORMAL);
      cv::resizeWindow(right_window, 600, 600);
      cv::moveWindow(right_window, 640, 20);
      cv::waitKey(10);
    }
  }

    cares_msgs::StereoCameraInfo calibrate(std::string save_directory);
    void processImages(Mat &left_image, Mat &right_image, std::string save_directory="");

protected:
    Size board_size;//Number of points on the board, width and height
    Size image_size;//Size of the image in pixels
    double square_length;//Size of the squares on the calibration board in (m or mm)

    double average_min_difference_threshold;//Threshold to determine if calibration images are different enough.

    virtual void findCorners(Mat &left_image,
                     Mat &right_image,
                     std::vector<Point2f> &left_corners,
                     std::vector<Point2f> &right_corners,
                     std::vector<Point3f> &object_points) = 0;
};

#endif //STEREO_CALIBRATION_STEREOCALIBRATOR_H
