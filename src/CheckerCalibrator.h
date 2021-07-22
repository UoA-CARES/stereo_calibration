//
// Created by henry on 6/04/21.
//

#ifndef STEREO_CALIBRATION_CHECKERCALIBRATOR_H
#define STEREO_CALIBRATION_CHECKERCALIBRATOR_H


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

#include "StereoCalibrator.h"

using namespace cv;

class CheckerCalibrator : public StereoCalibrator {
private:

    void createObjectPoints(std::vector<Point3f> &object_points);
    void findCorners(Mat &image, std::vector<Point2f> &corners, Mat &image_copy);

public:
    CheckerCalibrator(Size board_size, double square_length, Size image_size, double average_min_difference_threshold, bool display)
            : StereoCalibrator(board_size, square_length, image_size, average_min_difference_threshold, display) {
    }

protected:
    void findCorners (Mat &left_image,
                      Mat &right_image,
                      std::vector<Point2f> &left_corners,
                      std::vector<Point2f> &right_corners,
                      std::vector<Point3f> &object_points) override;
};


#endif //STEREO_CALIBRATION_CHECKERCALIBRATOR_H
