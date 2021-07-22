//
// Created by henry on 1/04/21.
//

#ifndef STEREO_CALIBRATION_CharucoCalibrator_H
#define STEREO_CALIBRATION_CharucoCalibrator_H

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

class CharucoCalibrator : public StereoCalibrator {
private:
    double marker_length;//Size of the squares on the calibration board in (m)

    Ptr<aruco::Dictionary> dictionary;
    Ptr<aruco::CharucoBoard> charucoboard;

    void createObjectPoints(std::vector<int> &corner_ids, std::vector<Point3f> &object_points);

    void balanceCorners(std::vector< int > &left_ids,
                        std::vector< Point2f > &left_corners,
                        std::vector< int > &right_ids,
                        std::vector< Point2f > &right_corners);

    void extractCorners(Mat &image,
                        std::vector<int> &charuco_ids,
                        std::vector< Point2f> &charuco_corners,
                        Mat image_copy);

public:
    CharucoCalibrator(Size board_size, double square_length, double marker_length, int dictionary_id, Size image_size, double average_min_difference_threshold, bool display)
            : StereoCalibrator(board_size, square_length, image_size, average_min_difference_threshold, display) {

      this->marker_length = marker_length;
      this->dictionary   = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
      ///<Note charucoboard is the number of squares (corner size +1) in each direction
      this->charucoboard = aruco::CharucoBoard::create(this->board_size.width+1, this->board_size.height+1,
                                                       this->square_length, this->marker_length, this->dictionary);
    }

protected:
    void findCorners (Mat &left_image,
                     Mat &right_image,
                     std::vector<Point2f> &left_corners,
                     std::vector<Point2f> &right_corners,
                     std::vector<Point3f> &object_points) override;
};


#endif //STEREO_CALIBRATION_CharucoCalibrator_H
