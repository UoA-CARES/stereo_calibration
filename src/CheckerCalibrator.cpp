//
// Created by henry on 6/04/21.
//

#include "CheckerCalibrator.h"


void CheckerCalibrator::createObjectPoints(std::vector<Point3f> &object_points) {
  for (int i = 0; i < board_size.height; i++)
    for (int j = 0; j < board_size.width; j++)
      object_points.push_back(Point3f(j * this->square_length, i * this->square_length, 0));
}

void CheckerCalibrator::findCorners(Mat &image, std::vector<Point2f> &corners, Mat &image_copy) {
  bool found = findChessboardCorners(image, this->board_size, corners, CALIB_CB_FAST_CHECK);//openCV method
  // bool found_left = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
  if (found) {
    Mat gray_image;
    cvtColor(image, gray_image, COLOR_BGR2GRAY);
    cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01));
    drawChessboardCorners(image_copy, this->board_size, Mat(corners), found);
  }
}

void CheckerCalibrator::findCorners(Mat &left_image,
                                    Mat &right_image,
                                    std::vector<Point2f> &left_corners,
                                    std::vector<Point2f> &right_corners,
                                    std::vector<Point3f> &object_points) {

  Mat left_image_copy  = left_image.clone();
  Mat right_image_copy = right_image.clone();

  //Find checkerboard corners
  findCorners(left_image, left_corners, left_image_copy);
  findCorners(right_image, right_corners, right_image_copy);

  //Create matching object points
  createObjectPoints(object_points);

  if(this->display) {
    imshow("Left-image", left_image_copy);
    imshow("Right-image", right_image_copy);
    waitKey(10);
  }
}
