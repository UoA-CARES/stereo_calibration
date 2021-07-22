//
// Created by henry on 1/04/21.
//

#include "CharucoCalibrator.h"

//void CharucoCalibrator::GetCorners(Mat& image, Ptr<aruco::CharucoBoard>& board, vector<Corner>& corners)
//{
//  vector<int> ids; vector<PointSet> points, rejected;
//  aruco::detectMarkers(image, board->dictionary, points, ids);
//  aruco::refineDetectedMarkers(image, board, points, ids, rejected);
//  vector<int> gridCornerIds; vector<Point2f> gridCorners;
//  aruco::interpolateCornersCharuco(points, ids, image, board, gridCorners, gridCornerIds);
//
//  for (auto i = 0; i < (int)gridCornerIds.size(); i++)
//  {
//    auto imagePoint = gridCorners[i];
//    auto index = gridCornerIds[i];
//    auto scenePoint = board->chessboardCorners[index];
//    corners.push_back(Corner(scenePoint, imagePoint));
//  }
//}

void CharucoCalibrator::createObjectPoints(std::vector< int > &corner_ids, std::vector<Point3f> &object_points){
//  for(int id : corner_ids){
//    int number_of_corners
//    object_points.push_back(this->charucoboard->chessboardCorners[id]);
//  }
  for (int i = 0; i < board_size.height; ++i) {
    for (int j = 0; j < board_size.width; ++j) {
      int id = j + i * (board_size.width);
      if(std::find(corner_ids.begin(), corner_ids.end(), id) != corner_ids.end()) {
        int x = j;
        int y = (board_size.height - 1 - i);
//        ROS_INFO("%i %i %i", x, y, id);
        object_points.push_back(cv::Point3f(x * this->square_length, y * this->square_length, 0));
      }
    }
  }
}

void CharucoCalibrator::balanceCorners(std::vector< int > &left_ids,
                                       std::vector< Point2f > &left_corners,
                                       std::vector< int > &right_ids,
                                       std::vector< Point2f > &right_corners){

  //Stereo expects reverse of corner ids -> when creating "comparison/object"
  std::vector<int> result;
  std::set_intersection(left_ids.begin(),
                        left_ids.end(),
                        right_ids.begin(),
                        right_ids.end(),
                        std::inserter(result, result.begin()));

  std::vector< Point2f > balanced_left_corners;
  std::vector< Point2f > balanced_right_corners;
  for(int i = 0; i < result.size(); ++i){
    int id = result[i];
    for(int l_i = 0; l_i < left_ids.size(); ++l_i){
      if(left_ids[l_i] == id){
        balanced_left_corners.push_back(left_corners[l_i]);
        break;
      }
    }
    for(int r_i = 0; r_i < right_ids.size(); ++r_i){
      if(right_ids[r_i] == id){
        balanced_right_corners.push_back(right_corners[r_i]);
        break;
      }
    }
  }

  left_ids = result;
  left_corners = balanced_left_corners;

  right_ids = result;
  right_corners = balanced_right_corners;
}

void CharucoCalibrator::extractCorners(Mat &image,
                                       std::vector<int> &charuco_ids,
                                       std::vector< Point2f> &charuco_corners,
                                       Mat image_copy){

  std::vector< int > marker_ids;
  std::vector< std::vector< Point2f > > marker_corners;
  std::vector< std::vector< Point2f > > marker_rejected;

  // detect markers
  Ptr<aruco::DetectorParameters> detector_params = aruco::DetectorParameters::create();
  aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, detector_params, marker_rejected);

  // refine strategy to detect more markers
  aruco::refineDetectedMarkers(image, this->charucoboard, marker_corners, marker_ids, marker_rejected);

  // interpolate charuco marker_corners

  if(marker_ids.size() > 0) {
    //Corners of checker board not the aruco markers
    aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, charucoboard, charuco_corners, charuco_ids);

    aruco::drawDetectedMarkers(image_copy, marker_corners, marker_ids);

    if(charuco_corners.size() > 0)
      aruco::drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids);
  }
  else{
    ROS_WARN("No marker detected");
  }
}

void CharucoCalibrator::findCorners(Mat &left_image, Mat &right_image,
                                    std::vector< Point2f > &left_corners,
                                    std::vector< Point2f > &right_corners,
                                    std::vector<Point3f> &object_points) {
  //Extract markers and corners
  std::vector< int > left_ids;
  cv::Mat left_image_copy = left_image.clone();
  this->extractCorners(left_image, left_ids, left_corners, left_image_copy);

  std::vector< int > right_ids;
  cv::Mat right_image_copy = right_image.clone();
  this->extractCorners(right_image, right_ids, right_corners, right_image_copy);

  //Balance corners that match between the left and right images
  this->balanceCorners(left_ids, left_corners, right_ids, right_corners);

  //create matching object points
  this->createObjectPoints(left_ids, object_points);

  //Draw object points for debugging
  for(int i = 0; i < left_ids.size(); ++i){
    int id = left_ids[i];
    Point3f pt = object_points[i];
    cv::Scalar black( 0, 0, 0 );
    cv::putText(left_image_copy, std::to_string(id), Point(pt.x, pt.y+30),cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255));

//    cv::putText(left_image_copy, std::to_string(id), left_corners[i],cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255));
  }

  if(this->display) {
    imshow("Left-image", left_image_copy);
    imshow("Right-image", right_image_copy);
    waitKey(1);
  }
}

