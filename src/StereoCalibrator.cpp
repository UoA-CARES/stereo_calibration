//
// Created by henry on 6/04/21.
//

#include "StereoCalibrator.h"

void setCameraInfo(std::string camera_name,
                   Size image_size,
                   cv::Mat K,
                   cv::Mat D,
                   cv::Mat P,
                   cv::Mat R,
                   sensor_msgs::CameraInfo &camera_info){
  //Frame ID is named after the camera name
  camera_info.header.frame_id = camera_name;

  //Size of image at calibration
  camera_info.width  = image_size.width;
  camera_info.height = image_size.height;

  std::vector<double>K_v(K.begin<double>(), K.end<double>());
  std::vector<double>P_v(P.begin<double>(), P.end<double>());
  std::vector<double>R_v(R.begin<double>(), R.end<double>());

  //Intrinsic parameters
  camera_info.D = D;//dist
  for (int i=0; i<K_v.size(); i++)camera_info.K[i] = (K_v[i]);
  for (int i=0; i<P_v.size(); i++)camera_info.P[i] = (P_v[i]);
  for (int i=0; i<R_v.size(); i++)camera_info.R[i] = (R_v[i]);

  //Default is plump bob
  //TODO add this to calibration file and read from there
  camera_info.distortion_model = "plumb_bob";

  camera_info.binning_x = 0;//width
  camera_info.binning_y = 0;//height

  sensor_msgs::RegionOfInterest roi;
  camera_info.roi = roi;
}

cares_msgs::StereoCameraInfo setStereoInfo(Mat K1, Mat D1, Mat P1, Mat R1,
                                           Mat K2, Mat D2, Mat P2, Mat R2,
                                           Mat Q, Mat R, Mat T,
                                           Size image_size){
  cares_msgs::StereoCameraInfo stereo_camera_info;

  //Left parameters
  sensor_msgs::CameraInfo left_info;
  setCameraInfo("left", image_size, K1, D1, P1, R1, left_info);
  stereo_camera_info.left_info  = left_info;

  //Right parameters
  sensor_msgs::CameraInfo right_info;
  setCameraInfo("right", image_size, K2, D2, P2, R2, right_info);
  stereo_camera_info.right_info = right_info;

  //Stereo parameters
  std::vector<double>Q_v(Q.begin<double>(), Q.end<double>());
  std::vector<double>R_v(R.begin<double>(), R.end<double>());
  std::vector<double>T_v(T.begin<double>(), T.end<double>());

  stereo_camera_info.header.frame_id = "left";
  for (int i=0; i<Q_v.size(); i++)stereo_camera_info.Q[i] = Q_v[i];
  for (int i=0; i<R_v.size(); i++)stereo_camera_info.R_left_right[i] = R_v[i];
  for (int i=0; i<T_v.size(); i++)stereo_camera_info.T_left_right[i] = T_v[i];
  return stereo_camera_info;
}

cares_msgs::StereoCameraInfo StereoCalibrator::calibrate(std::string save_directory) {
  ROS_INFO("Calibrating the stereo camera");
  ROS_INFO("%i %i", all_corners[0].size(), all_corners[1].size());

  cares_msgs::StereoCameraInfo stereo_info;
  if(all_corners[0].size() == 0 || all_corners[1].size() == 0){
    ROS_ERROR("No images have been captured or loaded");
    return stereo_info;
  }

  Mat camera_matrix[2];
  Mat dist_coeffs[2];
  Mat rvecs, tvecs;//unused only placed for completion of calibrateCamera
  double left_rms  = calibrateCamera(object_points, all_corners[0], image_size, camera_matrix[0], dist_coeffs[0], rvecs, tvecs);
  double right_rms = calibrateCamera(object_points, all_corners[1], image_size, camera_matrix[1], dist_coeffs[1], rvecs, tvecs);

  ROS_INFO("Left RMS: %f Right RMS: %f", left_rms, right_rms);

  Mat R, T, E, F;
  ROS_INFO("Calculating RMS!");

  ros::Time start = ros::Time::now();
  double rms = stereoCalibrate(object_points, all_corners[0], all_corners[1],
                               camera_matrix[0], dist_coeffs[0],
                               camera_matrix[1], dist_coeffs[1],
                               image_size, R, T, E, F,
                               CALIB_USE_INTRINSIC_GUESS +
                               CALIB_RATIONAL_MODEL,
                               TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

  ROS_INFO("RMS: %f", rms);

  Mat R1, R2, P1, P2, Q;
  Rect valid_roi[2];
  //TODO extract out a toggle for the alpha setting
  stereoRectify(camera_matrix[0], dist_coeffs[0],
                camera_matrix[1], dist_coeffs[1],
                image_size, R, T, R1, R2, P1, P2, Q,
                CALIB_ZERO_DISPARITY, 0, image_size, &valid_roi[0], &valid_roi[1]);

  double difference = (ros::Time::now().toNSec() - start.toNSec())/1000000000.0;
  ROS_INFO("Time to calculate int/ext: %f s", difference);

  std::string caliration_file = save_directory+"stereo_calibration.json";
  ROS_INFO("Saving results to %s", caliration_file.c_str());

  FileStorage file(caliration_file, FileStorage::WRITE);
  //Stereo
  file.write("Q", Q);
  file.write("R", R);
  file.write("T", T);
  file << "imageSize" << image_size;

  //Left
  file.write("K1", camera_matrix[0]);
  file.write("D1", dist_coeffs[0]);
  file.write("P1", P1);
  file.write("R1", R1);

  //Right
  file.write("K2", camera_matrix[1]);
  file.write("D2", dist_coeffs[1]);
  file.write("P2", P2);
  file.write("R2", R2);

  stereo_info = setStereoInfo(camera_matrix[0], dist_coeffs[0], P1, R1,
                              camera_matrix[1], dist_coeffs[1], P2, R2,
                              Q, R, T, image_size);
  return stereo_info;
}

double closestMatchScore(const std::vector<std::vector<Point2f> > &db_points, const std::vector<Point2f> &corner_points) {
  double min = std::numeric_limits<double>::max();//Max double
  for (std::vector<Point2f> points : db_points) {
    if(points.size() == corner_points.size()) {
      double total = 0.0;
      for (int i = 0; i < points.size(); ++i) {
        total += sqrt(pow((points[i].x - corner_points[i].x), 2) + pow((points[i].y - corner_points[i].y), 2));
      }
      if (total < min) {
        min = total;
      }
    }
  }
  return (corner_points.size() == 0 ? min : min / corner_points.size());//Returns average difference between the points
}

void StereoCalibrator::processImages(Mat &left_image, Mat &right_image, std::string save_directory){
  std::vector<Point2f> left_corners;
  std::vector<Point2f> right_corners;
  std::vector<Point3f> object_points;

  //Overridden by Checker or Charuco detection methods.
  this->findCorners(left_image, right_image, left_corners, right_corners, object_points);

  if(left_corners.size() == right_corners.size()){
    if(left_corners.size() > 6) {
      //Determine if this pose has already been detected and stored
      double min_diff_left  = closestMatchScore(all_corners[0], left_corners);
      double min_diff_right = closestMatchScore(all_corners[1], right_corners);

      if (min_diff_left > this->average_min_difference_threshold || min_diff_right > this->average_min_difference_threshold) {
        this->all_images[0].push_back(left_image);
        this->all_corners[0].push_back(left_corners);

        this->all_images[1].push_back(right_image);
        this->all_corners[1].push_back(right_corners);

        this->object_points.push_back(object_points);

        if(save_directory != "") {
          int id = this->all_images[0].size();
          std::string left_image_file  = save_directory + std::to_string(id) + "_left_rgb.png";
          std::string right_image_file = save_directory + std::to_string(id) + "_right_rgb.png";
          ROS_INFO("Saving images to %s %s", left_image_file.c_str(), right_image_file.c_str());
          imwrite(left_image_file, left_image);
          imwrite(right_image_file, right_image);
        }
        ROS_INFO("Added new calibration pair - %i total pairs", all_corners[0].size());
      }
      else{
        ROS_WARN("Similar pair already captured - left %f right %f Threshold %f", min_diff_left, min_diff_right, this->average_min_difference_threshold);
      }
    }
    else{
      ROS_WARN("To few points for accurate calibration - %i", left_corners.size());
    }
  }
  else{
    ROS_ERROR("Corners are not balanced between Left %i and Right %i", left_corners.size(), right_corners.size());
  }
}
