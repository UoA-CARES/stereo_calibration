//
// Created by henry on 7/04/21.
//

#ifndef STEREO_CALIBRATION_PARAMETERS_H
#define STEREO_CALIBRATION_PARAMETERS_H

namespace CARES{
  namespace Calibration{
      const std::string CAMERA_NODE_LEFT_S  = "camera_left";
      const std::string CAMERA_NODE_RIGHT_S = "camera_right";

      const std::string CALIBRATION_METHOD_S = "method";

      const std::string BOARD_WIDTH_I   = "board_width";
      const std::string BOARD_HEIGHT_I  = "board_height";
      const std::string SQUARE_LENGTH_D = "square_length";

      const std::string IMAGE_WIDTH_I  = "image_width";
      const std::string IMAGE_HEIGHT_I = "image_height";

      const std::string MARKER_LENGTH_D = "marker_length";
      const std::string DICTIONARY_ID_I = "dictionary";

      const std::string IMAGE_PATH_S = "image_path";

      const std::string SAVE_DIRECTORY_PATH_S = "save_directory";

      const std::string THRESHOLD_D = "threshold";

      const std::string DISPLAY_B = "display";
  }
}

#endif //STEREO_CALIBRATION_PARAMETERS_H
