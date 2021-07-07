# Stereo Calibration

ROS package for calibrating stereo cameras via Checker or Charuco boards.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
1) ROS Neotic - works in Melodic as well

2) Pull master version of cares_msgs
    a) cd ~/catkin_ws/src
    b) git clone https://github.com/UoA-CARES/cares_msgs.git

3) Compile both libraries
    a) cd ~/catkin_ws
    b) catkin_make
```

### Installing

A step by step series of examples that tell you how to get a development env running

Clone the package into the catkin directory you are using, presumned here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/UoA-CARES/stereo_calibration.git
```

Build the package with catkin_make in the source directory

```
cd ~/catkin_src/
catkin_make
```

## Running the tests

Tests to be added

## Applications

### Stereo Calibration Node
This node runs in two modes.

* First Mode
    * Runs live and will collate images being published live off a pair of stereo cameras.
* Second mode
    * Load stereo image pairs from a provided source location.

Images and calibration data is saved into a specified folder location (same folder as the stereo pair images).
If a folder is not specified the location will default to ~/calibration_images/Y_M_D_H_S.

#### Subscribed Topics
Left and right image pairs being published by a stereo pair - live calibration.
Topic names are all default names (left/right), they can be changed via setting parameters in the launch file and refer to topic names.

* Image
  * left/image_raw
  * right/image_raw

#### Published Topics
No information is published.

##### live_stereo_calibrate_charuco.launch
Run using launch file as below - this will subscribe to the given stereo images and collate images until you press 'e' in the terminal to proceed to calibration.

```
roslaunch stereo_calibration live_stereo_calibrate_charuco.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <!--Save images and calibration data too folder - default "" = ~/calibration_images/M_D/N-->
  <arg name="save_directory" default=""/>

  <!--Image topic names-->
  <arg name="camera_left"  default="left/image_raw"/>
  <arg name="camera_right" default="right/image_raw"/>

  <!--Number of corners-->
  <arg name="board_width"   default="8"/>
  <arg name="board_height"  default="5"/>
  <arg name="square_length" default="45.0"/><!--mm-->
  <arg name="image_width"  default="1936"/>
  <arg name="image_height" default="1216"/>

  <!--Calibration modes-->
  <!--0 Checker-->
  <!--1 Charuco-->
  <arg name="method" default="1"/>

  <!--Difference Threshold-->
  <arg name="threshold" default="25"/><!--px-->

  <!--Charuco marker length if required-->
  <arg name="marker_length" default="23.0"/><!--mm-->

  <!--Dictionary ID-->
  <arg name="dictionary" default="0"/><!--DICT_4X4_50-->

  <node name="stereo_calibration_node" pkg="stereo_calibration" type="stereo_calibration_node" output="screen">
    <param name ="save_directory"  value="$(arg save_directory)"/>

    <param name ="camera_left"  value="$(arg camera_left)"/>
    <param name ="camera_right" value="$(arg camera_right)"/>

    <param name ="board_width"  value="$(arg board_width)"/>
    <param name ="board_height" value="$(arg board_height)"/>
    <param name ="square_length" value="$(arg square_length)"/>

    <param name ="image_width"  value="$(arg image_width)"/>
    <param name ="image_height" value="$(arg image_height)"/>

    <param name ="method" value="$(arg method)"/>

    <param name ="threshold" value="$(arg threshold)"/>

    <param name ="marker_length" value="$(arg marker_length)"/>

    <param name ="dictionary" value="$(arg dictionary)"/>
  </node>
</launch>
```
##### live_stereo_calibrate_checker.launch
Run using launch file as below - this will subscribe to the given stereo images and collate images until you press 'e' in the terminal to proceed to calibration.

```
roslaunch stereo_calibration live_stereo_calibrate_checker.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <!--Save images and calibration data too folder - default "" = ~/calibration_images/M_D/N-->
  <arg name="save_directory" default=""/>

  <!--Image topic names-->
  <arg name="camera_left"  default="left/image_raw"/>
  <arg name="camera_right" default="right/image_raw"/>

  <!--Number of corners-->
  <arg name="board_width"   default="9"/>
  <arg name="board_height"  default="6"/>
  <arg name="square_length" default="25.0"/><!--mm-->
  <arg name="image_width"  default="1936"/>
  <arg name="image_height" default="1216"/>

  <!--Calibration modes-->
  <!--0 Checker-->
  <!--1 Charuco-->
  <arg name="method" default="0"/>

  <!--Difference Threshold-->
  <arg name="threshold" default="25"/><!--px-->

  <node name="stereo_calibration_node" pkg="stereo_calibration" type="stereo_calibration_node" output="screen">
    <param name ="save_directory"  value="$(arg save_directory)"/>

    <param name ="camera_left"  value="$(arg camera_left)"/>
    <param name ="camera_right" value="$(arg camera_right)"/>

    <param name ="board_width"  value="$(arg board_width)"/>
    <param name ="board_height" value="$(arg board_height)"/>
    <param name ="square_length" value="$(arg square_length)"/>

    <param name ="image_width"  value="$(arg image_width)"/>
    <param name ="image_height" value="$(arg image_height)"/>

    <param name ="method" value="$(arg method)"/>

    <param name ="threshold" value="$(arg threshold)"/>
  </node>
</launch>
```

##### file_stereo_calibrate_charuco.launch
Run using launch file as below - this will load the images in the given directory and find the given calibration.

```
roslaunch stereo_calibration file_stereo_calibrate_charuco.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <!--File Path-->
  <arg name="image_path"     default="$(env HOME)/calibration_images/..."/>
  <arg name="save_directory" default="$(arg image_path)"/>

  <!--Number of corners-->
  <arg name="board_width"   default="8"/>
  <arg name="board_height"  default="5"/>
  <arg name="square_length" default="45.0"/><!--mm-->

  <arg name="image_width"  default="1936"/>
  <arg name="image_height" default="1216"/>

  <!--Calibration modes-->
  <!--0 Checker-->
  <!--1 Charuco-->
  <arg name="method" default="1"/>

  <!--Difference Threshold-->
  <arg name="threshold" default="0"/><!--px-->

  <!--Charuco marker length if required-->
  <arg name="marker_length" default="23.0"/><!--mm-->

  <!--Dictionary ID-->
  <arg name="dictionary" default="0"/><!--DICT_4X4_50-->

  <node name="stereo_calibration_node" pkg="stereo_calibration" type="stereo_calibration_node" output="screen">
    <param name ="image_path"      value="$(arg image_path)"/>
    <param name ="save_directory"  value="$(arg save_directory)"/>

    <param name ="board_width"  value="$(arg board_width)"/>
    <param name ="board_height" value="$(arg board_height)"/>
    <param name ="square_length" value="$(arg square_length)"/>

    <param name ="image_width"  value="$(arg image_width)"/>
    <param name ="image_height" value="$(arg image_height)"/>

    <param name ="method" value="$(arg method)"/>

    <param name ="threshold" value="$(arg threshold)"/>

    <param name ="marker_length" value="$(arg marker_length)"/>

    <param name ="dictionary" value="$(arg dictionary)"/>
  </node>
</launch>
```

##### file_stereo_calibrate_checker.launch
Run using launch file as below - this will load the images in the given directory and find the given calibration.

```
roslaunch stereo_calibration file_stereo_calibrate_checker.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <!--File Path-->
  <arg name="image_path"     default="$(env HOME)/calibration_images/..."/>
  <arg name="save_directory" default="$(arg image_path)"/>

  <!--Number of corners-->
  <arg name="board_width"   default="9"/>
  <arg name="board_height"  default="6"/>
  <arg name="square_length" default="25.0"/><!--mm-->

  <arg name="image_width"  default="1936"/>
  <arg name="image_height" default="1216"/>

  <!--Calibration modes-->
  <!--0 Checker-->
  <!--1 Charuco-->
  <arg name="method" default="0"/>

  <!--Difference Threshold-->
  <arg name="threshold" default="0"/><!--px-->

  <node name="stereo_calibration_node" pkg="stereo_calibration" type="stereo_calibration_node" output="screen">
    <param name ="image_path"      value="$(arg image_path)"/>
    <param name ="save_directory"  value="$(arg save_directory)"/>

    <param name ="board_width"  value="$(arg board_width)"/>
    <param name ="board_height" value="$(arg board_height)"/>
    <param name ="square_length" value="$(arg square_length)"/>

    <param name ="image_width"  value="$(arg image_width)"/>
    <param name ="image_height" value="$(arg image_height)"/>

    <param name ="method" value="$(arg method)"/>

    <param name ="threshold" value="$(arg threshold)"/>
  </node>
</launch>
```

### Stereo Calibration Service
This node/service will run a calibration service- using the cares_msgs/CalibrationService service type on the service topic "stereo_calibration".

#### Service Msg
The service takes a directory location of stereo pair images and returns a resulting cares_msgs/StereoCameraInfo message.
The calibration data is also saved into the image_directory by the calibration service. 
```
string image_directory
---
cares_msgs/StereoCameraInfo stereo_info

```

##### service_stereo_calibration.launch
Run using launch file as below - this will setup the calibration service.

```
roslaunch stereo_calibration service_stereo_calibration.launch
```

```xml
<?xml version="1.0"?>
<launch>
  <!--Number of corners-->
  <arg name="board_width"   default="8"/>
  <arg name="board_height"  default="5"/>

  <arg name="square_length" default="45.0"/><!--mm-->

  <arg name="image_width"  default="1936"/>
  <arg name="image_height" default="1216"/>

  <!--Calibration modes-->
  <!--0 Checker-->
  <!--1 Charuco-->
  <arg name="method" default="1"/>

  <!--Difference Threshold-->
  <arg name="threshold" default="0"/><!--px-->

  <!--Charuco marker length if required-->
  <arg name="marker_length" default="23.0"/><!--mm-->

  <!--Dictionary ID-->
  <arg name="dictionary" default="0"/><!--DICT_4X4_50-->

  <node name="stereo_calibration_service" pkg="stereo_calibration" type="stereo_calibration_service" output="screen">
    <param name ="board_width"  value="$(arg board_width)"/>
    <param name ="board_height" value="$(arg board_height)"/>
    <param name ="square_length" value="$(arg square_length)"/>

    <param name ="image_width"  value="$(arg image_width)"/>
    <param name ="image_height" value="$(arg image_height)"/>

    <param name ="method" value="$(arg method)"/>

    <param name ="threshold" value="$(arg threshold)"/>

    <param name ="marker_length" value="$(arg marker_length)"/>

    <param name ="dictionary" value="$(arg dictionary)"/>
  </node>
</launch>
```

## Version

Version 1.0

## Authors

* **Henry Williams**

## License

TBD

## Acknowledgments



