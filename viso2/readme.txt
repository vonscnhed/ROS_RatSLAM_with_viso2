#Setting up the package Edit the .launch files in accordance with the comments in them

#IF the camera driver is not publishing any sensor_msgs::CameraInfo with the images in the image stream This package includes a node named cam_helper that will a publish camera information message corresponding to each recieved raw image (sensor_msgs::Image)

    Get the calibration parameters using opencv calibration method also described at http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

    Add camera information in catkin_ws/src/ratslam_ros/src/main_camhelper.cpp by following comments in that file

    Run camerahelper node that is defined in catkin_ws/src/ratslam_ros/main_camhelper.cpp: rosrun ratslam_ros ratslam_camhelper

#To run this package

    Run desired camera publisher (in this case a camera stream from an Erle rover, a picamera is used that publishes messages of the type sensor_msgs::CompressedImage)

    Run the (modified) cam_helper.launch file which will to decompress the image stream to sensor_msgs::Image and republish it on a desired topic.

    ONLY IF CAMERA DRIVER IS NOT PUBLISING CAMERA INFORMATION
