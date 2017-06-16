/*
 * Camhelper node - Helper to add camera information message to a raw image stream
 * Made part of (and included licence from):
 * openRatSLAM
 *
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

#include "utils/utils.h"
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <image_transport/image_transport.h>

using namespace ratslam;
using namespace std;


ros::Publisher pub_ch;



void image_callback(sensor_msgs::ImageConstPtr image)
{
  //Create an empty CameraInfo message
  sensor_msgs::CameraInfo cam_info;

  //Copy sequence ID, time stamp and frame ID from the incoming picture
  cam_info.header = image->header;
  //Add camera calibration information (recieved from calibration tool) in cam_info message
  cam_info.height = 480;
  cam_info.width = 640;
  cam_info.distortion_model = "plumb_bob";
  std::vector<double> tmp;

  cam_info.D.push_back(0.14366000125377434);
  cam_info.D.push_back(-0.22402111310254338);
  cam_info.D.push_back(-0.0010960640203878421);
  cam_info.D.push_back(0.005730073657602402);
  cam_info.D.push_back(0.0);
  
  cam_info.K[0]=(500.3800934954465);
  cam_info.K[1]=(0.0);
  cam_info.K[2]=(326.23988631881633);
  cam_info.K[3]=(0.0);
  cam_info.K[4]=(499.23023745446886);
  cam_info.K[5]=(241.9362101300265);
  cam_info.K[6]=(0.0);
  cam_info.K[7]=(0.0);
  cam_info.K[8]=(1.0);  

  cam_info.R[0]=(1.0);
  cam_info.R[1]=(0.0);
  cam_info.R[2]=(0.0);
  cam_info.R[3]=(0.0);
  cam_info.R[4]=(1.0);
  cam_info.R[5]=(0.0);
  cam_info.R[6]=(0.0);
  cam_info.R[7]=(0.0);
  cam_info.R[8]=(1.0);

  cam_info.P[0]=(510.92626953125);
  cam_info.P[1]=(0.0);
  cam_info.P[2]=(329.8608014283818);
  cam_info.P[3]=(0.0);
  cam_info.P[4]=(0.0);
  cam_info.P[5]=(512.4221801757812);
  cam_info.P[6]=(241.52504018384207);
  cam_info.P[7]=(0.0);
  cam_info.P[8]=(0.0);
  cam_info.P[9]=(0.0);
  cam_info.P[10]=(1.0);
  cam_info.P[11]=(0.0);

  cam_info.binning_x = 0;
  cam_info.binning_y = 0;

  cam_info.roi.x_offset = 140;//40;//140;//180;
  cam_info.roi.width = 500;//600;//500;//420;
  cam_info.roi.y_offset = 210;//220;//210;//0;//30;//215;
  cam_info.roi.height = 270;//250;//270;//480;//330;//255; //image->height
  cam_info.roi.do_rectify = false;

  // //publish the camera information
  pub_ch.publish(cam_info);
  
  /*  
  ROS_DEBUG_STREAM("VO:image_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

  static nav_msgs::Odometry odom_output;

  vo->on_image(&image->data[0], (image->encoding == "bgr8" ? false : true), image->width, image->height, &odom_output.twist.twist.linear.x, &odom_output.twist.twist.angular.z);

  odom_output.header.stamp = image->header.stamp;
  odom_output.header.seq++;

  pub_vo.publish(odom_output);
  */
}


int main(int argc, char * argv[])
{
  
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "cam_helper");
  }
  ros::NodeHandle node;

  //NodeHandle::advertise() returns a ros::Publisher object, which serves two purposes: 
  //1) it contains a publish() method that lets you publish messages onto the topic it was created with, and 
  //2) when it goes out of scope, it will automatically unadvertise. 

  //Tell master this node will pulish a sensor_msgs::CameraInfo on topic "/camera/CameraInfo" with 0 queue size
  pub_ch = node.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 0);

  //Use ImageTransport to subscribe to image message 
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, image_callback);

  ros::spin();

  return 0;
  
  
  

  
  
  
  
/*  
  
  ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }

  std::string topic_root = "";

  boost::property_tree::ptree settings, general_settings, vo_settings;
  read_ini(argv[1], settings);
  ratslam::get_setting_child(vo_settings, settings, "visual_odometry", true);
  ratslam::get_setting_child(general_settings, settings, "general", true);
  ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  vo = new ratslam::VisualOdometry(vo_settings);
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMVisualOdometry");
  }
  ros::NodeHandle node;

  pub_vo = node.advertise<nav_msgs::Odometry>(topic_root + "/odom", 0);

  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe(topic_root + "/camera/image", 1, image_callback);

  ros::spin();

  return 0;

*/



}
