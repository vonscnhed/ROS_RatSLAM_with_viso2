/*
 * Odomhelper node - Helper functions for odometry data
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

////////////////////
///TEMPORARY TEXT///
////////////////////

//Both these indicate experience number
//ExperienceMap/MapMarker/header/seq
//ExperienceMap/RobotPose/header/seq


//These indicate template number
//LocalView/Template/current_id

//LocalView/Template/header/seq


//CHECK THIS ONE OUT!
//http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom


//////////////
///Includes///
//////////////

#include <iostream>

#include "utils/utils.h"
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <mavros_msgs/VFR_HUD.h>
#include <sensor_msgs/Imu.h>
#include <ratslam_ros/ViewTemplate.h>
#include <tf2_msgs/TFMessage.h>

#include <math.h>
#include <vector> //for vectors?
#include <numeric> //for for-summing :P
#include <ros/duration.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>


using namespace std; //TODO needed?
using namespace ratslam;
//////////////////
///Declarations///
//////////////////
//Publishers
ros::Publisher pub_oh;                    //Publishes message type: xxx::xxx on topic /xxx/xx TODO
ros::Publisher pub_exp_map_pose;//TODO keep?

//Subscribers
ros::Subscriber sub_oh;                   //Subscribes to message: xxx::xxx on topic /xxx/xx TODO
ros::Subscriber sub_heading;              //Subscribes to message: xxx::xxx on topic /xxx/xx TODO
ros::Subscriber sub_imu;                  //Subscribes to message: xxx::xxx on topic /xxx/xx TODO
ros::Subscriber sub_tf;                   //Subscribes to message: xxx::xxx on topic /xxx/xx TODO

//Messages
nav_msgs::Odometry odom; //Message generated and published  TODO

//Local variables? TODO
int current_heading_mavros;
int last_heading_mavros = -1;

double current_x_pos;
double current_y_pos;
double current_z_rot;



//*********
//NOT USED VARIABLES?
//*********
//Vectors for storing sequences of data
vector<double> rot_z_accumulator;
vector<double> trans_x_accumulator;
double rot_z_sum = 0;
double trans_x_sum = 0;
double rot_z_avg = 0;
double trans_x_avg = 0;
ros::Time current_time, last_lv_update; //Calulated time since last change in active local view cell id (ratSLAM).

ros::Duration update_duration;

//Store a certain amount of imu-data points to integrate over
double imu_x, imu_y, imu_z;
double vel_x_acc, vel_y_acc, vel_z_acc;

//Subscribe to the last heading data and weigh that into the "twist.angular.z""
int lastHeading;
double largestOdom = 0;




//////////////////////
///SUBSCRIBERS ONLY///
//////////////////////



/////////////////////////////////
///SUBSCRIBERS THAT RE-PUBLISH///
/////////////////////////////////

//Called when odometry-message is recieved on topic "/mono_odometer/odometry"
//Republishes refurbished odometry message
void odom_callback(const nav_msgs::Odometry::ConstPtr& given_odom)
{
  //Create empty odometry message
  nav_msgs::Odometry odom;

  //Fill new message with info
  odom.header = given_odom ->header;

  //TODO MOVE OUTSIDE FUNCTION TO VARIABLES OUTSIDE
  double scaleLinear = 1;        //0.85 from tuning;
  double scaleAngular = 1.90;    //1.8 from tuning;
  bool allowReverse = false;
  double maxiumVelocity = 3;
  double smallestAngle = 0;   //Smallest angular change between frames that is propagated TODO keep?
  double largestAngle = 10;
  
  //TRANSLATION
  //Restrain to forward motion if selected
  if(!allowReverse && (given_odom->twist.twist.linear.z < 0)){
        odom.twist.twist.linear.x = 0;
  }else{
        odom.twist.twist.linear.x = given_odom->twist.twist.linear.z*scaleLinear;
  }

  //Restrain maximum velocity
  if (abs(odom.twist.twist.linear.x) > maxiumVelocity){
    odom.twist.twist.linear.x = maxiumVelocity;
  }

  //ROTATION
  //Restrain to rotation of certain amount
  if(abs(given_odom->twist.twist.angular.y) > smallestAngle){
    odom.twist.twist.angular.z = -given_odom->twist.twist.angular.y*scaleAngular;
  }else{
    odom.twist.twist.angular.z = 0;
  }
   
  //Restrain maximum rotation
  if (abs(odom.twist.twist.linear.x) > largestAngle)
  {
   odom.twist.twist.linear.x = largestAngle;
  }
 
  //Publish new odometry message
  pub_oh.publish(odom);





//DUMP BELOW


//   double temp_rot_z;

//   if (last_heading_mavros != -1)
//   {
// //Calculate a change in direction on heading data
//     if (last_heading_mavros == last_heading_mavros)
//     {
//       temp_rot_z = 0;
//     }
//     else
//     {
//       if(current_heading_mavros >= 270)
//       {
//         current_heading_mavros - 270;
//       }
//       else if (current_heading_mavros >= 180){
//         current_heading_mavros - 180;
//       }
//       else if(current_heading_mavros >= 90)
//       {
//         current_heading_mavros - 90;
//       }


//       if(last_heading_mavros >= 270)
//       {
//         last_heading_mavros - 270;
//       }
//       else if (last_heading_mavros >= 180){
//         last_heading_mavros - 180;
//       }
//       else if(last_heading_mavros >= 90)
//       {
//         last_heading_mavros - 90;
//       }
//     }
//   }

//   temp_rot_z = -sin((current_heading_mavros-last_heading_mavros)*M_PI/180);//current_heading_mavros/180*M_PI;
//   //current_heading_mavros;//-2*M_PI*sin(current_heading_mavros);//temp_rot_z = -2*M_PI*sin(current_heading_mavros - last_heading_mavros);
//   last_heading_mavros = current_heading_mavros;


//WORKS SUPRINSINGLY WELL
// if (given_odom->twist.twist.linear.z*0.01*1000 > 10){
//   odom.twist.twist.linear.x = 10;
// }else{
//   odom.twist.twist.linear.x = given_odom->twist.twist.linear.z*0.01*1000;
// }


  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_mavros);
  // geometry_msgs::TransformStamped odom_trans;
  // //odom_trans.header.frame_id = "odom";
  // //odom_trans.child_frame_id = "base_link";

  // //robot's position in x,y, and z
  // odom_trans.transform.translation.x = given_odom->twist.twist.linear.z;
  // odom_trans.transform.translation.y = given_odom->twist.twist.angular.y;
  // odom_trans.transform.translation.z = 0.0;

  // //robot's heading in quaternion
  // odom_trans.transform.rotation = odom_quat;
  // odom_trans.header.stamp = current_time;
  // //publish robot's tf using odom_trans object
  // odom_broadcaster.sendTransform(odom_trans)

}

void heading_callback(const mavros_msgs::VFR_HUD::ConstPtr& heading_msg)
{
  current_heading_mavros = heading_msg->heading;

  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_mavros);
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "base_link";

  // //robot's position in x,y, and z
  // odom_trans.transform.translation.x = x_pos;
  // odom_trans.transform.translation.y = y_pos;
  // odom_trans.transform.translation.z = 0.0;

  // //robot's heading in quaternion
  // odom_trans.transform.rotation = odom_quat;
  // odom_trans.header.stamp = current_time;
  // //publish robot's tf using odom_trans object
  // odom_broadcaster.sendTransform(odom_trans)
}

void heading_callback2(const std_msgs::Float64::ConstPtr& heading_msg)
{
  current_heading_mavros = heading_msg->data;
}


void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  imu_x = imu_msg->linear_acceleration.x;
  imu_y = imu_msg->linear_acceleration.y;
  imu_z = imu_msg->linear_acceleration.z;
}



void viso2_poseCallback(const tf2_msgs::TFMessage::ConstPtr& tf_msg)
{

//`x` is pointing right, `y` downwards and `z` from the camera into the scene. The origin is where the camera's principle axis hits the image plane (as given in <<MsgLink(sensor_msgs/CameraInfo)>>).

//Roughly the steps of Viso2 are the following:
//  1. Find F matrix from point correspondences using RANSAC and 8-point algorithm
//  2. Compute E matrix using the camera calibration
//  3. Compute 3D points and R|t up to scale
//  4. Estimate the ground plane in the 3D points
// 5. Use `camera_height` and `camera_pitch` to scale points and R|t
//Another problem occurs when the camera performs just pure rotation: even if there are enough features, the linear system to calculate the F matrix degenerates.


  //Store tf-data and recalculate it do odometry
  //z from viso2 is x in ratslam
  //x from viso2 is y in ratslam
  double last_x_pos;
  double last_y_pos;

  
  double diff_x;
  double diff_y;
  double twist_angular_z;
  
  double x_trans_between_frames;
  double z_rot_between_frames;

  current_x_pos = tf_msg->transforms[0].transform.translation.z;
  current_y_pos = tf_msg->transforms[0].transform.translation.x;
  current_z_rot = tf_msg->transforms[0].transform.rotation.z;
  diff_x = current_x_pos - last_x_pos;
  diff_y = current_y_pos - last_y_pos;

//Get translated distance between frames
  x_trans_between_frames = sqrt(diff_x*diff_x + diff_y*diff_y);
//Get rotated angle between frames
  z_rot_between_frames = atan(abs(diff_y/diff_x));

  if(diff_x > 0){
    if(diff_y > 0){
      //Quadrant one
      twist_angular_z = cos(z_rot_between_frames)*M_PI/2;
    }else if(diff_y < 0){
      //Quadrant 4
      twist_angular_z = M_PI/2 + cos(z_rot_between_frames)*M_PI/2;
    }else{
      //Only translation happened
      twist_angular_z = 0;
    }
  }
  else if(diff_x < 0){
    if(diff_y > 0){
      //Quadrant 2
      twist_angular_z = -cos(z_rot_between_frames)*M_PI/2;
    }else if (diff_y < 0){
      //Quadrant 3
      twist_angular_z = -M_PI/2 - cos(z_rot_between_frames)*M_PI/2;
    }else{
      twist_angular_z = 0;
    }
  }else{
    //Assuming only rotation can happen with translation
    twist_angular_z = 0;
  }
  
//COMMENTED JUST NOW
  // //Create odometery message
  // nav_msgs::Odometry odom;
  // odom.header = tf_msg->transforms[0].header;

  // odom.twist.twist.linear.x = x_trans_between_frames; //*0.01*1000  
  // odom.twist.twist.angular.z = twist_angular_z;  
  // //odom.twist.twist.angular.z = -z_rot_between_frames*1.15;  
 
  // pub_oh.publish(odom);

  // //Update last pos to be the one measured at this call
  // last_x_pos = current_x_pos;
  // last_y_pos = current_y_pos;

 














//  double temp;
//  temp = tf_msg->transforms[0].transform.translation.x;
 
}


void lv_callback(const ratslam_ros::ViewTemplate::ConstPtr& lv_msg)
{
  // //Update time and reset the calculated summed odometry since laste LV-update
  // last_lv_update = ros::Time::now();
  // odom.twist.twist.angular.z = 0;
  // odom.twist.twist.linear.x = 0;


  // //Empty accumulator vectors and update the time of last lv_update
  // rot_z_accumulator.clear(); 
  // trans_x_accumulator.clear();
  // rot_z_sum = 0;
  // trans_x_sum = 0;
  // rot_z_avg = 0;
  // trans_x_avg = 0;


  ///LocalView/Template.current_id has id of current active local view cell.
  //If no new scenes are seen, this number stops changing
  //If this number decreases an old local view cell has been activated, which-->
    //injects energy into pose-cell network, if (currently) 6/10 or more LVs in a row are of an older sequence, a loop closure and relocalization will occur


//   //Create a nav_msgs::Odometry to be published TODO move outside?
//   nav_msgs::Odometry odom;
//   //Fill the message with information
//  // odom.header = lv_msg ->header;
//   odom.twist.twist.angular.z = rot_z_avg;
//   odom.twist.twist.linear.x = trans_x_avg;


//   //last_lv_update = ros::Time::now();
//   //update_duration



//   //Test to republish imu-signal
//  //odom.twist.twist.angular.x = imu_z;

//   //Publish the refurbished odometry message
//   pub_oh.publish(odom);
}


void viso_poseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose_msg->pose.position.z, pose_msg->pose.position.x, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, pose_msg->pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "CRAZYRATNAME"));


  static tf::TransformBroadcaster br_origin;
  tf::Transform transform_origin;
  transform_origin.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q_origin;
  q_origin.setRPY(0, 0, 0);
  transform_origin.setRotation(q);
  br_origin.sendTransform(tf::StampedTransform(transform_origin, ros::Time::now(),  "world", "base_link"));
  


// //NOW
// sub to /mono_odometer->pose/position.z
// sub to /mono_odometer->pose/position.x
// and set odom so that the below equals the above
// ExperienceMap/RobotPose->pose.position.x
// ExperienceMap/RobotPose->pose.position.y


geometry_msgs::PoseStamped mono_odom_pose;
//mono_odom_pose.header = given_odom.header;

mono_odom_pose.pose.position.x = pose_msg->pose.position.z;
mono_odom_pose.pose.position.y = pose_msg->pose.position.x;


//pub_exp_map_pose.publish(mono_odom_pose);




}

int main(int argc, char * argv[])
{
  //Initialize and name node, if not existing
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "odom_helper");
  }
  //Create a NodeHandle for publishing and subscribing
  ros::NodeHandle node;

  //SUBSCRIBERS
//Node for recieving odometry messages from vo-algorithm (saving odometry messages in an array which average is to be published in lv_callback)
  sub_oh = node.subscribe<nav_msgs::Odometry>("/visual_odometry/mono_odometer/odometry", 1, odom_callback);

  
  

  
  //PUSLISHERS
//Tell master this node will pulish a nav_msgs/Odometry on topic "/odom" with 0 queue size
  pub_oh = node.advertise<nav_msgs::Odometry>("/ratrover/odom", 0);

  
  
  
  
  
  
  
  
  
  //TODO REMOVE 2 below
//Node that is called when ratSLAM updates its odometry (here the constructed (by time integration) odometry message is "published and reset")
  //sub_experience_updates  = node.subscribe<ratslam_ros::ViewTemplate>("/ratrover/LocalView/Template", 1, lv_callback);

//Node for recieving tf-data
  sub_tf = node.subscribe<tf2_msgs::TFMessage>("/tf", 1, viso2_poseCallback);
  //sub_tf = node.subscribe<geometry_msgs::PoseStamped>("/visual_odometry/mono_odometer/pose", 1, viso_poseCallback);

//Node for recieving imu-data
  //sub_imu = node.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, imu_callback);
//Node for recieving heading_data (magnetometer??? TODO)
  //sub_heading = node.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud", 1, heading_callback);
  //sub_heading = node.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 1, heading_callback2);

  
  //pub_exp_map_pose = node.advertise<geometry_msgs::PoseStamped>("/ExperienceMap/RobotPose", 0);


  ros::spin();

  return 0;













  //NodeHandle::advertise() returns a ros::Publisher object, which serves two purposes: 
  //1) it contains a publish() method that lets you publish messages onto the topic it was created with, and 
  //2) when it goes out of scope, it will automatically unadvertise. 


  //Tell the master that this node will subscribe to the following topics



  
  //ros::Subscriber sub = node.subscribe("/pose", 10, &viso2_poseCallback);
  //ros::Subscriber sub = node.subscribe("/pose", 10, &ratslam_poseCallback);

  
  
  



// /ExperienceMap/RobotPose.pose.orientation.w represents y-coordinate on unity circle? (-1, 1) (geometry_msgs::PoseStamped)
// /ExperienceMap/RobotPose.pose.orientation.z represents x-coordinate on unity circle?

// /mavros/vfr_hud.heading (mavro_msgs/VFR_HUD) (0-360)
//   heading (int16)
// 0   -> 1, 0
// 90  -> 0, 1
// 180 -> -1,0
// 270 -> 0,-1
//OR NORTH-UPWARDS ALIGNED?
// 0   -> 0, 1
// 90  -> -1, 0
// 180 -> 0, -1
// 270 -> 1, 0

//TOPIC TO SUBSCRIBE TO
// /mavros/imu/data (sensor_msgs/Imu)
//   linear_acceleration (geometry_msgs/Vector3)

// /mavros/vfr_hud (mavro_msgs/VFR_HUD)
//   heading (int16)

// /mavros/rc/in ()



  
  
  

  
  
  
  
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
