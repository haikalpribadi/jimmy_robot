/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Haikal Pribadi nor the names of other
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "jimmy_controller.h"


JimmyController::JimmyController():
l_scale_(1.0), a_scale_(1.0)
{
  user_joint_srv_ = node_handle_.serviceClient<user_tracker::GetJointCoordinate>("get_joint_coordinate");
  camera_angle_srv_ = node_handle_.serviceClient<user_tracker::GetCameraAngle>("get_camera_angle");
  camera_target_pub_ = node_handle_.advertise<user_tracker::Coordinate>("camera_target", 1);
  set_angle_pub_ = node_handle_.advertise<std_msgs::Float64>("/tilt_angle", 1);
  navigate_to_user_srv_ = node_handle_.advertiseService("navigate_to_user", &JimmyController::navigateToUser, this);
  velocity_pub_ = node_handle_.advertise<parallax_eddie_robot::Velocity > ("/eddie/command_velocity", 1);
  
  node_handle_.param("scale_angular", a_scale_, a_scale_);
  node_handle_.param("scale_linear", l_scale_, l_scale_);
}

bool JimmyController::navigateToUser(jimmy::NavigateToUser::Request& req, jimmy::NavigateToUser::Response& res)
{
  ROS_INFO("Entering navigateToUser function");
  user_tracker::GetJointCoordinate right_hand;
  user_tracker::GetJointCoordinate left_hand;
  user_tracker::GetJointCoordinate neck;
  user_tracker::GetJointCoordinate torso;
  user_tracker::GetJointCoordinate previous;
  user_tracker::Coordinate camera_target;
  parallax_eddie_robot::Velocity velocity;
  std_msgs::Float64 center;
  center.data = 0;
  int counter = 0;
  
  while(ros::ok())
  {
    for(int i=1; i<=10; i++)
    {
      set_angle_pub_.publish(center);
      std::stringstream ss;
      ss << frame_hand_right << "_" << i;
      right_hand.request.joint_frame = ss.str();
      ss.str(std::string());ss.clear();
      ss << frame_hand_left << "_" << i;
      left_hand.request.joint_frame = ss.str();
      ss.str(std::string());ss.clear();
      ss << frame_neck << "_" << i;
      neck.request.joint_frame = ss.str();
      ss.str(std::string());ss.clear();
      ss << frame_torso << "_" << i;
      torso.request.joint_frame = ss.str();
      
      if(user_joint_srv_.call(torso))
      {
        velocity.angular = 0;
        velocity.linear = 0;
        velocity_pub_.publish(velocity);
        //camera_target.x = torso.response.x;
        //camera_target.y = torso.response.y;
        //camera_target.z = torso.response.z;
        //camera_target_pub_.publish(camera_target);
        usleep(10);
        bool has_neck = user_joint_srv_.call(neck);
        bool has_right = user_joint_srv_.call(right_hand);
        bool has_left = user_joint_srv_.call(left_hand);
        bool hand_raised = false;
        if(has_neck)
        {
          ROS_INFO("neck position x: %f, y: %f, z: %f", 
            neck.response.x, neck.response.y, neck.response.z);
        }
        else
        {
          ROS_INFO("User %d detected but neck is not detected", i);
          usleep(10);
          i--;
          continue;
        }
        if(has_left)
        {
          ROS_INFO("left hand position x: %f, y: %f, z: %f",
            left_hand.response.x, left_hand.response.y, left_hand.response.z);
          if(left_hand.response.z > neck.response.z)
            hand_raised = true;
        }
        if(has_right)
        {
          ROS_INFO("right hand position x: %f, y: %f, z: %f",
            right_hand.response.x, right_hand.response.y, right_hand.response.z);
          if(right_hand.response.z > neck.response.z)
            hand_raised = true;
        }
        if((!has_right && !has_left) || !hand_raised)
        {
          ROS_INFO("User %d detected but hand is not raised", i);
          //usleep(10);
          continue;
        }
        else if(hand_raised)
        {
          int freeze_count = 0;
          int track_count = 0;
          user_joint_srv_.call(torso);
          previous = torso;
          while(torso.response.x > 120)
          {
            //ROS_INFO("Tracking user at x: %f, y: %f, z: %f",
            //torso.response.x, torso.response.y, torso.response.z);
            if(track_count==0){
              camera_target.x = torso.response.x;
              camera_target.y = torso.response.y;
              camera_target.z = torso.response.z;
              camera_target_pub_.publish(camera_target);
            }
            velocity.angular = atan2(torso.response.y, torso.response.x) * 180 / PI;
            velocity.angular = velocity.angular>180 ? velocity.angular-360 : velocity.angular;
            velocity.angular = -1 * velocity.angular * a_scale_;
            velocity.linear = 1.0 * l_scale_;
            //ROS_INFO("Driving at angular: %f", velocity.angular);
            velocity_pub_.publish(velocity);
            if(!user_joint_srv_.call(torso))
              break;
            if(torso.response.x == previous.response.x && 
               torso.response.y == previous.response.y &&
               torso.response.z == previous.response.z){
              freeze_count++;
              if(freeze_count>5)
                break;
            }
            previous = torso;
            track_count++;
            if(track_count==5) track_count = 0;
            usleep(100);
          }
          velocity.angular = 0;
          velocity.linear = 0;
          velocity_pub_.publish(velocity);
          camera_target.x = 0;
          camera_target.y = 0;
          camera_target.z = 0;
          camera_target_pub_.publish(camera_target);
          //return true;
        }
      }
      else
      {
        ROS_ERROR("Joint position not detected for user %d.", i);
        //usleep(10);
        continue;
      }
    }
    counter++;
    if(counter<5)
      continue;
    
    counter = 0;
    velocity.angular = 45;
    velocity.linear = 0;
    //velocity_pub_.publish(velocity);
    //usleep(10);
  }
  return true;
}

void JimmyController::test()
{
  ROS_INFO("Entering Jimmy Test 0");
  jimmy::NavigateToUser navigate;
  ROS_INFO("Entering Jimmy Test 1");
  ros::ServiceClient test;
  ROS_INFO("Entering Jimmy Test 2");
  test = node_handle_.serviceClient<jimmy::NavigateToUser>("navigate_to_user");
  ROS_INFO("Entering Jimmy Test 3");
  test.call(navigate);
  ROS_INFO("Entering Jimmy Test 4");

  
}

/*
 * 
 */
int main(int argc, char** argv)
{
  ROS_INFO("Jimmy Controller is booting up");
  ros::init(argc, argv, "jimmy_controller");
  JimmyController jimmy;
  
  ros::spin();
  
  return (EXIT_SUCCESS);
}

