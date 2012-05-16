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

JimmyController::JimmyController() :
  l_scale_(1.0), a_scale_(1.0), max_freeze_(10), total_users_(5)
{
  user_joint_srv_ = node_handle_.serviceClient<user_tracker::GetJointCoordinate > ("get_joint_coordinate");
  camera_angle_srv_ = node_handle_.serviceClient<user_tracker::GetCameraAngle > ("get_camera_angle");
  camera_target_pub_ = node_handle_.advertise<user_tracker::Coordinate > ("camera_target", 1);
  camera_angle_pub_ = node_handle_.advertise<std_msgs::Float64 > ("/tilt_angle", 1);
  navigate_to_user_srv_ = node_handle_.advertiseService("navigate_to_user", &JimmyController::navigateToUser, this);
  follow_user_srv_ = node_handle_.advertiseService("follow_user", &JimmyController::followUser, this);
  velocity_pub_ = node_handle_.advertise<parallax_eddie_robot::Velocity > ("/eddie/command_velocity", 1);

  node_handle_.param("angular_scale", a_scale_, a_scale_);
  node_handle_.param("linear_scale", l_scale_, l_scale_);
  node_handle_.param("max_freeze", max_freeze_, max_freeze_);
  node_handle_.param("total_users", total_users_, total_users_);
}

bool JimmyController::navigateToUser(jimmy::NavigateToUser::Request& req, jimmy::NavigateToUser::Response& res)
{
  user_tracker::GetJointCoordinate torso;
  bool complete = false, found = false;
  while(ros::ok() && !complete)
  {
    found = searchUser(torso);
    if(found)
      complete = driveToUser(torso, navigate_mode);
  }
  return true;
}

bool JimmyController::followUser(jimmy::FollowUser::Request& req, jimmy::FollowUser::Response& res)
{
  user_tracker::GetJointCoordinate torso;
  bool complete = false, found = false;
  while(ros::ok() && !complete)
  {
    found = searchUser(torso);
    if(found)
      complete = driveToUser(torso, follow_mode);
  }
  return true;
}

bool JimmyController::searchUser(user_tracker::GetJointCoordinate &torso)
{
  user_tracker::GetJointCoordinate right_hand;
  user_tracker::GetJointCoordinate left_hand;
  user_tracker::GetJointCoordinate neck;
  parallax_eddie_robot::Velocity velocity;
  std_msgs::Float64 center;
  center.data = 0;
  int counter = 0;

  while (ros::ok())
  {
    for (int i = 1; i <= total_users_; i++)
    {
      camera_angle_pub_.publish(center);
      std::stringstream ss;
      ss << frame_hand_right << "_" << i;
      right_hand.request.joint_frame = ss.str();
      ss.str(std::string());
      ss.clear();
      ss << frame_hand_left << "_" << i;
      left_hand.request.joint_frame = ss.str();
      ss.str(std::string());
      ss.clear();
      ss << frame_neck << "_" << i;
      neck.request.joint_frame = ss.str();
      ss.str(std::string());
      ss.clear();
      ss << frame_torso << "_" << i;
      torso.request.joint_frame = ss.str();

      if (user_joint_srv_.call(torso))
      {
        stopNavigating();
        bool has_neck = user_joint_srv_.call(neck);
        bool has_right = user_joint_srv_.call(right_hand);
        bool has_left = user_joint_srv_.call(left_hand);
        bool hand_raised = false;
        if (!has_neck || !has_left || !has_right)
        {
          ROS_INFO("User %d detected but neck and hands are not detected", i);
          usleep(10);
          i--;
          continue;
        }
        if (left_hand.response.z > neck.response.z)
          hand_raised = true;
        if (right_hand.response.z > neck.response.z)
          hand_raised = true;
        if (hand_raised)
          return true;
        else
        {
          ROS_INFO("User %d detected but hand is not raised", i);
          continue;
        }
      }
      else
      {
        ROS_ERROR("Joint position not detected for user %d.", i);
        continue;
      }
    }
    counter++;
    if (counter < 3)
      continue;
    counter = 0;
    velocity.angular = 20;
    velocity.linear = 0;
    velocity_pub_.publish(velocity);
    usleep(10);
  }
  return false;
}

bool JimmyController::driveToUser(user_tracker::GetJointCoordinate joint, std::string mode)
{
  user_tracker::GetJointCoordinate previous;
  parallax_eddie_robot::Velocity velocity;
  int freeze_count = 0, track_count = 0, min_distance;
  user_joint_srv_.call(joint);
  previous = joint;
  if(mode==follow_mode)
    min_distance = 70;
  else 
    min_distance = 110;
  while (joint.response.x > min_distance)
  {
    if (track_count == 0) targetCameraTilt(joint);
    velocity = setVelocity(joint, mode);
    velocity_pub_.publish(velocity);

    if (!user_joint_srv_.call(joint))
    {
      stopNavigating();
      return false;
    }

    if (joint.response.x == previous.response.x &&
        joint.response.y == previous.response.y &&
        joint.response.z == previous.response.z)
    {
      freeze_count++;
      if (freeze_count > max_freeze_)
      {
        stopNavigating();
        return false;
      }
    }
    previous = joint;
    track_count++;
    if (track_count == 5) track_count = 0;
    usleep(100);
  }
  return true;
}

parallax_eddie_robot::Velocity JimmyController::setVelocity(
  user_tracker::GetJointCoordinate joint, std::string mode)
{
  parallax_eddie_robot::Velocity velocity;
  velocity.angular = atan2(joint.response.y, joint.response.x) * 180 / PI;
  velocity.angular = velocity.angular > 180 ? velocity.angular - 360 : velocity.angular;
  velocity.angular = -1 * velocity.angular * a_scale_;
  if (mode==follow_mode && joint.response.x < 130 && joint.response.x > 70)
  {
    velocity.linear = (((double) joint.response.x - 150) / 80) * l_scale_;
    velocity.angular = (-1 * velocity.angular * 0.5) - (a_scale_ * 0.5);
  }
  else if (joint.response.x < 200)
  {
    if(mode==follow_mode)
      velocity.linear = ((double) (joint.response.x - 130) / 70) * l_scale_;
    else
      velocity.linear = ((double) (joint.response.x - 70) / 130) * l_scale_;
  }
  else
    velocity.linear = 1.0 * l_scale_;

  return velocity;
}

void JimmyController::stopNavigating()
{
  parallax_eddie_robot::Velocity velocity;
  velocity.angular = 0;
  velocity.linear = 0;
  velocity_pub_.publish(velocity);
}

void JimmyController::targetCameraTilt(user_tracker::GetJointCoordinate joint)
{
  user_tracker::Coordinate camera_target;
  camera_target.x = joint.response.x;
  camera_target.y = joint.response.y;
  camera_target.z = joint.response.z;
  camera_target_pub_.publish(camera_target);
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

