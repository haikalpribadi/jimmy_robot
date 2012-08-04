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
#include "jimmy/Speak.h"

JimmyController::JimmyController() :
  linear_scale_(1.0), angular_scale_(1.0), max_freeze_(10), total_users_(5), search_repeat_(3)
{
  navigate_to_user_sub_ = node_handle_.subscribe("/jimmy/navigate_to_user", 1, &JimmyController::navigateToUserCallback, this);
  stop_controller_sub_ = node_handle_.subscribe("/jimmy/stop_controller", 1, &JimmyController::stopControllerCallback, this);
  emergency_status_srv_ = node_handle_.serviceClient<parallax_eddie_robot::GetStatus > ("emergency_status");
  speech_srv_ = node_handle_.serviceClient<jimmy::Speak > ("jimmy_speak");
  user_joint_srv_ = node_handle_.serviceClient<user_tracker::GetJointCoordinate > ("get_joint_coordinate");
  object_tracker_srv_ = node_handle_.serviceClient<object_tracker::GetObjectCoordinate>("get_object_coordinate");
  camera_angle_srv_ = node_handle_.serviceClient<user_tracker::GetCameraAngle > ("get_camera_angle");
  velocity_pub_ = node_handle_.advertise<parallax_eddie_robot::Velocity > ("/eddie/command_velocity", 1);
  camera_target_pub_ = node_handle_.advertise<user_tracker::Coordinate > ("camera_target", 1);
  camera_angle_pub_ = node_handle_.advertise<std_msgs::Float64 > ("/tilt_angle", 1);

  node_handle_.param("angular_scale", angular_scale_, angular_scale_);
  node_handle_.param("linear_scale", linear_scale_, linear_scale_);
  node_handle_.param("max_freeze", max_freeze_, max_freeze_);
  node_handle_.param("total_users", total_users_, total_users_);
  node_handle_.param("search_repeat", search_repeat_, search_repeat_);

  sem_init(&mutex_interrupt_, 0, 1);
  sem_wait(&mutex_interrupt_);
  stop_ = false;
  process_ = false;
  sem_post(&mutex_interrupt_);
}

void JimmyController::stopControllerCallback(const std_msgs::Empty::ConstPtr& message)
{
  sem_wait(&mutex_interrupt_);
  stop_ = true;
  sem_post(&mutex_interrupt_);
}

void JimmyController::navigateToUserCallback(const std_msgs::String::ConstPtr& message)
{
  sem_wait(&mutex_interrupt_);
  process_ = true;
  stop_ = false;
  destination_ = destination_user;
  navigate_mode_ = message->data;
  sem_post(&mutex_interrupt_);
}

void JimmyController::navigateToObjectCallback(const std_msgs::Empty::ConstPtr& message)
{
  sem_wait(&mutex_interrupt_);
  process_ = true;
  stop_ = false;
  destination_ = destination_object;
  sem_post(&mutex_interrupt_);
}

void JimmyController::navigateToUser(std::string mode)
{
  user_tracker::GetJointCoordinate torso;
  bool complete = false, found = false;
  while (ros::ok() && !complete && !stop_)
  {
    if(mode!=return_mode)
      found = searchUser(torso);
    else
      found = searchUserReturn(torso);
    
    if (found && !stop_)
      complete = driveToUser(torso, mode);
  }
  sem_wait(&mutex_interrupt_);
  stop_ = false;
  sem_post(&mutex_interrupt_);
  if (complete)
  {
    jimmy::Speak speech;
    speech.request.speech = "What can I help you with, $name ?";
    speech_srv_.call(speech);
  }
}

void JimmyController::navigateToObject()
{
  bool complete = false, found = false;
  while(ros::ok() && !complete && !stop_)
  {
    found = searchObject();
    if(found && !stop_)
      complete = driveToObject();
    sem_wait(&mutex_interrupt_);
    if(complete)
    {
      //notify jimmy has reached object
    }
  }
}

bool JimmyController::searchUser(user_tracker::GetJointCoordinate &torso)
{
  user_tracker::GetJointCoordinate right_hand;
  user_tracker::GetJointCoordinate left_hand;
  user_tracker::GetJointCoordinate neck;
  parallax_eddie_robot::Velocity velocity;
  //std_msgs::Float64 center;
  //center.data = 0;
  int counter = 0;
  bool stop = stop_;

  while (ros::ok() && !stop)
  {
    for (int i = 1; i <= total_users_ && !stop; i++)
    {
      //camera_angle_pub_.publish(center);
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
          ROS_INFO("User %d detected but neck and hands are not all detected", i);
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
          ROS_INFO("User %d detected but hand is not raised", i);
      }
      else
      {
        ROS_ERROR("Joint position not detected for user %d.", i);
      }
      ros::spinOnce();
      usleep(10);
      sem_wait(&mutex_interrupt_);
      stop = stop_;
      sem_post(&mutex_interrupt_);
      parallax_eddie_robot::GetStatus status;
      emergency_status_srv_.call(status);
      if (!status.response.okay)
        stop = true;
    }
    counter++;
    if (counter < search_repeat_)
      continue;
    counter = 0;
    if(!stop)
    {
      velocity.angular = 20;
      velocity.linear = 0;
      velocity_pub_.publish(velocity);
      usleep(10);
    }
  }
  return false;
}

bool JimmyController::searchUserReturn(user_tracker::GetJointCoordinate& torso)
{
  user_tracker::GetJointCoordinate right_hip;
  user_tracker::GetJointCoordinate left_hip;
  user_tracker::GetJointCoordinate right_knee;
  user_tracker::GetJointCoordinate left_knee;
  parallax_eddie_robot::Velocity velocity;
  //std_msgs::Float64 center;
  //center.data = 0;
  int counter = 0;
  bool stop = stop_;
  
  while(ros::ok() && !stop)
  {
    for(int i = 1; i <= total_users_ && !stop; i++)
    {
      //camera_angle_pub_.publish(center);
      std::stringstream ss;
      ss << frame_hip_right << "_" << i;
      right_hip.request.joint_frame = ss.str();
      ss.str(std::string());
      ss.clear();
      ss << frame_hip_left << "_" << i;
      left_hip.request.joint_frame = ss.str();
      ss.str(std::string());
      ss.clear();
      ss << frame_knee_right << "_" << i;
      right_knee.request.joint_frame = ss.str();
      ss.str(std::string());
      ss.clear();
      ss << frame_knee_left << "_" << i;
      left_knee.request.joint_frame = ss.str();
      ss.str(std::string());
      ss.clear();
      ss << frame_torso << "_" << i;
      torso.request.joint_frame = ss.str();
      
      if(user_joint_srv_.call(torso))
      {
        stopNavigating();
        bool has_right_hip = user_joint_srv_.call(right_hip);
        bool has_left_hip = user_joint_srv_.call(left_hip);
        bool has_right_knee = user_joint_srv_.call(right_knee);
        bool has_left_knee = user_joint_srv_.call(left_knee);
        if(!has_right_hip || !has_left_hip || !has_right_knee || !has_left_knee)
        {
          ROS_INFO("User %d detected but hips and knees are not all detected", i);
          usleep(10);
          i--;
          continue;
        }
        
        // Check if user is sitting down
        //
        float right_leg_slope, left_leg_slope;
        Coordinate rh, lh, rk, lk;
        rh.x = right_hip.response.x;
        rh.y = right_hip.response.y;
        rh.z = right_hip.response.z;
        lh.x = left_hip.response.x;
        lh.y = left_hip.response.y;
        lh.z = left_hip.response.z;
        rk.x = right_knee.response.x;
        rk.y = right_knee.response.y;
        rk.z = right_knee.response.z;
        lk.x = left_knee.response.x;
        lk.y = left_knee.response.y;
        lk.z = left_knee.response.z;
        right_leg_slope = (rh.z-rk.z) / sqrt(pow((rh.x-rk.x),2) + pow((rh.y-rk.y),2));
        left_leg_slope = (lh.z-lk.z) / sqrt(pow((lh.x-lk.x),2) + pow((lh.y-lk.y),2));
        
        if(right_leg_slope<0.5 && left_leg_slope<0.5)
          return true;
        else
          ROS_INFO("User %d detected but is not sitting down", i);
      }
      else
      {
        ROS_ERROR("Joint position not detected for user %d", i);
      }
      ros::spinOnce();
      usleep(10);
      sem_wait(&mutex_interrupt_);
      stop = stop_;
      sem_post(&mutex_interrupt_);
      parallax_eddie_robot::GetStatus status;
      emergency_status_srv_.call(status);
      if(!status.response.okay)
        stop = true;
    }
    counter++;
    if(counter < search_repeat_)
      continue;
    counter = 0;
    if(!stop)
    {
      velocity.angular = 20;
      velocity.linear = 0;
      velocity_pub_.publish(velocity);
      usleep(10);
    }
  }
  return false;
}

bool JimmyController::driveToUser(user_tracker::GetJointCoordinate torso, std::string mode)
{
  user_tracker::GetJointCoordinate previous;
  parallax_eddie_robot::Velocity velocity;
  int freeze_count = 0, track_count = 0, min_distance;
  user_joint_srv_.call(torso);
  previous = torso;
  bool stop = stop_;
  if (mode == follow_mode)
    min_distance = 70;
  else
    min_distance = 140;
  while (torso.response.x > min_distance && !stop)
  {
    //if (track_count == 0) targetCameraTilt(joint);
    Coordinate target;
    target.x = torso.response.x;
    target.y = torso.response.y;
    target.z = torso.response.z;
    velocity = setVelocity(target, mode);
    velocity_pub_.publish(velocity);

    if (!user_joint_srv_.call(torso))
    {
      stopNavigating();
      return false;
    }

    if (torso.response.x == previous.response.x &&
        torso.response.y == previous.response.y &&
        torso.response.z == previous.response.z)
    {
      freeze_count++;
      if (freeze_count > max_freeze_)
      {
        stopNavigating();
        return false;
      }
    }
    previous = torso;
    track_count++;
    if (track_count == 15) track_count = 0;
    ros::spinOnce();
    usleep(100);
    sem_wait(&mutex_interrupt_);
    stop = stop_;
    sem_post(&mutex_interrupt_);
    parallax_eddie_robot::GetStatus status;
    emergency_status_srv_.call(status);
    if (!status.response.okay)
      stop = true;
  }
  stopNavigating();
  if (stop)
    return false;
  else if (torso.response.x <= min_distance)
    return true;
  else
    return false;
}

bool JimmyController::searchObject()
{
  object_tracker::GetObjectCoordinate object;
  parallax_eddie_robot::Velocity velocity;
  int counter = 0;
  bool stop = stop_;
  
  while(ros::ok() && !stop)
  {
    if(object_tracker_srv_.call(object))
      return true;
    
    ros::spinOnce();
    usleep(10);
    sem_wait(&mutex_interrupt_);
    stop = stop_;
    sem_post(&mutex_interrupt_);
    parallax_eddie_robot::GetStatus status;
    emergency_status_srv_.call(status);
    if(!status.response.okay)
      stop = true;
    
    counter++;
    if(counter < search_repeat_)
      continue;
    counter = 0;
    if(!stop)
    {
      velocity.angular = 20;
      velocity.linear = 0;
      velocity_pub_.publish(velocity);
      usleep(10);
    }
  }
  return false;
}

bool JimmyController::driveToObject()
{
  object_tracker::GetObjectCoordinate object, previous;
  parallax_eddie_robot::Velocity velocity;
  int freeze_count = 0, track_count = 0, min_distance = 70;
  object_tracker_srv_.call(object);
  previous = object;
  bool stop = stop_;
  
  while(object.response.x > min_distance && !stop)
  {
    Coordinate target;
    target.x = object.response.x;
    target.y = object.response.y;
    target.z = object.response.z;
    
    velocity = setVelocity(target);
    
    if(!object_tracker_srv_.call(object))
    {
      stopNavigating();
      return false;
    }
    
    if(object.response.x == previous.response.x &&
       object.response.y == previous.response.y &&
       object.response.z == previous.response.z)
    {
      freeze_count++;
      if(freeze_count > max_freeze_)
      {
        stopNavigating();
        return false;
      }
    }
    
    previous = object;
    track_count++;
    if(track_count == 15) track_count = 0;
    ros::spinOnce();
    usleep(100);
    sem_wait(&mutex_interrupt_);
    stop = stop_;
    sem_post(&mutex_interrupt_);
    parallax_eddie_robot::GetStatus status;
    emergency_status_srv_.call(status);
    if(!status.response.okay)
      stop = true;
  }
  stopNavigating();
  if(stop)
    return false;
  else if(object.response.x <= min_distance)
    return true;
  else
    return false;
}

parallax_eddie_robot::Velocity JimmyController::setVelocity(
  Coordinate target, std::string mode)
{
  parallax_eddie_robot::Velocity velocity;
  velocity.angular = atan2(target.y, target.x) * 180 / PI;
  velocity.angular = velocity.angular > 180 ? velocity.angular - 360 : velocity.angular;
  velocity.angular = -1 * velocity.angular * angular_scale_;
  if (mode == follow_mode && target.x < 130 && target.x > 70)
  {
    velocity.linear = (((double) target.x - 150) / 80) * linear_scale_;
    velocity.angular = (-1 * velocity.angular * 0.5) - (angular_scale_ * 0.5);
  }
  else if (target.x < 200)
  {
    if (mode == follow_mode)
      velocity.linear = ((double) (target.x - 130) / 70) * linear_scale_;
    else
      velocity.linear = ((double) (target.x - 70) / 130) * linear_scale_;
  }
  else
    velocity.linear = 1.0 * linear_scale_;

  return velocity;
}

void JimmyController::stopNavigating()
{
  parallax_eddie_robot::Velocity velocity;
  velocity.angular = 0;
  velocity.linear = 0;
  velocity_pub_.publish(velocity);
  std_msgs::Float64 center;
  center.data = 0;
  //camera_angle_pub_.publish(center);
}

void JimmyController::targetCameraTilt(user_tracker::GetJointCoordinate joint)
{
  user_tracker::Coordinate camera_target;
  camera_target.x = joint.response.x;
  camera_target.y = joint.response.y;
  camera_target.z = joint.response.z;
  camera_target_pub_.publish(camera_target);
}

void JimmyController::execute()
{
  ros::Rate rate(1000);
  while (ros::ok())
  {
    sem_wait(&mutex_interrupt_);
    bool stop = stop_;
    bool ex = process_;
    sem_post(&mutex_interrupt_);
    if (ex && !stop)
    {
      sem_wait(&mutex_interrupt_);
      process_ = false;
      std::string destination = destination_;
      std::string mode = navigate_mode_;
      sem_post(&mutex_interrupt_);
      if(destination == destination_user)
        navigateToUser(mode);
      else
        navigateToObject();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

/*
 * 
 */
int main(int argc, char** argv)
{
  ROS_INFO("Jimmy Controller is booting up");
  ros::init(argc, argv, "jimmy_controller");
  JimmyController jimmyController;
  jimmyController.execute();

  return (EXIT_SUCCESS);
}

