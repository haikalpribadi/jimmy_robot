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

#ifndef _JIMMY_CONTROLLER_H
#define	_JIMMY_CONTROLLER_H

#include <ros/ros.h>
#include <semaphore.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <jimmy/Speak.h>
#include <parallax_eddie_robot/Velocity.h>
#include <parallax_eddie_robot/GetStatus.h>
#include <user_tracker/GetJointCoordinate.h>
#include <user_tracker/GetCameraAngle.h>
#include <user_tracker/Coordinate.h>
#include <object_tracker/GetObjectCoordinate.h>

#define frame_head "/head"
#define frame_neck "/neck"
#define frame_torso "/torso"
#define frame_shoulder_left "/left_shoulder"
#define frame_shoulder_right "/right_shoulder"
#define frame_elbow_left "/left_elbow"
#define frame_elbow_right "/right_elbow"
#define frame_hand_left "/left_hand"
#define frame_hand_right "/right_hand"
#define frame_hip_left "/left_hip"
#define frame_hip_right "/right_hip"
#define frame_knee_left "/left_knee"
#define frame_knee_right "/right_knee"
#define frame_foot_left "/left_foot"
#define frame_foot_right "/right_foot"
#define destination_user "user"
#define destination_object "object"
#define follow_mode "follow"
#define navigate_mode "navigate"
#define return_mode "return"

#define PI 3.14159265

class Coordinate{
public:
    Coordinate(){}
    int x;
    int y;
    int z;
};

class JimmyController{
public:
    JimmyController();
    void execute();
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber navigate_to_user_sub_;
    ros::Subscriber navigate_to_object_sub_;
    ros::Subscriber stop_controller_sub_;
    ros::Publisher velocity_pub_;
    ros::Publisher camera_target_pub_;
    ros::Publisher camera_angle_pub_;
    ros::ServiceClient emergency_status_srv_;
    ros::ServiceClient speech_srv_;
    ros::ServiceClient user_joint_srv_;
    ros::ServiceClient object_tracker_srv_;
    ros::ServiceClient camera_angle_srv_;

    sem_t mutex_interrupt_;
    double linear_scale_, angular_scale_;
    int max_freeze_, total_users_, search_repeat_;
    bool stop_;
    bool process_;
    std::string destination_;
    std::string navigate_mode_;
    
    void stopControllerCallback(const std_msgs::Empty::ConstPtr& message);
    void navigateToUserCallback(const std_msgs::String::ConstPtr& message);
    void navigateToObjectCallback(const std_msgs::Empty::ConstPtr& message);
    void navigateToUser(std::string mode);
    bool searchUser(user_tracker::GetJointCoordinate &torso);
    bool searchUserReturn(user_tracker::GetJointCoordinate &torso);
    bool driveToUser(user_tracker::GetJointCoordinate torso, std::string mode);
    void navigateToObject();
    bool searchObject();
    bool driveToObject();
    parallax_eddie_robot::Velocity setVelocity(Coordinate target, std::string mode=follow_mode);
    void targetCameraTilt(user_tracker::GetJointCoordinate joint);
    void stopNavigating();
    
};


#endif	/* _JIMMY_CONTROLLER_H */

