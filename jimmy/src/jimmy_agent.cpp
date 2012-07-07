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

#include "jimmy_agent.h"


JimmyAgent::JimmyAgent():
  linear_scale_(1.0), angular_scale_(1.0)
{
  user_name_ = "Sir";
  jimmy_name_ = "Jimmy the robot";
  navigate_to_user_pub_ = node_handle_.advertise<std_msgs::String>("/jimmy/navigate_to_user", 1);
  stop_controller_pub_ = node_handle_.advertise<std_msgs::Empty>("/jimmy/stop_controller", 1);
  speak_srv_ = node_handle_.advertiseService("jimmy_speak", &JimmyAgent::speak, this);
  command_sub_ = node_handle_.subscribe("/speech/speech_to_command", 1, &JimmyAgent::commandCallback, this);
  speech_pub_ = node_handle_.advertise<std_msgs::String > ("/speech/text_to_speech_input", 1);
  velocity_pub_ = node_handle_.advertise<parallax_eddie_robot::Velocity > ("/eddie/command_velocity", 1);
  
  node_handle_.param("angular_scale", angular_scale_, angular_scale_);
  node_handle_.param("linear_scale", linear_scale_, linear_scale_);
  
  driving_ = false;
}

void JimmyAgent::commandCallback(const std_msgs::String::ConstPtr& message)
{
  std::string command = message->data;
  
  if(command==cmd_jimmy)
    sayYes();
  else if(command==cmd_come_here)
    navigateToUser();
  else if(command==cmd_hello)
    sayHello();
  else if(command==cmd_help)
    offerHelp();
  else if(command==cmd_follow_me || command==cmd_come_with_me)
    followUser();
  else if(command==cmd_jimmy_name)
    sayName();
  else if(command==cmd_how_are_you)
    sayFeeling();
  else if(command.substr(0, 7)==cmd_user_name && command.length()>8)
    recordUserName(command.substr(8));
  else if(command==cmd_stop || command==cmd_halt || 
          command==cmd_kill || command==cmd_abort)
    stop();
  else if(command==cmd_move_forward || command==cmd_go_forward)
    drive(forward);
  else if(command==cmd_move_backward || command==cmd_go_backward)
    drive(backward);
  else if(command==cmd_faster)
    changeSpeed(faster);
  else if(command==cmd_slower)
    changeSpeed(slower);
  else if(command==cmd_turn_left)
    turn(-45);
  else if(command==cmd_turn_right)
    turn(45);
  else if(command==cmd_slant_left)
    turn(-20);
  else if(command==cmd_slant_right)
    turn(20);
  else if(command==cmd_rotate_left)
    turn(-360);
  else if(command==cmd_rotate_right)
    turn(360);
  else if(command==cmd_steer_left)
    steer(-45);
  else if(command==cmd_steer_right)
    steer(45);
  else
    commandUnrecognized();
    
}

bool JimmyAgent::speak(jimmy::Speak::Request& req, jimmy::Speak::Response& res)
{
  std_msgs::String speech;
  speech.data = req.speech;
  stringReplace(speech.data, "$name", user_name_);
  speech_pub_.publish(speech);
  return true;
}

bool JimmyAgent::stringReplace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

void JimmyAgent::sayYes()
{
  std_msgs::String speech;
  
  speech.data = "Yes, " + user_name_;
  speech_pub_.publish(speech);
}

void JimmyAgent::sayName()
{
  std_msgs::String speech;
  
  speech.data = "My name is " + jimmy_name_;
  speech_pub_.publish(speech);
}

void JimmyAgent::sayHello()
{
  std_msgs::String speech;
  
  speech.data = "Hi, " + user_name_;
  speech_pub_.publish(speech);
}

void JimmyAgent::offerHelp()
{
  std_msgs::String speech;
  
  speech.data = "What can I help you with, " + user_name_;
  speech_pub_.publish(speech);
}
void JimmyAgent::sayFeeling()
{
  std_msgs::String speech;
  
  speech.data = "I'm fine thank you, " + user_name_;
  speech_pub_.publish(speech);
}

void JimmyAgent::recordUserName(std::string name)
{
  std_msgs::String speech;
  
  user_name_ = name;
  speech.data = "Hello, " + user_name_ + ". How are you?";
  speech_pub_.publish(speech);
}

void JimmyAgent::navigateToUser()
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);
  
  std_msgs::String speech;
  speech.data = "Yes, " + user_name_;
  speech_pub_.publish(speech);
  
  std_msgs::String navigate;
  navigate.data = navigate_mode;
  navigate_to_user_pub_.publish(navigate);
}

void JimmyAgent::followUser()
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);
  
  std_msgs::String speech;
  speech.data = "Yes, " + user_name_ + ". I'm coming";
  speech_pub_.publish(speech);
  
  std_msgs::String navigate;
  navigate.data = follow_mode;
  navigate_to_user_pub_.publish(navigate);
}

void JimmyAgent::stop()
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);
  
  parallax_eddie_robot::Velocity velocity;
  velocity.angular = 0;
  velocity.linear = 0;
  velocity_pub_.publish(velocity);
  
  driving_ = false;
  direction_ = 0;
  linear_ = 0;
}

void JimmyAgent::changeSpeed(int direction)
{
  if(driving_)
  {
    linear_ += 0.5 * direction;
    if(linear_<0)
      linear_ = 0;
    
    parallax_eddie_robot::Velocity velocity;
  
    velocity.angular = 0;
    velocity.linear = linear_ * direction_;
    velocity_pub_.publish(velocity);
  }
}

void JimmyAgent::drive(int direction)
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);
  
  parallax_eddie_robot::Velocity velocity;
  velocity.angular = 0;
  velocity.linear = linear_scale_ * direction;
  linear_ = linear_scale_;
  velocity_pub_.publish(velocity);
  driving_ = true;
  direction_ = direction;
}


void JimmyAgent::turn(int degree)
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);
  
  parallax_eddie_robot::Velocity velocity;
  velocity.angular = degree;
  velocity.linear = 0;
  velocity_pub_.publish(velocity);
  driving_ = false;
  direction_ = 0;
  linear_ = 0;
}

void JimmyAgent::steer(int degree)
{
  if(driving_)
  {
    parallax_eddie_robot::Velocity velocity;
    velocity.angular = degree;
    velocity.linear = linear_ * direction_;
    velocity_pub_.publish(velocity);
  }
}

void JimmyAgent::commandUnrecognized()
{
  
}
/*
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "jimmy_agent");
  JimmyAgent agent;

  ros::spin();

  return (EXIT_SUCCESS);
}

