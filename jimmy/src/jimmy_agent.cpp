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

JimmyAgent::JimmyAgent() :
  jimmy_host_("localhost"), jimmy_port_(41000), state_(0)
{
  node_handle_.param("jimmy_host", jimmy_host_, jimmy_host_);
  node_handle_.param("jimmy_port", jimmy_port_, jimmy_port_);

  try
  {
    ClientSocket socket(jimmy_host_, jimmy_port_);
    socket_ = socket;
    ROS_INFO("Socket connection has been set up");
  }
  catch (SocketException& e)
  {
    ROS_ERROR("%s", e.description().data());
    ros::requestShutdown();
  }

  std::string file;
  if (node_handle_.getParam("drink_list", file))
  {
    ROS_INFO("Using drink_list file: %s", file.c_str());
    parseDrinks(file);
  }
  else
  {
    ROS_ERROR("Please set the drink_list (file) parameter for jimmy_agent");
    ros::requestShutdown();
  }
  
  if (node_handle_.getParam("response_list", file))
  {
    ROS_INFO("Using response_list file: %s", file.c_str());
    parseResponses(file);
  }
  else
  {
    ROS_INFO("Using default response messages");
    response_list_.push_back("Coming right up, $name !");
    response_list_.push_back("I am sorry, $name");
    response_list_.push_back("Yes, $name !");
    response_list_.push_back(" ");
    response_list_.push_back("Are you sure with your request, $name ?");
    response_list_.push_back("I am sorry I cannot help you with that, $name");
  }
  
  user_name_ = "Sir";
  jimmy_name_ = "Jimmy the robot";
  navigate_to_user_pub_ = node_handle_.advertise<std_msgs::String > ("/jimmy/navigate_to_user", 1);
  navigate_to_object_pub_ = node_handle_.advertise<std_msgs::Empty > ("/jimmy/navigate_to_object", 1);
  stop_controller_pub_ = node_handle_.advertise<std_msgs::Empty > ("/jimmy/stop_controller", 1);
  speak_srv_ = node_handle_.advertiseService("jimmy_speak", &JimmyAgent::speak, this);
  command_sub_ = node_handle_.subscribe("/speech/speech_to_command", 1, &JimmyAgent::commandCallback, this);
  accept_drink_sub_ = node_handle_.subscribe("jimmy/accept_drink", 1, &JimmyAgent::acceptDrinkCallback, this);
  speech_pub_ = node_handle_.advertise<std_msgs::String > ("/speech/text_to_speech_input", 1);
  velocity_pub_ = node_handle_.advertise<parallax_eddie_robot::Velocity > ("/eddie/command_velocity", 1);
  
}

void JimmyAgent::commandCallback(const std_msgs::String::ConstPtr& message)
{
  std::string command = message->data;

  if (command == cmd_jimmy)
  {
    speak("Yes, " + user_name_);
    navigateToUser(navigate_mode);
  }
  else if (command == cmd_hello)
    speak("Hi, " + user_name_);
  else if (command == cmd_jimmy_name)
    speak("My name is " + jimmy_name_);
  else if (command == cmd_how_are_you)
    speak("I'm fine thank you, " + user_name_);
  else if (command.substr(0, 7) == cmd_user_name && command.length() > 8)
    recordUserName(command.substr(8));
  else if (command == cmd_stop || command == cmd_halt ||
           command == cmd_kill || command == cmd_abort)
    stop();
  else if (command.substr(0, 3) == cmd_get && command.length() > 5)
    getDrink(command.substr(5));
  else
    commandUnrecognized();

}

void JimmyAgent::stop()
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);

  parallax_eddie_robot::Velocity velocity;
  velocity.angular = 0;
  velocity.linear = 0;
  velocity_pub_.publish(velocity);
}

void JimmyAgent::recordUserName(std::string name)
{
  std_msgs::String speech;

  user_name_ = name;
  speech.data = "Hello, " + user_name_ + ". How are you?";
  speech_pub_.publish(speech);
}

void JimmyAgent::speak(std::string text)
{
  std_msgs::String speech;
  stringReplace(text, "$name", user_name_);
  speech.data = text;
  speech_pub_.publish(speech);
}

bool JimmyAgent::speak(jimmy::Speak::Request& req, jimmy::Speak::Response& res)
{
  speak(req.speech);
  return true;
}

bool JimmyAgent::stringReplace(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

void JimmyAgent::commandUnrecognized()
{
  std_msgs::String speech;
  speech.data = "I'm sorry I don't understand, " + user_name_;
  speech_pub_.publish(speech);
}

void JimmyAgent::parseResponses(std::string filename)
{
  std::ifstream infile;
  std::string message;
  infile.open(filename.data());
  while(!infile.eof())
  {
    getline(infile, message);
    response_list_.push_back(message);
  }
  infile.close();
}

void JimmyAgent::parseDrinks(std::string filename)
{
  std::ifstream infile;
  std::string drink;
  infile.open(filename.data());
  while (!infile.eof())
  {
    getline(infile, drink);
    if (drink.size() == 0)
      continue;
    drink_list_.push_back(drink);
  }
  infile.close();
}

bool JimmyAgent::verifyDrink(std::string drink)
{
  for (uint i = 0; i < drink_list_.size(); i++)
  {
    if (drink == drink_list_[i])
      return true;
  }
  return false;
}

void JimmyAgent::navigateToUser(std::string mode)
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);

  std_msgs::String navigate;
  navigate.data = mode;
  navigate_to_user_pub_.publish(navigate);
}

void JimmyAgent::navigateToBar()
{
  std_msgs::Empty stop;
  stop_controller_pub_.publish(stop);
  
  std_msgs::Empty navigate;
  navigate_to_object_pub_.publish(navigate);
}

void JimmyAgent::getDrink(std::string drink)
{
  if (verifyDrink(drink))
  {
    int response = respondToCommand("get", drink);
    if(response == 0 || response == 2 || response == 3)
      processOrder(drink);
    else if(response == 1)
      speak("I am afraid we are out of " + drink);
  }
  else
    speak("I'm sorry we don't have " + drink + ", " + user_name_);
}

void JimmyAgent::changeDrink(std::string drink)
{
  if (verifyDrink(drink))
  {
    int response = respondToCommand("change", drink);
    if(response == 0 || response == 2 || response == 3)
      processOrder(drink);
    else if(response == 1)
      speak("I am afraid we are out of " + drink);
  }
  else
    speak("I'm sorry we don't have " + drink + ", " + user_name_);
}

void JimmyAgent::incorrectDrink()
{
  respondToCommand("incorrect", current_drink_);
  navigateToBar();
  //return drink
  current_drink_ = "";
}

void JimmyAgent::rejectDrink()
{
  respondToCommand("reject", current_drink_);
  navigateToBar();
  //return drink
  current_drink_ = "";
}

void JimmyAgent::acceptDrinkCallback(const std_msgs::Empty::ConstPtr& message)
{
  if(state_ == 3)
    acceptDrink();
}

void JimmyAgent::acceptDrink()
{
  socket_ << generateCommand("accept", current_drink_);
  current_drink_ = "";
  navigateToBar();
}

int JimmyAgent::respondToCommand(std::string cmd, std::string drink1, std::string drink2)
{
  int response;
  std::string reply;
  std::string command = drink2=="" ? generateCommand(cmd, drink1) : generateCommand(cmd, drink1, drink2);
  socket_ << command;
  socket_ >> reply;
  response = atoi(reply.data());
  
  speak(response_list_[response]);
  
  return response;
}

std::string JimmyAgent::generateCommand(std::string cmd, std::string drink1, std::string drink2)
{
  std::stringstream ss;
  ss << cmd << " " << stringReplace(user_name_, " ", "_") << " " << stringReplace(drink1, " ", "_");
  if(drink2!="")
    ss << " " << stringReplace(drink2, " ", "_");
  ss << "\n";
  return ss.str();
}

void JimmyAgent::processOrder(std::string drink)
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

