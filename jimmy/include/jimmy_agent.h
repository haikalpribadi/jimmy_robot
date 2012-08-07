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

#ifndef _JIMMY_AGENT_H
#define	_JIMMY_AGENT_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <jimmy/Speak.h>
#include <parallax_eddie_robot/Velocity.h>
#include <vector>
#include <sstream>
#include <fstream>

#define cmd_hello "hello"
#define cmd_jimmy "jimmy"
#define cmd_user_name "my name"
#define cmd_jimmy_name "your name"
#define cmd_how_are_you "how are you"
#define cmd_come_here "come here"
#define cmd_get "get"
#define cmd_cancel "cancel"
#define cmd_change "change"
#define cmd_stop "stop"
#define cmd_halt "halt"
#define cmd_kill "kill"
#define cmd_abort "abort"

#define follow_mode "follow"
#define navigate_mode "navigate"
#define forward 1
#define backward -1
#define slower -1
#define faster 1

class JimmyAgent {
public:
    JimmyAgent();
private:
    ros::NodeHandle node_handle_;
    ros::ServiceServer speak_srv_;
    ros::Subscriber command_sub_;
    ros::Publisher navigate_to_user_pub_;
    ros::Publisher stop_controller_pub_;
    ros::Publisher speech_pub_;
    ros::Publisher velocity_pub_;
    
    bool auto_run_;
    std::string user_name_, jimmy_name_;
    double linear_scale_, angular_scale_, linear_;
    int direction_;
    bool driving_;
    
    std::vector<std::string> drink_list_;

    void parseDrinks(std::string filename);
    bool verifyDrink(std::string drink);
    void commandCallback(const std_msgs::String::ConstPtr& message);
    void navigateToUser();
    void getDrink(std::string drink);
    void processOrder(std::string drink);
    int decideResponse(std::string command);
    void sayYes();
    void sayName();
    void sayHello();
    void sayFeeling();
    void recordUserName(std::string name);
    void stop();
    bool speak(jimmy::Speak::Request& req, jimmy::Speak::Response& res);
    bool stringReplace(std::string& str, const std::string& from, const std::string& to);
    void commandUnrecognized();

};

#endif	/* _JIMMY_AGENT_H */

