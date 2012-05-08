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


JimmyAgent::JimmyAgent()
{
  navigate_srv_ = node_handle_.serviceClient<jimmy::NavigateToUser>("navigate_to_user");
}

void JimmyAgent::test()
{
  for(int i=0; i<10; i++){
    ROS_INFO("Entering Jimmy Test 0");
    jimmy::NavigateToUser navigate;
    ROS_INFO("Entering Jimmy Test 1");
    ros::ServiceClient test;
    ROS_INFO("Entering Jimmy Test 2");
    test = node_handle_.serviceClient<jimmy::NavigateToUser>("navigate_to_user");
    ROS_INFO("Entering Jimmy Test 3");
    test.call(navigate);
    ROS_INFO("Entering Jimmy Test 4");
    usleep(1000000);
  }
  
}
/*
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "jimmy_agent");
  JimmyAgent agent;
  agent.test();
  return 0;
}

