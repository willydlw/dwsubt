/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <string>

/// \brief Robot base control class, running as a ROS node to control a robot.
/// Based upon subt_seed_node.cc 

class BaseController
{
   /// \brief Constructor
   /// The controller user the given name as a prefix of its topics, e.g., 
   /// "x1/cmd_vel" if name is specified as "x1"
   /// \param[in] name Name of the robot
   public: BaseController(const std::string &name);


   /// \brief ROS node handler
   private: ros::NodeHandle nh;

   /// \brief robot name
   private: std::string rname;
}

BaseController(const std::string &name)
{
   ROS_INFO("Waiting for /clock, /subt/start, and /subt/pose_from_artifact");
   ROS_WARN("TODO: does each robot need to wait for /subt/start??");

   ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->nh);

   // wait for the start service to be ready

   this->rname = name;
}


int main(int argc, char** argv)
{

   for(int i = 0; i < argc; ++i)
   {
      ROS_DEBUG_STREAM("argv[" << i << "]: " << argv[i]);
   }

   // initialize ros. calling ros::init() with argc and argv will remove 
   // ROS arguments from the command line.
   ros::init(argc, argv, argv[1]);

   ROS_INFO_STREAM("starting base control");
   std::string name;

   if(argc < 2 || std::strlen(argv[1]) == 0)
   {
      while(name.empty())
      {
         ros::master::V_TopicInfo masterTopics;
         ros::master::getTopics(masterTopics);

         for(ros::master::V_TopicInfo::iterator it = masterTopics.begin(); 
            it != masterTopics.end(); ++it)
         {
            const ros::master::TopicInfo &info = *it;
            if(info.name.find("battery_state") != std::string::npos)
            {
               int rpos = info.name.rfind("/");
               name = info.name.sbutstr(1, rpos - 1);
            }
         }

         if(name.empty())
         {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
         }
      }
   }
   else
   {
      name = argv[1];
   }

   BaseController(name);

   return 0;
   
}