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

#include <chrono>

#include <geometry_msgs/Twist.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosgraph_msgs/Clock.h>

#include <std_srvs/SetBool.h>
#include <subt_msgs/PoseFromArtifact.h>

#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>

#include <string>
#include <thread>

// custom message include <package_name/header file>
#include <dwsubt/Turn.h>

#include "mission.h"

/// \brief Robot base control class, running as a ROS node to control a robot.
/// Based upon subt_seed_node.cc 

class BaseController
{
   /// \brief Constructor
   /// The controller user the given name as a prefix of its topics, e.g., 
   /// "x1/cmd_vel" if name is specified as "x1"
   /// \param[in] name Name of the robot
   public: BaseController(const std::string &name);



   /// \brief Initialization routine performed only at start up. Sends 
   /// start signal to subt. Initializes comm client, publishers, subscribers
   /// \return true when ros call /subt/start service is successful. 
   /// Otherwise returns false when calling service fails.
   public: bool StartUp();

   /// \brief Function to be called at start of robot's run to move from
   /// start position to entrance of tunnel, cave, urban.
   private: void MoveToEntrance(void);

   /// \brief state machine
   public: void Update();

   /// \brief subscriber callback
   public: void AvoidObstacleTurnCallback(const dwsubt::Turn::ConstPtr &msg);

   
   /// \brief robot name
   private: std::string rname;

   private: double turnAngleZ;
   private: double linearX;


   /// \brief ROS node handler
   private: ros::NodeHandle nh;

   /// \brief subscriber for avoid obstacle turn angle
   private: ros::Subscriber turnAngleSub;

   /// \brief publisher to send cmd_vel
   private: ros::Publisher velPub;

   /// \brief Communication client.
   private: std::unique_ptr<subt::CommsClient> client;

   /// \brief Client to request pose from origin.
   private: ros::ServiceClient originClient;

   /// \brief Service to request pose from origin.
   private: subt_msgs::PoseFromArtifact originSrv;

   /// \brief mission state
   private: MissionState missionState;

   /// \brief True if robot has arrived at destination
   private: bool arrived{false};

  

};

BaseController::BaseController(const std::string &name)
{
   ROS_INFO("Waiting for /clock, /subt/start, and /subt/pose_from_artifact");
   ROS_WARN("TODO: does each robot need to wait for /subt/start??");

   ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->nh);

   // wait for the start service to be ready
   ros::service::waitForService("/subt/start", -1);
   ros::service::waitForService("/subt/pose_from_artifact_origin", -1);
   ROS_WARN("remove wait for start when adapting for multiple robot launch");


   this->missionState = MissionState::AWAIT_START;

   ROS_DEBUG_STREAM("mission state: " << this->missionState);

   this->rname = name;
   ROS_INFO("Using robot name[%s]\n", this->rname.c_str());

   this->linearX = 0.0;
   this->turnAngleZ = 0.0;
}



/////////////////////////////////////////////////
bool BaseController::StartUp()
{
    // Send start signal
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response res;
    req.data = true;
    if (!ros::service::call("/subt/start", req, res))
    {
      ROS_ERROR("Unable to send start signal.");
      return false;
    }
    else
    {
      ROS_INFO("Sent start signal.");
    }

  
    // Create subt communication client
    //this->client.reset(new subt::CommsClient(this->name));
    //this->client->Bind(&Controller::CommClientCallback, this);

    // Create a cmd_vel publisher to control a vehicle.
    this->velPub = this->nh.advertise<geometry_msgs::Twist>(
        this->rname + "/cmd_vel", 1);

   // Create 
   this->turnAngleSub = this->nh.subscribe("/avoid_obstacle_turn", 5, &BaseController::AvoidObstacleTurnCallback, this);

   // request pose relative to the origin
   this->originClient = this->nh.serviceClient<subt_msgs::PoseFromArtifact>(
        "/subt/pose_from_artifact_origin", true);
   this->originSrv.request.robot_name.data = this->rname;
   
   return true;
}


void BaseController::Update()
{
   static int printCount = 0;

   switch(this->missionState)
   {
      case MissionState::AWAIT_START:
         ROS_DEBUG("changing state from AWAIT_START to INIT");
         this->missionState = MissionState::INIT;
      break;

      case MissionState::INIT:
         if( this->StartUp())
         {
            ROS_INFO_STREAM("robot " << this->rname << " completed initialization");
            this->missionState = MissionState::FIND_ENTRANCE;
         }
         else
         {
            ROS_ERROR("BaseController::Startup returned false");
         }   
      break;

      case MissionState::FIND_ENTRANCE:
         if(this->arrived)
         {
            this->missionState = MissionState::STANDBY;
            ROS_DEBUG("changing state from FIND_ENTRANCE to STANDBY");
            printCount = 1;
         }
         else
         {
            if(printCount % 10 == 0)
            {
               ROS_DEBUG("still seeking the entrance");
               ++printCount;
            }
            
            this->MoveToEntrance();
         }
      break;

      case MissionState::EXPLORE:
        {
           geometry_msgs::Twist msg;
           msg.linear.x = this->linearX;
           msg.angular.z = this->turnAngleZ;
           ROS_INFO("EXPLORE state, linear x: %f, turn angle z: %f", this->linearX, this->turnAngleZ);
           this->velPub.publish(msg);
        }
         
      break;

      case MissionState::STANDBY:
         this->missionState = MissionState::EXPLORE;
         ROS_DEBUG("changing state from STANDBY to EXPLORE");
      break;

      case MissionState::STOP:
         ROS_ERROR("BaseController::Update MissionState::STOP not completed");
      break;
   }
}

void BaseController::AvoidObstacleTurnCallback(const dwsubt::Turn::ConstPtr &msg)
{
   this->linearX = msg->linearx;
   this->turnAngleZ = msg->anglez;
}



void BaseController::MoveToEntrance(void)
{
  bool call = this->originClient.call(this->originSrv);
  // Query current robot position w.r.t. entrance
  if (!call || !this->originSrv.response.success)
  {
    ROS_ERROR("Failed to call pose_from_artifact_origin service, \
      robot may not exist, be outside staging area, or the service is \
      not available.");

    // Stop robot
    geometry_msgs::Twist msg;
    this->velPub.publish(msg);
    return;
  }

  auto pose = this->originSrv.response.pose.pose;

  // Simple example for robot to go to entrance
  geometry_msgs::Twist msg;

  // Distance to goal
  double dist = pose.position.x * pose.position.x +
    pose.position.y * pose.position.y;

  //ROS_INFO("Distance to goal: %.2f", dist);
  
  // Arrived
  if (dist < 0.3 || pose.position.x >= -0.3)
  {
    msg.linear.x = 0;
    msg.angular.z = 0;
    this->arrived = true;
    ROS_INFO("Arrived at entrance!");
    ROS_INFO("position: x %.2f, y %.2f, z %.2f", pose.position.x, pose.position.y, pose.position.z);
  }
  // Move towards entrance
  else
  {
    // Yaw w.r.t. entrance
    // Quaternion to yaw:
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code_2
    auto q = pose.orientation;
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    auto yaw = atan2(siny_cosp, cosy_cosp);

   // ROS_INFO("yaw: %.2f", yaw);
    //ROS_INFO("position: x %.2f, y %.2f, z %.2f", pose.position.x, pose.position.y, pose.position.z);

    auto facingFront = abs(yaw) < 0.1;
    auto facingEast = abs(yaw + M_PI * 0.5) < 0.1;
    auto facingWest = abs(yaw - M_PI * 0.5) < 0.1;

    auto onCenter = abs(pose.position.y) <= 1.0;
    auto westOfCenter = pose.position.y > 1.0;
    auto eastOfCenter = pose.position.y < -1.0;

    double linVel = 3.0;
    double angVel = 1.5;

    // Robot is facing entrance
    if (facingFront && onCenter)
    {
      msg.linear.x = linVel;
      msg.angular.z = angVel * -yaw;
    }
    // Turn to center line
    else if (!facingEast && westOfCenter)
    {
      msg.angular.z = -angVel;
    }
    else if (!facingWest && eastOfCenter)
    {
      msg.angular.z = angVel;
    }
    // Go to center line
    else if (facingEast && westOfCenter)
    {
      msg.linear.x = linVel;
    }
    else if (facingWest && eastOfCenter)
    {
      msg.linear.x = linVel;
    }
    // Center line, not facing entrance
    else if (onCenter && !facingFront)
    {
      msg.angular.z = angVel * -yaw;
    }
    else
    {
      ROS_ERROR("Unhandled case");
    }
  }

  this->velPub.publish(msg);

}


int main(int argc, char** argv)
{
   if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
   {
      ros::console::notifyLoggerLevelsChanged();
   }


   for(int i = 0; i < argc; ++i)
   {
      ROS_DEBUG_STREAM("argv[" << i << "]: " << argv[i]);
   }

   // initialize ros. calling ros::init() with argc and argv will remove 
   // ROS arguments from the command line.
   ros::init(argc, argv, argv[1]);

   ROS_INFO_STREAM("starting base control");
   std::string name;

   ROS_INFO_STREAM("after ros::init, argc: " << argc);
   for(int i = 0; i < argc; ++i)
   {
      ROS_DEBUG_STREAM("argv[" << i << "]: " << argv[i]);
   }

   
   if(argc < 2 || std::strlen(argv[1]) == 0)
   {
     
      while(name.empty())
      {
         ROS_DEBUG("retrieving robot name");
         ros::master::V_TopicInfo masterTopics;
         ros::master::getTopics(masterTopics);

         for(ros::master::V_TopicInfo::iterator it = masterTopics.begin(); 
            it != masterTopics.end(); ++it)
         {
            const ros::master::TopicInfo &info = *it;
            ROS_DEBUG_STREAM("TopicInfo:info.name " << info.name);
            if(info.name.find("battery_state") != std::string::npos)
            {
               int rpos = info.name.rfind("/");
               name = info.name.substr(1, rpos - 1);
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
      ROS_WARN_STREAM("Unexpected robot name is argv[1]: " << argv[1]);
      name = argv[1];
   }

   ROS_INFO_STREAM("extracted name: " << name);

   BaseController baseController(name);

   ROS_INFO("constructed base controller");

   ros::Rate loop_rate(10);
   while(ros::ok())
   {
      baseController.Update();
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
   
}