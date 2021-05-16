/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */
#include "turtlebot3_gazebo/turtlebot3_drive.h"
#include <iostream>
#include <cstdio>
#include <ctime>

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30 * DEG2RAD;
  check_forward_dist_ = 0.80;
  check_side_dist_    = 0.50;

  cwh_dist = 0.50;
  min_left_dist = 0.19;
  lwh_dist = 0.30;
  min_front_dist = 0.10;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;
 
  diff = abs(tb3_pose_- prev_tb3_pose_);
  abs_val = abs(scan_data_[LEFT_FORWARD] - scan_data_[LEFT_BACK]);

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_ = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[5] = {0, 75, 90, 10, 270};

  for (int num = 0; num < 5; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;


  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub_.publish(cmd_vel);

  std::cout << "linear:" << cmd_vel.linear.x << "  angular:" << cmd_vel.angular.z << std::endl;
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;

  std::cout << "forward:" << check_forward_dist_  << "  side:" << check_side_dist_ << "  lwh:" << lwh_dist << std::endl;
  std::cout << "center:" << scan_data_[CENTER]  << "  left:" << scan_data_[LEFT] << "  left-forward:" << scan_data_[LEFT_FORWARD] << "  left-back:" << scan_data_[LEFT_BACK] << std::endl;
  
  switch(turtlebot3_state_num)
  {

    // Orientation
    case GET_TB3_DIRECTION:
     
      if (scan_data_[CENTER] > check_forward_dist_ && scan_data_[LEFT] > lwh_dist)
      {
        std::cout << "driving left" << std::endl;
        turtlebot3_state_num = TB3_LEFT_DRIVE;
      }
      
      else if (scan_data_[CENTER] > cwh_dist && min_left_dist < scan_data_[LEFT] && scan_data_[LEFT] < lwh_dist)
      {
        if(scan_data_[LEFT] < scan_data_[LEFT_BACK] && scan_data_[LEFT] < scan_data_[LEFT_FORWARD])
        {
          std::cout << "drive forward wh" << std::endl;
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }

        else if(scan_data_[LEFT] > scan_data_[LEFT_BACK])
        {
          std::cout << "turn left wh" << std::endl;
          turtlebot3_state_num = TB3_LEFT_WH;
        }

        else if(scan_data_[LEFT] > scan_data_[LEFT_FORWARD])
        {
          std::cout << "turn right wh" << std::endl;
          turtlebot3_state_num = TB3_RIGHT_WH;
        }

        else
        {
          std::cout << "UNKNOWN SCENARIO" << std::endl;
          turtlebot3_state_num = GET_TB3_DIRECTION;
        }

      }

      //U-TURN
      else if(scan_data_[CENTER] < cwh_dist && scan_data_[LEFT] < lwh_dist && scan_data_[RIGHT] < lwh_dist)
      {
        std::cout << "u turn" << std::endl;
        turtlebot3_state_num = TB3_U_TURN;
      }

      //RIGT TURN TO START WH
      else if (scan_data_[CENTER] < cwh_dist)
      {
        std::cout << "right turn  before wh" << std::endl;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }

      //RIGHT TURN
      else if (scan_data_[CENTER] < check_forward_dist_ && scan_data_[LEFT] < lwh_dist && scan_data_[RIGHT] > lwh_dist)
      {
        std::cout << "right turn general" << std::endl;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      } 

      else 
      {
        std::cout << "all false .. " << std::endl;
        turtlebot3_state_num = GET_TB3_DIRECTION;
      }
      
      break;
  
    // Drive forward 
    case TB3_DRIVE_FORWARD:

        // Drive forward
        std::cout << "TB3_DRIVE_FORWARD" << std::endl;
        updatecommandVelocity(0.15, 0.0);
        turtlebot3_state_num = GET_TB3_DIRECTION;
      
      break;

    // Turn right
    case TB3_RIGHT_TURN:
      
      std::cout << "TB3_RIGHT_TURN" << std::endl;
      updatecommandVelocity(0.0, -0.2);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
    

    // Turn left
    case TB3_LEFT_TURN:
      
      std::cout << "TB3_LEFT_TURN" << std::endl;
      updatecommandVelocity(0.0, 0.2);
      turtlebot3_state_num = GET_TB3_DIRECTION;
    
  
      break;

      // U_turn
    case TB3_U_TURN:

      if(fabs(prev_tb3_pose_ - tb3_pose_) >= 2.6)
      {
        turtlebot3_state_num = GET_TB3_DIRECTION;
      }
      
      else
      {
        std::cout << "TB3_U_TURN" << std::endl;
        updatecommandVelocity(0.0, -1.0);
        std::cout << "absolute-value" << fabs(prev_tb3_pose_ - tb3_pose_) << std::endl;
      }

      break;

    case TB3_LEFT_DRIVE:

      std::cout << "TB3_LEFT_DRIVE" << std::endl;
      updatecommandVelocity(0.05, 0.2);
      turtlebot3_state_num = GET_TB3_DIRECTION;

      break;

    case TB3_RIGHT_DRIVE:
    
      std::cout << "TB3_RIGHT_DRIVE" << std::endl;
      updatecommandVelocity(0.05, -0.2);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      
      break;

    case TB3_LEFT_WH:

      std::cout << "TB3_LEFT_WH" << std::endl;
      updatecommandVelocity(0.0, 0.05);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
    
    case TB3_RIGHT_WH:

      std::cout << "TB3_RIGHT_WH" << std::endl;
      updatecommandVelocity(0.0, -0.05);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    default:

      // Orientation
      turtlebot3_state_num = GET_TB3_DIRECTION;
      std::cout << "default reached ... " <<std::endl;
      break;
  }
  std::cout << "returning true .. " <<std::endl << std::endl;
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;
  std::clock_t start;
  double duration;

  start = std::clock(); // get current time
  
  ros::Rate loop_rate(125);
  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    std::cout << "Operation took "<< duration << "seconds" << std::endl;
  return 0;
}
