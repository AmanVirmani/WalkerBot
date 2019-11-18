/** ￼
* MIT License
* 
* Copyright (c) 2019 Aman Virmani
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
 */

/**
 *@file guidance.h
 *@author Aman Virmani
 *@copyright MIT License
 *@brief declares WalkerBot class methods
 */

#ifndef INCLUDE_WALKER_WALKER_H_
#define INCLUDE_WALKER_WALKER_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief      Class for WalkerBot.
 */
class WalkerBot {
 private:
  /// Variable to detect obstaclePresents
  bool obstaclePresent;
  /// Variable for velocities
  geometry_msgs::Twist msg;
  /// node handler
  ros::NodeHandle nh;
  /// publish velocities
  ros::Publisher publishVelocity;
  /// subscriber to laserscan topic
  ros::Subscriber subscribeLaser;
  /// variable to store linear speed of the bot
  float linearVelocity;
  /// variable to store angular speed of the bot
  float angularVelocity;
  /// variable to specify min. distance from obstacle befor bot starts turning
  float proximityThresh;

 public:
  /**
   * @brief      constructor for guidance
   */
  WalkerBot();
  /**
   * @brief      destroys object
   */
  ~WalkerBot();
  /**
   * @brief      callback function for laserscan
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief      Checks for obstacles nearby
   * @return     of type bool. 1 if obstacle found,
   *             0 otherwise
   */
  bool isObstaclePresent();
  /**
   * @brief      function which runs robot
   */
  void run();
};
#endif  // INCLUDE_WALKER_WALKER_H_
