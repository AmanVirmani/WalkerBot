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
 *@file guidance.cpp
 *@author Aman Virmani
 *@copyright MIT License
 *@brief implements walker navigation methods
 */

#include <iostream>
#include "walker/walker.h"

/**
 * @brief      Constructs the object.
 */
WalkerBot::WalkerBot() {
  /// define linear and angular speeds
  linearVelocity = 0.2;
  angularVelocity = 1.0;
  /// proximity Threshold before bot starts turning
  proximityThresh = 0.8;

  /// obstacle present is false by default
  obstaclePresent = false;

  /// Publish the velocity
  publishVelocity = nh.advertise <geometry_msgs::Twist>
  ("/mobile_base/commands/velocity", 1000);
  /// subscribeLaser to subscribe to laser scan data
  subscribeLaser = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1000,
      &WalkerBot::laserCallback, this);
  /// inititializing with zero velocity
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  /// publisher the initial velocity data
  publishVelocity.publish(msg);
}

/**
 * @brief      Class Destructor
 */
WalkerBot::~WalkerBot() {
  /// Stop the turtlebot at the end of the program
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  publishVelocity.publish(msg);
}

/**
 * @brief      Callback for the laser scan data
 * @param      msg, data value of laser scan output
 */
void WalkerBot::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < proximityThresh) {
      obstaclePresent = true;
      return;
    }
  }
  obstaclePresent = false;
}

/**
 * @brief      Returns the obstaclePresent flag
 * @return     of type bool return when obstacle is detected
 */
bool WalkerBot::isObstaclePresent() {
  return obstaclePresent;
}

/**
 * @brief      fundamental logic to run the robot
 */
void WalkerBot::run() {
  /// publish at 10 Hz
  ros::Rate loop(10);
  /// keep running until ros dies
  while (ros::ok()) {
    /// check if any obstacle is present
    if (isObstaclePresent()) {
      /// ros out obstacle found
      ROS_INFO("Obstacle present; Steering away!!");
      /// Stop the robot
      msg.linear.x = 0.0;
      /// Turn the robot
      msg.angular.z = angularVelocity;
    } else {
      ROS_INFO("Moving Forward");
      /// Stop turning
      msg.angular.z = 0.0;
      /// set forward speed
      msg.linear.x = linearVelocity;
    }

    /// Publish the velocity data
    publishVelocity.publish(msg);

    ros::spinOnce();
    loop.sleep();
  }
}
