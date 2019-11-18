/**
 * @file motion.hpp
 * @brief class declaration for controlling motion of turtleBot
 * @author Varun Asthana
 *
 * Copyright [2019] Varun Asthana
 * All rights reserved.
 *
 * MIT License
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

#ifndef INCLUDE_TURTLEBOT_WALKER_MOTION_HPP_
#define INCLUDE_TURTLEBOT_WALKER_MOTION_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Motion {
 private:
  /*
   * @brief declaring a node handle variable
   */
  ros::NodeHandle nh;
  /*
   * @brief variable to publish velocity for turtlebot
   */
  ros::Publisher cmdVelPub;
  /*
   * @brief variable to subscribe to scan data
   */
  ros::Subscriber scanSub;
  /*
   * @brief variable to hold sensor scan data
   */
  sensor_msgs::LaserScan data;
  /*
   * @brief variable to set velocity for the turtlebot
   */
  geometry_msgs::Twist vel;
  /*
   * @brief variable to check if turtlebot should mode forward or rotate.
   * If forward is 1 then only linear velocity will be published. if forward
   * is 0 then only angular velocity will be published
   */
  int forward;

 public:
  /*
   * @brief constructor to initialize publisher, subscriber and forward variables
   */
  Motion();
  /*
   * @brief callback function for the subscriber scanSub
   * @param msg sensor_msgs::LaserScan::ConstPtr
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  /*
   * @brief function to set and publish the linear velocity of turtlebot
   */
  int moveBot();
  /*
   * @brief function to set and publish the angular velocity of turtlebot
   */
  int rotateBot();
  /*
   * @brief function to check if turtlebot is in close vicinity to any obstacle
   * Based on the status, function toggels the value of clsss member forward
   * between 0 and 1
   */
  int collisionStatus();
};
#endif /* INCLUDE_TURTLEBOT_WALKER_MOTION_HPP_ */
