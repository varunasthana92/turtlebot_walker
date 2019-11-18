/**
 * @file motion.cpp
 * @brief class definition for controlling motion of turtleBot
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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "motion.hpp"

Motion::Motion() {
  forward = 0;
  cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
  scanSub = nh.subscribe("/scan", 100, &Motion::laserCallback, this);
}

void Motion::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  /*
   * @brief storing scan data for further processing by other class methods
   */
  data.ranges = msg->ranges;
  return;
}

int Motion::moveBot() {
  vel.linear.x = 1;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.z = 0;
  cmdVelPub.publish(vel);
  forward = 0;
  return 1;
}

int Motion::rotateBot() {
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  /*
   * @brief angular velocity is set to 120 degrees by converting to radians
   */
  vel.angular.z = 120 * 3.14 / 180;
  cmdVelPub.publish(vel);
  return 1;
}

int Motion::collisionStatus() {
  /*
   * @brief loop to check if turtlebot is close to any obstacle
   * by parsing over the range data
   */
  for (auto i : data.ranges) {
    if (i < 1) {
      /*
       * @brief if found closer than 0.6 units, then forward is set to 0
       */
      forward = 0;
      return forward;
    }
  }
  forward = 1;
  return forward;
}
