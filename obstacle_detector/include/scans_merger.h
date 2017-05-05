/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

namespace obstacle_detector
{

class ScansMerger
{
public:
  ScansMerger(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void frontScanCallback(const sensor_msgs::LaserScan::ConstPtr front_scan);
  void rearScanCallback(const sensor_msgs::LaserScan::ConstPtr rear_scan);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

  void publishScan();
  void publishPCL();
  void publishAll();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber front_scan_sub_;
  ros::Subscriber rear_scan_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher pcl_pub_;
  ros::ServiceServer params_srv_;

  tf::TransformListener front_tf_;
  tf::TransformListener rear_tf_;

  bool front_scan_received_;
  bool rear_scan_received_;
  bool front_scan_error_;
  bool rear_scan_error_;

  std::vector<float> ranges_;
  std::vector<geometry_msgs::Point32> points_;

  // Parameters
  bool p_active_;
  bool p_publish_scan_;
  bool p_publish_pcl_;

  int p_ranges_num_;

  double p_min_scanner_range_;
  double p_max_scanner_range_;
  double p_max_x_range_;
  double p_min_x_range_;
  double p_max_y_range_;
  double p_min_y_range_;

  std::string p_frame_id_;
};

} // namespace obstacle_detector
