//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (haoming.zhang@rwth-aachen.de)
//
//

//
// Created by haoming on 12.06.24.
//

#ifndef ONLINE_FGO_DATASETKITTI_H
#define ONLINE_FGO_DATASETKITTI_H
#pragma once

#include <ranges>
#include <image_transport/image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <robognss_msgs/msg/pvt.hpp>
#include "include/dataset/Dataset.h"
#include "sensor/gnss/RoboGNSSParser.h"


namespace fgo::dataset {
  using namespace fgo::data;

  struct KittiDataBatch : DataBatch {
    std::vector<StereoPair> stereo_mono_batch;
    std::vector<StereoPair> stereo_rgb_batch;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_batch;
    std::vector<Pose> tf_batch;
  };


  class Kitti : public DatasetBase {
  protected:
    DataBlock<StereoPair> data_stereo_mono;
    DataBlock<StereoPair> data_stereo_rgb;
    DataBlock<sensor_msgs::msg::PointCloud2::SharedPtr> data_lidar;
    DataBlock<Pose> data_tf;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_{};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
    std::shared_ptr<image_transport::ImageTransport> it;
    image_transport::Publisher pub_left_mono;
    image_transport::Publisher pub_right_mono;
    image_transport::Publisher pub_left_rgb;
    image_transport::Publisher pub_right_rgb;

  public:
    explicit Kitti() = default;
    ~Kitti() override = default;

    void initialize(std::string name_,
                    offline_process::OfflineFGOBase &node,
                    fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager) override;

    void parseDataFromBag() override;

    void trimDataBlocks() override {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": trimming datablocks to start timestamp "
                                               << std::fixed << timestamp_start.seconds() << " and end timestamp "
                                               << timestamp_end.seconds());
      data_imu.trimData(timestamp_start, timestamp_end);
      data_reference_state.trimData(timestamp_start, timestamp_end);
      data_reference.trimData(timestamp_start, timestamp_end);
      data_stereo_mono.trimData(timestamp_start, timestamp_end);
      data_stereo_rgb.trimData(timestamp_start, timestamp_end);
      data_lidar.trimData(timestamp_start, timestamp_end);
      data_tf.trimData(timestamp_start, timestamp_end);
    }

    data::State referenceToState(const PVASolution &reference) override {
      static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
      return sensor::gnss::PVASolutionToState(reference, transReferenceFromBase.translation());
    }

    fgo::graph::StatusGraphConstruction feedDataToGraph(const std::vector<double> &stateTimestamps) override;

    KittiDataBatch getDataBefore(double timestamp,
                                  bool erase = false) {
      KittiDataBatch batch;
      batch.timestamp_end = timestamp;
      const auto ros_timestamp = utils::secondsToROSTime(timestamp);
      batch.imu = data_imu.getDataBefore(ros_timestamp, erase);
      batch.reference_pva = data_reference.getDataBefore(ros_timestamp, erase);
      batch.reference_state = data_reference_state.getDataBefore(ros_timestamp, erase);
      batch.stereo_mono_batch = data_stereo_mono.getDataBefore(ros_timestamp, erase);
      batch.stereo_rgb_batch = data_stereo_rgb.getDataBefore(ros_timestamp, erase);
      batch.lidar_batch = data_lidar.getDataBefore(ros_timestamp, erase);
      batch.tf_batch = data_tf.getDataBefore(ros_timestamp, erase);
      return batch;
    }

    KittiDataBatch getDataBetween(double timestamp_start,
                                   double timestamp_end) {
      KittiDataBatch batch;
      batch.timestamp_end = timestamp_end;
      const auto ros_timestamp_start = utils::secondsToROSTime(timestamp_start);
      const auto ros_timestamp_end = utils::secondsToROSTime(timestamp_end);
      batch.imu = data_imu.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_pva = data_reference.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_state = data_reference_state.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.stereo_mono_batch = data_stereo_mono.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.stereo_rgb_batch = data_stereo_rgb.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.lidar_batch = data_lidar.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.tf_batch = data_tf.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      return batch;
    }


  };


}


#endif //ONLINE_FGO_DATASETKITTI_H
