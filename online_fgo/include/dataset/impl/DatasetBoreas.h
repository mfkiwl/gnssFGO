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
// Created by haoming on 07.05.24.
//

#ifndef ONLINE_FGO_DATASETBOREAS_H
#define ONLINE_FGO_DATASETBOREAS_H
#pragma once

#include <boreas_msgs/msg/sensor_pose.hpp>
#include "include/dataset/Dataset.h"


namespace fgo::dataset {
  using namespace fgo::data;

  struct BoreasDataBatch : DataBatch {
    std::vector<PVASolution> gnss;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_raw;
    std::vector<cv_bridge::CvImagePtr> image;
    std::vector<boreas_msgs::msg::SensorPose> camera_pose;
    std::vector<boreas_msgs::msg::SensorPose> lidar_pose;
  };


  class Boreas : public DatasetBase {
  protected:
    DataBlock<PVASolution> data_pva_gps;
    DataBlock<cv_bridge::CvImagePtr> data_image;
    DataBlock<sensor_msgs::msg::PointCloud2::SharedPtr> data_lidar_raw;
    DataBlock<boreas_msgs::msg::SensorPose> data_camera_pose;
    DataBlock<boreas_msgs::msg::SensorPose> data_lidar_pose;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_{};
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gt_{};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_{};
    std::shared_ptr<image_transport::ImageTransport> it;
    image_transport::Publisher pub_image;

  public:
    explicit Boreas() = default;
    ~Boreas() override = default;

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
      data_pva_gps.trimData(timestamp_start, timestamp_end);
      data_lidar_pose.trimData(timestamp_start, timestamp_end);
      data_camera_pose.trimData(timestamp_start, timestamp_end);
      data_lidar_raw.trimData(timestamp_start, timestamp_end);
      data_image.trimData(timestamp_start, timestamp_end);
    }

    data::State referenceToState(const PVASolution &reference) override {
      static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
      return sensor::gnss::PVASolutionToState(reference, transReferenceFromBase.translation());
    }

    PVASolution getNextGNSSPVA() { return data_pva_gps.getNextData(); }

    fgo::graph::StatusGraphConstruction feedDataToGraph(const std::vector<double> &stateTimestamps) override;

    BoreasDataBatch getDataBefore(double timestamp,
                                  bool erase = false) {
      BoreasDataBatch batch;
      batch.timestamp_end = timestamp;
      const auto ros_timestamp = utils::secondsToROSTime(timestamp);
      batch.imu = data_imu.getDataBefore(ros_timestamp, erase);
      batch.reference_pva = data_reference.getDataBefore(ros_timestamp, erase);
      batch.reference_state = data_reference_state.getDataBefore(ros_timestamp, erase);
      batch.gnss = data_pva_gps.getDataBefore(ros_timestamp, erase);
      batch.image = data_image.getDataBefore(ros_timestamp, erase);
      batch.camera_pose = data_camera_pose.getDataBefore(ros_timestamp, erase);
      batch.lidar_raw = data_lidar_raw.getDataBefore(ros_timestamp, erase);
      batch.lidar_pose = data_lidar_pose.getDataBefore(ros_timestamp, erase);
      return batch;
    }

    BoreasDataBatch getDataBetween(double timestamp_start,
                                   double timestamp_end) {
      BoreasDataBatch batch;
      batch.timestamp_end = timestamp_end;
      const auto ros_timestamp_start = utils::secondsToROSTime(timestamp_start);
      const auto ros_timestamp_end = utils::secondsToROSTime(timestamp_end);
      batch.imu = data_imu.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_pva = data_reference.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_state = data_reference_state.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.gnss = data_pva_gps.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.image = data_image.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.camera_pose = data_camera_pose.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.lidar_raw = data_lidar_raw.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.lidar_pose = data_lidar_pose.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      return batch;
    }
  };


}
#endif //ONLINE_FGO_DATASETBOREAS_H
