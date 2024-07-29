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
// Created by haoming on 22.05.24.
//

#include "dataset/Dataset.h"
#include "offline_process/OfflineFGOBase.h"

namespace fgo::dataset {

  void DatasetBase::initialize(std::string name_, offline_process::OfflineFGOBase &node,
                               fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager) {

    name = std::move(name_);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": initializing DatasetBase ...");
    appPtr_ = &node;
    param_ = std::make_shared<DatasetParam>(reinterpret_cast<rclcpp::Node*>(&node), name);
    sensor_calib_manager_ = sensor_calib_manager;
    data_imu = DataBlock<IMUMeasurement>("IMU", param_->sensor_topic_map["imu"], 0., IMUDataTimeGetter);
    data_reference = DataBlock<PVASolution>("Reference", param_->sensor_topic_map["reference"], 0., PVADataTimeGetter);
    data_reference_state = DataBlock<State>("ReferenceState", "", 0., StateTimeGetter);
    reader_ = std::make_unique<BagReader>(param_);
    reader_->readFullDataFromBag();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": DatasetBase initialized!");
  }

  void DatasetBase::setTimestampsFromReference(bool reset_first) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name
                                                                                    << ": Setting start and end timestamps based on the reference data ...");
    if (timestamp_start.nanoseconds() == 0 || reset_first) {
      size_t first_pos = 0;
      auto iter = data_reference.data.begin();
      auto first_timestamp = iter->first;
      iter = data_reference.data.erase(iter);
      first_pos++;
      if (param_->start_offset != 0.) {
        while (iter != data_reference.data.end()) {
          if ((iter->first - first_timestamp).seconds() >= param_->start_offset)
            break;
          iter = data_reference.data.erase(iter);
          first_pos++;
        }
      }
      first_timestamp = iter->first;
      timestamp_start = first_timestamp;
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": setting start time " << timestamp_start.seconds());
      prior_state = referenceToState(iter->second);
      const auto imu_data_vec = data_imu.getDataBefore(first_timestamp);
      prior_state.omega = imu_data_vec.end()->gyro;
      prior_state.accMeasured = (gtsam::Vector6()
        << imu_data_vec.end()->accRot, imu_data_vec.end()->accLin).finished();

      if (data_reference_state.data.size() > first_pos) {
        auto eraseIter = data_reference_state.data.begin();
        std::advance(eraseIter, first_pos);
        data_reference_state.data.erase(data_reference_state.data.begin(), eraseIter);
        data_reference_state.data_iter = data_reference_state.data.begin();
      }
    }

    if (param_->pre_defined_duration == 0.) {
      timestamp_end = (--data_reference.data.end())->first;
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": setting the end time from parameter current");
      timestamp_end = timestamp_start + rclcpp::Duration::from_nanoseconds(param_->pre_defined_duration * 1e9);
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << " started from " << std::fixed << timestamp_start.seconds()
                                             << " end at " << timestamp_end.seconds());
  }
}