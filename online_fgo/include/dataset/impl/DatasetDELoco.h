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

#ifndef ONLINE_FGO_DATSETDELOCO_H
#define ONLINE_FGO_DATSETDELOCO_H
#pragma once

#include "dataset/Dataset.h"

namespace fgo::dataset {
  using namespace fgo::data;

  struct DELocoBatch : DataBatch {
    std::vector<GNSSMeasurement> gnss_obs_novatel;
  };

  class DELoco : public DatasetBase {
  protected:
    DataBlock<GNSSMeasurement> data_gnss;

    integrator::param::IntegratorGNSSTCParamsPtr gnss_param_ptr;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_pvt_;

  public:
    explicit DELoco() = default;

    ~DELoco() override = default;

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
      data_gnss.trimData(timestamp_start, timestamp_end);
    }

    data::State referenceToState(const PVASolution &reference) override {
      static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
      return sensor::gnss::PVASolutionToState(reference, transReferenceFromBase.translation());
    }

    fgo::graph::StatusGraphConstruction feedDataToGraph(const std::vector<double> &stateTimestamps) override;

    DELocoBatch getDataBefore(double timestamp,
                              bool erase = false) {
      DELocoBatch batch;
      batch.timestamp_end = timestamp;
      const auto ros_timestamp = utils::secondsToROSTime(timestamp);
      batch.imu = data_imu.getDataBefore(ros_timestamp, erase);
      batch.reference_pva = data_reference.getDataBefore(ros_timestamp, erase);
      batch.reference_state = data_reference_state.getDataBefore(ros_timestamp, erase);
      batch.gnss_obs_novatel = data_gnss.getDataBefore(ros_timestamp, erase);
      return batch;
    }

    DELocoBatch getDataBetween(double timestamp_start,
                               double timestamp_end) {
      DELocoBatch batch;
      batch.timestamp_end = timestamp_end;
      const auto ros_timestamp_start = utils::secondsToROSTime(timestamp_start);
      const auto ros_timestamp_end = utils::secondsToROSTime(timestamp_end);
      batch.imu = data_imu.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_pva = data_reference.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_state = data_reference_state.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.gnss_obs_novatel = data_gnss.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      return batch;
    }
  };
}
#endif //ONLINE_FGO_DATSETDELOCO_H
