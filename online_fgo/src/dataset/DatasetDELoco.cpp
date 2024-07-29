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

#include "dataset/impl/DatasetDELoco.h"
#include "offline_process/OfflineFGOBase.h"

namespace fgo::dataset {

  void DELoco::initialize(std::string name_, offline_process::OfflineFGOBase &node,
                          fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager) {
    DatasetBase::initialize(name_, node, sensor_calib_manager);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": preparing dataset ...");

    //data_param_deloco->topic_type_map = {
    //        {"/irt_gnss_preprocessing/PVT", fgo::data::DataType::IRTPVAGeodetic},
    //        {"/imu/data", fgo::data::DataType::IMU},
    //        {"/irt_gnss_preprocessing/gnss_obs_preprocessed", fgo::data::DataType::IRTGNSSObsPreProcessed},
    //};

    data_gnss = DataBlock<GNSSMeasurement>("gnss", "/irt_gnss_preprocessing/gnss_obs_preprocessed", 0.,
                                           GNSSDataTimeGetter);

    pub_pvt_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/gt_nav_fix",
                                                                  rclcpp::SystemDefaultsQoS());
    auto func_on_gt = [this](const std::vector<PVASolution> &data) -> void {
      for (const auto &pva: data) {
        sensor_msgs::msg::NavSatFix navfix;
        navfix.header.stamp = pva.timestamp;
        navfix.latitude = pva.llh[0] * constants::rad2deg;
        navfix.longitude = pva.llh[1] * constants::rad2deg;
        navfix.altitude = pva.llh[2];
        pub_pvt_->publish(navfix);
      }
    };
    data_reference.setCbOnData(func_on_gt);
    parseDataFromBag();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": prepared!");
  }

  void DELoco::parseDataFromBag() {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": Parsing raw data into data blocks ...");
    const auto raw_imu_msg = readROSMessages<sensor_msgs::msg::Imu>("/imu/data");
    DataBlock<IMUMeasurement>::DataMap imu_map;
    for (const auto &time_msg_pair: raw_imu_msg) {
      imu_map.insert(
        std::make_pair(time_msg_pair.first, convertIMUMsgToIMUMeasurement(time_msg_pair.second, time_msg_pair.first)));
    }
    data_imu.setData(imu_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed IMU data ...");

    const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");

    const auto raw_pva_msg = readROSMessages<irt_nav_msgs::msg::PVAGeodetic>("/irt_gnss_preprocessing/PVA");
    DataBlock<PVASolution>::DataMap gps_map;
    DataBlock<State>::DataMap gt_state_map;
    for (const auto &time_msg_pair: raw_pva_msg) {
      auto [pva, state] = sensor::gnss::parseIRTPVAMsg(time_msg_pair.second, gnss_param_ptr,
                                                       transReferenceFromBase.translation());
      gps_map.insert(std::make_pair(time_msg_pair.first, pva));
      gt_state_map.insert(std::make_pair(time_msg_pair.first, state));
    }
    data_reference_state.setData(gt_state_map, true);
    data_reference.setData(gps_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed gnss PVA data ...");

    const auto raw_gnss = readROSMessages<irt_nav_msgs::msg::GNSSObsPreProcessed>(
      "/irt_gnss_preprocessing/gnss_obs_preprocessed");
    DataBlock<GNSSMeasurement>::DataMap gnss_map;
    for (const auto &time_msg_pair: raw_gnss) {
      gnss_map.insert(std::make_pair(time_msg_pair.first,
                                     sensor::gnss::convertGNSSObservationMsg(time_msg_pair.second, gnss_param_ptr)));
    }
    data_gnss.setData(gnss_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed gnss data ...");
    setTimestampsFromReference();
    trimDataBlocks();
  }

  fgo::graph::StatusGraphConstruction DELoco::feedDataToGraph(const vector<double> &stateTimestamps) {
    const auto &last_timestamp = stateTimestamps.back();

    // next step we get all data from the database
    auto data_batch = this->getDataBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator

    appPtr_->propagateIMU(data_batch.imu);
    //auto gnss = data_deloco_->getGNSSObservationBefore(last_timestamp, true);
    auto gps_integrator_base = appPtr_->getGraphPtr()->getIntegrator("IRTPVALCIntegrator");
    auto gps_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator> &>(gps_integrator_base);
    gps_integrator->feedRAWData(data_batch.reference_pva, data_batch.reference_state);
    appPtr_->updateReferenceBuffer(data_batch.reference_pva);
    return appPtr_->getGraphPtr()->constructFactorGraphOnTime(stateTimestamps, data_batch.imu);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::dataset::DELoco, fgo::dataset::DatasetBase)