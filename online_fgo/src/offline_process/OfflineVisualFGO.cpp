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
// Created by haoming on 07.06.24.
//

#include "offline_process/OfflineVisualFGO.h"

namespace offline_process {

  OfflineVisualFGO::OfflineVisualFGO(const rclcpp::NodeOptions &opt) : OfflineFGOBase("OfflineVisualFGO", opt){
    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: Starting the OfflineVisualFGO Application ...");

    RCLCPP_INFO(this->get_logger(),
                "---------------------  OfflineFGOTimeCentricNode initializing! --------------------- ");

    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineVisualFGO: Data Container initializing");

    ::utils::RosParameter<std::string> used_dataset("used_dataset", *this);


    auto data_param_pohang = std::make_shared<DatasetParam>(*this, used_dataset.value());

    data_param_pohang->topic_type_map = {
      {"/gps/gps_nav_raw",   fgo::data::DataType::IRTPVAGeodetic},
      {"/gx5/baseline",  fgo::data::DataType::Pose},
      {"/gx5/imu_calib/data", fgo::data::DataType::IMU},
      {"/lidar_front/os_cloud_node/points", fgo::data::DataType::PointCloud2},
      {"/lidar_front/imu/data", fgo::data::DataType::IMU},
      {"/infrared/image", fgo::data::DataType::MonoImage},
      {"/radar/image", fgo::data::DataType::MonoImage},
    };


    //auto data_param_deloco = std::make_shared<DatasetParam>(*this, "DELoco");
    //data_param_deloco->topic_type_map = {
    //        {"/irt_gnss_preprocessing/PVT", fgo::data::DataType::IRTPVAGeodetic},
    //        {"/imu/data", fgo::data::DataType::IMU},
    //        {"/irt_gnss_preprocessing/gnss_obs_preprocessed", fgo::data::DataType::IRTGNSSObsPreProcessed},
    //};

    auto gps_integrator_base = graph_->getIntegrator("PohangGNSSLCIntegrator");
    auto gps_integrator_param = gps_integrator_base->getIntegratorBaseParamPtr();
    data_pohang_ = std::make_unique<DatasetPohang>(*this, data_param_pohang, gps_integrator_param, sensorCalibManager_);
    //data_deloco_ = std::make_unique<DatadetDELoco>(*this, data_param_deloco);

    RCLCPP_INFO_STREAM(this->get_logger(), "Data Container initialized");
    //paramsPtr_ = std::make_shared<gnss_fgo::GNSSFGOParams>();
    this->startOfflineProcess(data_pohang_->timestamp_start.seconds(), data_pohang_->timestamp_end.seconds());
    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: OfflineVisualFGO Initialized ...");
  }

  StatusGraphConstruction OfflineVisualFGO::feedDataPohang(const vector<double> &stateTimestamps) {

    const auto &last_timestamp = stateTimestamps.back();
    RCLCPP_INFO(this->get_logger(), "Feeding Boreas Data");
    // next step we get all data from the database

    auto data_batch = data_pohang_->getDataBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator
    RCLCPP_INFO_STREAM(this->get_logger(), "got imu of size " << data_batch.imu.size());
    propagate_imu(data_batch.imu);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu propagated ");

    RCLCPP_INFO_STREAM(this->get_logger(), "got gt of size " << data_batch.reference_pva.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "got gps of size " << data_batch.gnss.size());

    auto gps_integrator_base = graph_->getIntegrator("PohangGNSSLCIntegrator");
    auto gps_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator> &>(gps_integrator_base);
    gps_integrator->feedRAWData(data_batch.reference_pva, data_batch.reference_state);
    for (const auto &pva: data_batch.reference_pva)
      referenceBuffer_.update_buffer(pva, pva.timestamp);

    //auto beacon_integrator_base = graph_->getIntegrator("BeaconPohang");
    //auto beacon_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::BeaconIntegrator> &>(beacon_integrator_base);
    //beacon_integrator->feedRAWData(data_batch.stereo_pair);
    return graph_->constructFactorGraphOnTime(stateTimestamps, data_batch.imu);
    return SUCCESSFUL;
  }

  StatusGraphConstruction OfflineVisualFGO::feedDataOffline(const vector<double> &stateTimestamps) {
    return feedDataPohang(stateTimestamps);
  }

  fgo::data::State OfflineVisualFGO::getPriorState() {
    return data_pohang_->prior_state;
  }


}
