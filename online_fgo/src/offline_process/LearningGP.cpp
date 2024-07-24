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

#include "integrator/GNSSLCIntegrator.h"
#include "integrator/GNSSTCIntegrator.h"
#include "offline_process/LearningGP.h"

namespace offline_process {
  LearningGP::LearningGP(const rclcpp::NodeOptions &opt) : OfflineFGOBase("OfflineFGOGPLearning", opt) {
    //Load databags

    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: Starting the LearningGP Application ...");

    RCLCPP_INFO(this->get_logger(),
                "---------------------  OfflineFGOTimeCentricNode initializing! --------------------- ");

    RCLCPP_INFO_STREAM(this->get_logger(), "LearningGP: Data Container initializing");

    auto data_param_boreas = std::make_shared<DatasetParam>(*this, "Boreas");

    data_param_boreas->topic_type_map = {
      {"/boreas/gps_gt",   fgo::data::DataType::Odometry},
      {"/boreas/gps_raw",  fgo::data::DataType::Odometry},
      {"/boreas/imu/data", fgo::data::DataType::IMU},
      {"/boreas/compressed_image", fgo::data::DataType::MonoImage},
      {"/boreas/velodyne_points", fgo::data::DataType::PointCloud2},
      {"/boreas/camera_pose", fgo::data::DataType::Pose},
      {"/boreas/lidar_pose", fgo::data::DataType::Pose},
    };


    //auto data_param_deloco = std::make_shared<DatasetParam>(*this, "DELoco");
    //data_param_deloco->topic_type_map = {
    //        {"/irt_gnss_preprocessing/PVT", fgo::data::DataType::IRTPVAGeodetic},
    //        {"/imu/data", fgo::data::DataType::IMU},
    //        {"/irt_gnss_preprocessing/gnss_obs_preprocessed", fgo::data::DataType::IRTGNSSObsPreProcessed},
    //};

    data_boreas_ = std::make_unique<DatasetBoreas>(*this, data_param_boreas, sensorCalibManager_);
    //data_deloco_ = std::make_unique<DatadetDELoco>(*this, data_param_deloco);

    RCLCPP_INFO_STREAM(this->get_logger(), "Data Container initialized");
    //paramsPtr_ = std::make_shared<gnss_fgo::GNSSFGOParams>();
    this->startOfflineProcess(data_boreas_->timestamp_start.seconds(), data_boreas_->timestamp_end.seconds());
    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: LearningGP Initialized ...");
  }

  fgo::data::State LearningGP::getPriorState() {
    return data_boreas_->prior_state;
  }

  StatusGraphConstruction LearningGP::feedDataOffline(const vector<double> &stateTimestamps) {
    return this->feedDataBoreas(stateTimestamps);
  }

  StatusGraphConstruction LearningGP::feedDataBoreas(const vector<double> &stateTimestamps) {
    const auto &last_timestamp = stateTimestamps.back();
    RCLCPP_INFO(this->get_logger(), "Feeding Boreas Data");
    // next step we get all data from the database

    auto data_batch = data_boreas_->getDataBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator
    RCLCPP_INFO_STREAM(this->get_logger(), "got imu of size " << data_batch.imu.size());
    propagate_imu(data_batch.imu);
    RCLCPP_INFO_STREAM(this->get_logger(), "got gt of size " << data_batch.reference_pva.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "got gps of size " << data_batch.gnss.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "got lidar of size " << data_batch.lidar_raw.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "got image of size " << data_batch.image.size());
    auto gnss_integrator_base = graph_->getIntegrator("BoreasGNSSLCIntegrator");
    auto gnss_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator>&>(gnss_integrator_base);
    gnss_integrator->feedRAWData(data_batch.reference_pva, data_batch.reference_state);
    for (const auto &pva: data_batch.reference_pva)
      referenceBuffer_.update_buffer(pva, pva.timestamp);
    return graph_->constructFactorGraphOnTime(stateTimestamps, data_batch.imu);
  }

  StatusGraphConstruction LearningGP::feedDataDELoco(const vector<double> &stateTimestamps) {
    const auto &last_timestamp = stateTimestamps.back();

    // next step we get all data from the database
    auto data_batch = data_deloco_->getDataBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator

    propagate_imu(data_batch.imu);
    //auto gnss = data_deloco_->getGNSSObservationBefore(last_timestamp, true);
    auto gps_integrator_base = graph_->getIntegrator("IRTPVALCIntegrator");
    auto gps_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator> &>(gps_integrator_base);
    gps_integrator->feedRAWData(data_batch.reference_pva, data_batch.reference_state);
    for (const auto &pva: data_batch.reference_pva)
      referenceBuffer_.update_buffer(pva, pva.timestamp);
    return graph_->constructFactorGraphOnTime(stateTimestamps, data_batch.imu);
  }
}
