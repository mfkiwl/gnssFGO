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
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
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
    };


    //auto data_param_deloco = std::make_shared<DatasetParam>(*this, "DELoco");
    //data_param_deloco->topic_type_map = {
    //        {"/irt_gnss_preprocessing/PVT", fgo::data::DataType::IRTPVAGeodetic},
    //        {"/imu/data", fgo::data::DataType::IMU},
    //        {"/irt_gnss_preprocessing/gnss_obs_preprocessed", fgo::data::DataType::IRTGNSSObsPreProcessed},
    //};


    auto gps_integrator_base = graph_->getIntegrator("BoreasLCIntegrator");
    auto gps_integrator_param = gps_integrator_base->getIntegratorBaseParamPtr();
    data_boreas_ = std::make_unique<DatasetBoreas>(*this, data_param_boreas, gps_integrator_param);
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
    auto imu = data_boreas_->getIMUBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator

    RCLCPP_INFO_STREAM(this->get_logger(), "got imu of size " << imu.size());
    propagate_imu(imu);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu propagated ");
    auto gt = data_boreas_->getReferenceStateBefore(last_timestamp, true);

    RCLCPP_INFO_STREAM(this->get_logger(), "got gt of size " << gt.size());
    auto gps = data_boreas_->getGNSSPVABefore(last_timestamp, true);

    RCLCPP_INFO_STREAM(this->get_logger(), "got gps of size " << gps.size());
    auto gt_pva = data_boreas_->getReferenceBefore(last_timestamp, true);

    RCLCPP_INFO_STREAM(this->get_logger(), "got gt pva of size " << gt_pva.size());

    auto gps_integrator_base = graph_->getIntegrator("BoreasLCIntegrator");
    auto gps_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator> &>(gps_integrator_base);
    gps_integrator->feedRAWData(gt_pva, gt);
    for (const auto &pva: gt_pva)
      referenceBuffer_.update_buffer(pva, pva.timestamp);
    return graph_->constructFactorGraphOnTime(stateTimestamps, imu);

  }

  StatusGraphConstruction LearningGP::feedDataDELoco(const vector<double> &stateTimestamps) {
    const auto &last_timestamp = stateTimestamps.back();

    // next step we get all data from the database
    auto imu = data_deloco_->getIMUBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator

    propagate_imu(imu);

    auto gt = data_deloco_->getReferenceStateBefore(last_timestamp, true);
    auto gt_pva = data_deloco_->getReferenceBefore(last_timestamp, true);
    //auto gnss = data_deloco_->getGNSSObservationBefore(last_timestamp, true);
    auto gps_integrator_base = graph_->getIntegrator("IRTPVALCIntegrator");
    auto gps_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator> &>(gps_integrator_base);
    gps_integrator->feedRAWData(gt_pva, gt);
    for (const auto &pva: gt_pva)
      referenceBuffer_.update_buffer(pva, pva.timestamp);
    return graph_->constructFactorGraphOnTime(stateTimestamps, imu);
  }
}
