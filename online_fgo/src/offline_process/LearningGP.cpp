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
    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: Starting the LearningGP Application ...");
    this->startOfflineProcess(dataset_->timestamp_start.seconds(), dataset_->timestamp_end.seconds());
    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: LearningGP Initialized ...");
  }

  fgo::data::State LearningGP::getPriorState() {
    return dataset_->prior_state;
  }

  StatusGraphConstruction LearningGP::feedDataOffline(const std::vector<double> &stateTimestamps) {
    return dataset_->feedDataToGraph(stateTimestamps);
  }

}
