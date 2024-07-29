//  Copyright 2024 Institute of Automatic Control RWTH Aachen University
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
    this->startOfflineProcess(dataset_->timestamp_start.seconds(), dataset_->timestamp_end.seconds());
    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: OfflineVisualFGO Initialized ...");
  }

  StatusGraphConstruction OfflineVisualFGO::feedDataOffline(const vector<double> &stateTimestamps) {
    return dataset_->feedDataToGraph(stateTimestamps);
  }

  fgo::data::State OfflineVisualFGO::getPriorState() {
    return dataset_->prior_state;
  }
}
