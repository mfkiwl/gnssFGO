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
//          Jiandong Chen (jiandong.chen@rwth-aachen.de)
//

#ifndef ONLINE_FGO_OFFLINEVISUALFGO_H
#define ONLINE_FGO_OFFLINEVISUALFGO_H
#pragma once

#include "OfflineFGOBase.h"
#include "include/dataset/impl/DatasetPohang.h"
#include "include/dataset/impl/DatasetKitti.h"

namespace offline_process {
  using namespace fgo::integrator;
  using namespace fgo::graph;
  using namespace fgo::dataset;

  class OfflineVisualFGO : public OfflineFGOBase {
  public:
    explicit OfflineVisualFGO(const rclcpp::NodeOptions &opt);
    ~OfflineVisualFGO() override = default;

  protected:
    fgo::data::State getPriorState() override;
    StatusGraphConstruction feedDataOffline(const std::vector<double> &stateTimestamps) override;

  };
}


#endif //ONLINE_FGO_OFFLINEVISUALFGO_H
