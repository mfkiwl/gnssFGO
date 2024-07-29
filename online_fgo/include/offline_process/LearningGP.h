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
// Created by haoming on 10.04.24.
//

#ifndef ONLINE_FGO_LEARNINGGP_H
#define ONLINE_FGO_LEARNINGGP_H

#pragma once

#include "OfflineFGOBase.h"
#include "include/dataset/impl/DatasetBoreas.h"
#include "include/dataset/impl/DatasetDELoco.h"


namespace offline_process {
    using namespace fgo::integrator;
    using namespace fgo::graph;
    using namespace fgo::dataset;

    class LearningGP : public OfflineFGOBase {
    protected:
        //fgo::graph::GraphTimeCentric::Ptr graph_;
        fgo::data::State getPriorState() override;
        StatusGraphConstruction feedDataOffline(const std::vector<double>& stateTimestamps) override;
    public:
        //function
        explicit LearningGP(const rclcpp::NodeOptions &opt);
        ~LearningGP() override = default;
    };

}
#endif //ONLINE_FGO_LEARNINGGP_H
