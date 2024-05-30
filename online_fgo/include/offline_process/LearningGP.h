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

//
// Created by haoming on 10.04.24.
//

#ifndef ONLINE_FGO_LEARNINGGP_H
#define ONLINE_FGO_LEARNINGGP_H

#pragma once

#include "OfflineFGOBase.h"
#include "dataset/DatasetBoreas.h"
#include "dataset/DatasetDELoco.h"

//third party

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
using gtsam::symbol_shorthand::N;  // integer ambiguities
using gtsam::symbol_shorthand::M;  // integer ddambiguities
using gtsam::symbol_shorthand::A;  // acceleration
using gtsam::symbol_shorthand::O;

namespace offline_process {
    using namespace fgo::integrator;
    using namespace fgo::graph;
    using namespace fgo::dataset;

    struct LearningGPParam
    {
        std::string dataset;
    };

    class LearningGP : public OfflineFGOBase {
    protected:
        //fgo::graph::GraphTimeCentric::Ptr graph_;
        fgo::data::State getPriorState() override;
        StatusGraphConstruction feedDataOffline(const std::vector<double>& stateTimestamps) override;

    public:
        //function
        explicit LearningGP(const rclcpp::NodeOptions &opt);
        ~LearningGP() override = default;

    private:

        std::unique_ptr<DatasetBoreas> data_boreas_;
        std::unique_ptr<DatasetDELoco> data_deloco_;
        StatusGraphConstruction feedDataBoreas(const std::vector<double>& stateTimestamps);
        StatusGraphConstruction feedDataDELoco(const std::vector<double>& stateTimestamps);

    };

}
#endif //ONLINE_FGO_LEARNINGGP_H
