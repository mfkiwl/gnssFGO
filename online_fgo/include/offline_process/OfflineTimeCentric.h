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
// Created by haoming on 14.06.23.
//

#ifndef ONLINE_FGO_OFFLINETIMECENTRIC_H
#define ONLINE_FGO_OFFLINETIMECENTRIC_H


#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <irt_nav_msgs/msg/gnss_obs_pre_processed.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "include/datasets/BagReader.h"
#include "integrator/IntegratorBase.h"
#include "integrator/GNSSTCIntegrator.h"
#include "graph/GraphBase.h"
#include "factor/measurement/gnss/GPSFactor.h"
#include "factor/measurement/gnss/GPInterpolatedGPSFactor.h"
#include "factor/measurement/gnss/PVTFactor.h"
#include "factor/measurement/gnss/GPInterpolatedPVTFactor.h"
#include "utils/GNSSUtils.h"
#include "utils/ROSParameter.h"
#include "factor/measurement/gnss/PrFactor.h"
#include "graph/GraphTimeCentric.h"
#include "gnss_fgo/param/GNSSFGOParams.h"
#include "gnss_fgo/GNSSFGOLocalizationBase.h"

#include "include/datasets/DataSet.h"

//third party
#include "CalculateMeasurementDelay_ert_rtw/CalculateMeasurementDelay.h"
#include "InitGyroBias_ert_rtw/InitGyroBias.h"
#include "InitStatePVT_ert_rtw/InitStatePVT.h"

using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
using gtsam::symbol_shorthand::N;  // integer ambiguities
using gtsam::symbol_shorthand::M;  // integer ddambiguities
using gtsam::symbol_shorthand::A;  // acceleration
using gtsam::symbol_shorthand::O;

namespace offline_fgo {
    using namespace fgo::integrator;
    using namespace fgo::graph;
    class OfflineFGOTimeCentric : public gnss_fgo::GNSSFGOLocalizationBase{

    protected:
        std::shared_ptr<BagReader> reader_;
        fgo::graph::GraphTimeCentric::Ptr graph_;
        std::shared_ptr<GNSSTCIntegrator> Integrator_;
        //Preintegrated IMU Measurements
        std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> currentIMUPreintegrator_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams_;

        //data container
        fgo::data::CircularDataBuffer<fgo::data::State> fgoPredStateBuffer_;
        fgo::data::CircularDataBuffer<fgo::data::State> fgoOptStateBuffer_;
        std::vector<fgo::data::IMUMeasurement> imuDataContainer_; //collect 10 imu measurements
        //std::atomic_uint32_t imuCounter_ = 0;

        fgo::data::State lastOptimizedState_;
        fgo::data::State currentPredState_;
        std::map<size_t,double> statetimeContainer_;
        fgo::data::CircularDataBuffer<PVASolution> pvtDataBuffer_;

        //Ros variable
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStatePredPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStateOptPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr fgoStateExtrapolatedPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::Error2GT>::SharedPtr pvtErrorPub_;
        rclcpp::Publisher<irt_nav_msgs::msg::Error2GT>::SharedPtr pvtErrorFromPredPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pvtInterpolatedPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fgoStatePredNavFixPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fgoStateOptNavFixPub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pvtTestPub_;

        rclcpp::Publisher<irt_nav_msgs::msg::FactorResiduals>::SharedPtr pubRes_;
        // initialize parameter

        std::shared_ptr<std::thread> optThread_;
        std::shared_ptr<std::thread> initFGOThread_;
        std::atomic_bool lastInitFinished_{};
        std::atomic_bool isStateInited_{};

        //vector data
        std::vector<std::pair<rclcpp::Time, double>> referenceSensorTimestampContainer_;


        //tools
        std::unique_ptr<fgo::utils::MeasurementDelayCalculator> GNSSDelayCalculator_;

    public:
        //function

        explicit OfflineFGOTimeCentric(const rclcpp::NodeOptions& opt);
        ~OfflineFGOTimeCentric() override = default;

        bool initializeParameter();

    private:
        void doOfflineFGOProcess();
        void calculateErrorOnState(const fgo::data::State& state) override;

    };


}
#endif //ONLINE_FGO_OFFLINETIMECENTRIC_H
