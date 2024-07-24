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
// Created by haoming on 04.05.24.
//

#ifndef ONLINE_FGO_OFFLINEFGOBASE_H
#define ONLINE_FGO_OFFLINEFGOBASE_H
#pragma once

#include <condition_variable>
#include <mutex>
#include <csignal>
#include <keyboard_handler/keyboard_handler.hpp>

#include "gnss_fgo/GNSSFGOLocalizationBase.h"
//#include "utils/indicators/indicators.hpp"

namespace offline_process
{
  using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
  using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
  using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
  using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
  using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
  using gtsam::symbol_shorthand::N;  // integer ambiguities
  using gtsam::symbol_shorthand::M;  // integer ddambiguities
  using gtsam::symbol_shorthand::A;  // acceleration
  using gtsam::symbol_shorthand::O;

    using namespace fgo::integrator;
    using namespace fgo::graph;
    class OfflineFGOBase : public gnss_fgo::GNSSFGOLocalizationBase
    {
    public:
        using SignalHandlerType = void (*)(int);
        typedef std::vector<double> StateTimestamps_t;
        typedef std::vector<std::pair<size_t, double>> StateIDTimestampMap_t;
        typedef std::map<size_t, StateIDTimestampMap_t> OptStatesMap_t;
        explicit OfflineFGOBase(const std::string& name, const rclcpp::NodeOptions &opt);

        ~OfflineFGOBase()
        {
            for(const auto& cb_pair : keyboard_callbacks_map_)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: deleting callback function " << cb_pair.first);
                keyboard_handler_->delete_key_press_callback(cb_pair.second);
            }
            process_deferred_signal();
            uninstall_signal_handlers();
        }

    protected:

        static void signal_handler(int sig_num)
        {
            if (sig_num == SIGINT || sig_num == SIGTERM) {
                deferred_sig_number_ = sig_num;
            }
        }

        static void install_signal_handlers()
        {
            deferred_sig_number_ = -1;
            old_sigterm_handler_ = std::signal(SIGTERM, &OfflineFGOBase::signal_handler);
            old_sigint_handler_ = std::signal(SIGINT, &OfflineFGOBase::signal_handler);
        }

        static void uninstall_signal_handlers()
        {
            if (old_sigterm_handler_ != SIG_ERR) {
                std::signal(SIGTERM, old_sigterm_handler_);
                old_sigterm_handler_ = SIG_ERR;
            }
            if (old_sigint_handler_ != SIG_ERR) {
                std::signal(SIGINT, old_sigint_handler_);
                old_sigint_handler_ = SIG_ERR;
            }
            deferred_sig_number_ = -1;
        }

        static void process_deferred_signal()
        {
            auto call_signal_handler = [](const SignalHandlerType & signal_handler, int sig_num) {
                if (signal_handler != SIG_ERR && signal_handler != SIG_IGN && signal_handler != SIG_DFL) {
                    signal_handler(sig_num);
                }
            };

            if (deferred_sig_number_ == SIGINT) {
                call_signal_handler(old_sigint_handler_, deferred_sig_number_);
            } else if (deferred_sig_number_ == SIGTERM) {
                call_signal_handler(old_sigterm_handler_, deferred_sig_number_);
            }
        }

        void startOfflineProcess(const double &timestamp_start, const double &timestamp_stop)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: Starting the process thread...");
            optThread_ = std::make_unique<std::thread>([this](const double &timestamp_start, const double &timestamp_stop) -> void
                                                       {
                this->offlineTimeCentricFGO(timestamp_start, timestamp_stop);
                                                       }, timestamp_start, timestamp_stop);
        };

        /*** 18.04.2024
        *  this function contains a loop for OFFLINE time-centric graph construction and optimization
        *  that is used for an OFFLINE FGO process. The data are read from rosbag directly into the pipeline
        */
        void offlineTimeCentricFGO(const double &timestamp_start, const double &timestamp_stop);

        void processSingleEpoch(const StateIDTimestampMap_t &state_id_timestamps);

        virtual fgo::data::State getPriorState() = 0;

        virtual fgo::graph::StatusGraphConstruction feedDataOffline(const StateTimestamps_t &stateTimestamps) {};


    protected:
        enum ProcessOptimizationMode
        {
            LOOP,  // process optimization epochs in a loop continuously till the end of the iteration; Controlled by space to start/resume or pause
            STEP,  // process the optimization by one epoch; controlled with the right key; next step will only be excuted if the last optimization epoch is done
            BATCH, // process the optimization in a batch of epochs; the number of the epochs is given by the keyboard followed by enter
            BREAK,  // stop the optimization iteration loop by pressing a key
        };

        enum ProcessStatus
        {
            IDLE,    // optimization not started
            PAUSED,  // optimization paused
            RUNNING, // optimization still running, if STEP/BATCH mode, status will be changed after a full epoch
            STOPPED, // optimization done or terminated
        };

        void cb_kb_start_pause();
        void cb_kb_step();
        void cb_kb_batch(KeyboardHandler::KeyCode key_code);
        void cb_kb_stop();

        static StateTimestamps_t extract_state_timestamps_from_map(const StateIDTimestampMap_t &state_map)
        {
            StateTimestamps_t timestamps;
            for(const auto & id_timestamp_pair : state_map)
                timestamps.emplace_back(id_timestamp_pair.second);
            return timestamps;
        }

        void propagate_imu(const std::vector<fgo::data::IMUMeasurement>& imus);

    private:
        std::condition_variable opt_condition_;
        std::mutex opt_mut_;
        bool opt_condition_check_ = false;
        std::shared_ptr<KeyboardHandler> keyboard_handler_;
        std::map<std::string, KeyboardHandler::callback_handle_t> keyboard_callbacks_map_;
        OptStatesMap_t opt_states_map_;
        OptStatesMap_t::iterator opt_states_iter_;
        ProcessOptimizationMode opt_mode_ = ProcessOptimizationMode::LOOP;
        ProcessStatus opt_status_ = ProcessStatus::IDLE;
        std::atomic_uint batch_budget_ = 0;

    protected:
        static SignalHandlerType old_sigint_handler_;
        static SignalHandlerType old_sigterm_handler_;
        static int deferred_sig_number_;














    };



}
#endif //ONLINE_FGO_OFFLINEFGOBASE_H
