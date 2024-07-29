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


#include "offline_process/OfflineFGOBase.h"

namespace offline_process {
  offline_process::OfflineFGOBase::SignalHandlerType offline_process::OfflineFGOBase::old_sigint_handler_{SIG_ERR};
  offline_process::OfflineFGOBase::SignalHandlerType offline_process::OfflineFGOBase::old_sigterm_handler_{SIG_ERR};
  int offline_process::OfflineFGOBase::deferred_sig_number_{-1};

  OfflineFGOBase::OfflineFGOBase(const std::string &name, const rclcpp::NodeOptions &opt)
    : gnss_fgo::GNSSFGOLocalizationBase(name, opt),
      datasetLoader_("online_fgo", "fgo::dataset::DatasetBase") {
    RCLCPP_INFO(this->get_logger(), "---------------------  OfflineFGOBase initializing! --------------------- ");

    if (!this->initializeCommon()) {
      RCLCPP_ERROR(this->get_logger(), "initializeParameters went wrong. Please check the parameters in config.");
      return;
    }
    install_signal_handlers();

    keyboard_handler_ = std::make_shared<KeyboardHandler>();

    keyboard_callbacks_map_.insert(std::make_pair("Start/Pause",
                                                  keyboard_handler_->add_key_press_callback(
                                                    [this](KeyboardHandler::KeyCode,
                                                           KeyboardHandler::KeyModifiers) { this->cb_kb_start_pause(); },
                                                    KeyboardHandler::KeyCode::SPACE)));

    keyboard_callbacks_map_.insert(std::make_pair("Step",
                                                  keyboard_handler_->add_key_press_callback(
                                                    [this](KeyboardHandler::KeyCode,
                                                           KeyboardHandler::KeyModifiers) { this->cb_kb_step(); },
                                                    KeyboardHandler::KeyCode::CURSOR_RIGHT)));

    keyboard_callbacks_map_.insert(std::make_pair("Batch5",
                                                  keyboard_handler_->add_key_press_callback(
                                                    [this](KeyboardHandler::KeyCode keycode,
                                                           KeyboardHandler::KeyModifiers) {
                                                      this->cb_kb_batch(keycode);
                                                    },
                                                    KeyboardHandler::KeyCode::NUMBER_5)));

    keyboard_callbacks_map_.insert(std::make_pair("Batch9",
                                                  keyboard_handler_->add_key_press_callback(
                                                    [this](KeyboardHandler::KeyCode keycode,
                                                           KeyboardHandler::KeyModifiers) {
                                                      this->cb_kb_batch(keycode);
                                                    },
                                                    KeyboardHandler::KeyCode::NUMBER_9)));

    keyboard_callbacks_map_.insert(std::make_pair("Stop",
                                                  keyboard_handler_->add_key_press_callback(
                                                    [this](KeyboardHandler::KeyCode,
                                                           KeyboardHandler::KeyModifiers) { this->cb_kb_stop(); },
                                                    KeyboardHandler::KeyCode::ESCAPE)));

    RCLCPP_INFO(this->get_logger(),
                "---------------------  OfflineFGOBase initializing Database... --------------------- ");
    utils::RosParameter<std::string> datasetName("Dataset.DatasetUsed", *this);
    RCLCPP_INFO_STREAM(get_logger(), "OfflineFGOBase Dataset used is: " << datasetName.value());
    if (datasetLoader_.isClassAvailable(datasetName.value())) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "OfflineFGOBase: initializing database: " << datasetName.value());
      dataset_ = datasetLoader_.createSharedInstance(datasetName.value());
      dataset_->initialize(datasetName.value(), *this, this->sensorCalibManager_);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "OfflineFGOBase: dataset: " << datasetName.value() << " initialized!");
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "OfflineFGOBase: cannot find the plugin of dataset: " << datasetName.value());
    }
    RCLCPP_INFO(this->get_logger(),
                "---------------------  OfflineFGOBase Database initialized! --------------------- ");

    RCLCPP_INFO(this->get_logger(), "---------------------  OfflineFGOBase initialized! --------------------- ");
  }

  OfflineFGOBase::~OfflineFGOBase() {
    for (const auto &cb_pair: keyboard_callbacks_map_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: deleting callback function " << cb_pair.first);
      keyboard_handler_->delete_key_press_callback(cb_pair.second);
    }
    process_deferred_signal();
    uninstall_signal_handlers();

  }

  void OfflineFGOBase::offlineTimeCentricFGO(const double &time_start, const double &time_end) {
    // in this loop we simulate the time-centric fgo process using the trimmed dataset
    // the dataset should be TRIMMED in advance so that the first data is aligned with the prior state

    static const auto time_offset_state = 1. / paramsPtr_->stateFrequency;
    static const auto time_offset_optimization = 1. / paramsPtr_->optFrequency;

    // we firstly initialize the graph using the prior_state queried while dataset setup

    lastOptimizedState_ = getPriorState();

    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: Factor Graph Initialization using the prior state at "
      << std::fixed << lastOptimizedState_.timestamp.seconds()
      << " s with state: " << lastOptimizedState_.state
      << " pose variances: " << lastOptimizedState_.poseVar.diagonal());

    gtsam::Vector3 gravity = gtsam::Vector3::Zero();
    gtsam::Vector3 gravity_b = gtsam::Vector3::Zero();
    if (paramsPtr_->calibGravity) {
      gravity = /*init_nedRe * */fgo::utils::gravity_ecef(lastOptimizedState_.state.position());
      gravity_b = lastOptimizedState_.state.pose().rotation().unrotate(gravity);
    }

    preIntegratorParams_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(gravity);
    preIntegratorParams_->accelerometerCovariance =
      pow(paramsPtr_->accelerometerSigma, 2) * gtsam::I_3x3; //Covariance of Sensor
    preIntegratorParams_->integrationCovariance = pow(paramsPtr_->integrationSigma, 2) * gtsam::I_3x3;
    preIntegratorParams_->gyroscopeCovariance = pow(paramsPtr_->gyroscopeSigma, 2) * gtsam::I_3x3;
    preIntegratorParams_->biasAccCovariance = pow(paramsPtr_->biasAccSigma, 2) * gtsam::I_3x3; //Covariance of Bias
    preIntegratorParams_->biasOmegaCovariance = pow(paramsPtr_->biasOmegaSigma, 2) * gtsam::I_3x3;
    preIntegratorParams_->biasAccOmegaInt = paramsPtr_->biasAccOmegaInt * gtsam::I_6x6;
    preIntegratorParams_->omegaCoriolis = gtsam::Vector3(0, 0, fgo::constants::earthRot); //Coriolis force from earth
    preIntegratorParams_->setUse2ndOrderCoriolis(false);
    currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                                          lastOptimizedState_.imuBias);

    const auto transReferenceFromBase = sensorCalibManager_->getTransformationFromBase("reference");
    graph_->initGraph(lastOptimizedState_, lastOptimizedState_.timestamp.seconds(), preIntegratorParams_);
    currentPredState_ = lastOptimizedState_;
    //add first state time
    fgoPredStateBuffer_.update_buffer(currentPredState_, currentPredState_.timestamp);
    fgoOptStateBuffer_.update_buffer(currentPredState_, currentPredState_.timestamp);
    //publish
    fgoStateOptPub_->publish(this->convertFGOStateToMsg(lastOptimizedState_));
    fgoStateOptNavFixPub_->publish(
      this->convertPositionToNavFixMsg(lastOptimizedState_.state, lastOptimizedState_.timestamp,
                                       transReferenceFromBase.translation()));

    // for an offline process, we firstly generate a state ids and timestamps map

    size_t state_num = (time_end - time_start) / time_offset_state;
    size_t opt_num = (time_end - time_start) / time_offset_optimization;
    size_t state_num_every_opt = paramsPtr_->optFrequency / paramsPtr_->stateFrequency;

    RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: Starting offline process with " << std::fixed << state_num
                                                                                        << " states in " << opt_num
                                                                                        << " optimization processes. Each process extends "
                                                                                        << state_num_every_opt
                                                                                        << " states. State time offset: "
                                                                                        << time_offset_state
                                                                                        << " min. optimization time in between: "
                                                                                        << time_offset_optimization
                                                                                        << ". Dataset duration "
                                                                                        << time_end - time_start
    );

    std::map<size_t, double> state_id_timestamp_map;
    // we define all state id timestamp map, where
    // * the keys are state ids starting from 1 (the initial state got 0)
    // * values are state timestamps
    for (size_t i = 1; i < state_num + 1; i++) {
      state_id_timestamp_map.insert(std::make_pair(i, time_start + i * time_offset_state));
    }
    // as the last measurement may be in between of two state variables, we compare the last state_id_timestamp
    const auto last_state_id_timestamp_pair = --state_id_timestamp_map.end();
    if (time_end < last_state_id_timestamp_pair->second)
      state_id_timestamp_map.insert(std::make_pair((--state_id_timestamp_map.end())->first + 1, time_start +
                                                                                                (last_state_id_timestamp_pair->first +
                                                                                                 1) *
                                                                                                time_offset_state));

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "OfflineFGO: State ID/Timestamp Map: totally " << state_id_timestamp_map.size()
                                                                      << " states. Time start: "
                                                                      << std::fixed << time_start
                                                                      << "s. Measurement time_end: "
                                                                      << time_end
                                                                      << "s. State time_end: "
                                                                      << last_state_id_timestamp_pair->second);

    // To control the FGO process, we then plan the optimization and states based on state_num and opt_num
    opt_states_map_.clear();
    auto state_iter = state_id_timestamp_map.begin();
    size_t opt_counter = 0;
    for (; opt_counter < opt_num; opt_counter++) {
      std::vector<std::pair<size_t, double>> state_id_timestamps;
      size_t state_budget = state_num_every_opt;
      do {
        state_id_timestamps.emplace_back(state_iter->first, state_iter->second);
        state_iter++;
        state_budget--;
      } while (state_budget > 0);
      opt_states_map_.insert(std::make_pair(opt_counter, state_id_timestamps));
    }

    // Trick: due to rounding, it could happen that the number of optimization epochs does not back up all states
    // if the state_iter didnot iterate to the end, we add a new optimization epoch
    StateIDTimestampMap_t state_id_timestamps;
    if (state_iter != state_id_timestamp_map.end()) {
      state_id_timestamps.emplace_back(std::make_pair(state_iter->first, state_iter->second));
      state_iter++;
    }
    opt_states_map_.insert(std::make_pair(++opt_counter, state_id_timestamps));

    RCLCPP_WARN_STREAM(this->get_logger(), "OfflineFGO: Optimization: totally " << opt_states_map_.size()
                                                                                << " epochs. \n Waiting for keyboard signal to start.... PRESS SPACE TO START...");
    this->isStateInited_ = true;
    //MAIN LOOP
    opt_states_iter_ = opt_states_map_.begin();
    while (opt_states_iter_ != opt_states_map_.end()) {
      // here we check the conditional mutex to get the loop started
      // THREE options are given:
      // O1: press space to start/resume OR pause the loop
      // O2: press left to run only one optimization epoch
      // O3: enter a number x to run x optimization epochs

      const auto dis_to_begin = std::distance(opt_states_map_.begin(), opt_states_iter_);
      const auto dis_to_end = std::distance(opt_states_iter_, opt_states_map_.end());

      std::unique_lock<std::mutex> lg(opt_mut_);
      opt_condition_.wait(lg, [this] { return opt_condition_check_; });

      if (opt_status_ == IDLE)
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "OfflineFGO: In optimization loop, waiting keyboard signals to DEFINE and START the optimization mode...");
      else if (opt_mode_ != ProcessOptimizationMode::LOOP)
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "OfflineFGO: In optimization loop, waiting keyboard signals to continue...");

      switch (opt_mode_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: processing in the loop");
        case ProcessOptimizationMode::LOOP:
        default: {
          opt_status_ = RUNNING;
          processSingleEpoch(opt_states_iter_->second);
          opt_states_iter_++;
          break;
        }
        case ProcessOptimizationMode::STEP: {
          RCLCPP_INFO_STREAM(this->get_logger(), "OfflineFGO: Starting ONE epoch ");
          opt_status_ = RUNNING;
          processSingleEpoch(opt_states_iter_->second);
          opt_states_iter_++;
          opt_status_ = PAUSED;
          opt_condition_check_ = false;
          break;
        }
        case ProcessOptimizationMode::BATCH: {

          if (batch_budget_) {
            opt_status_ = RUNNING;
            processSingleEpoch(opt_states_iter_->second);
            opt_states_iter_++;
            batch_budget_--;
          } else {
            opt_status_ = PAUSED;
            opt_condition_check_ = false;
          }
          break;
        }
        case ProcessOptimizationMode::BREAK: {
          opt_status_ = STOPPED;
          break;
        }
      }

      if (opt_status_ == STOPPED)
        break;
    }
  }

  void OfflineFGOBase::processSingleEpoch(const StateIDTimestampMap_t &state_id_timestamps) {
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    const auto start_fgo_construction = this->now();
    RCLCPP_INFO(this->get_logger(), "get timestamps...");
    const auto state_timestamps = extract_state_timestamps_from_map(state_id_timestamps);

    RCLCPP_INFO(this->get_logger(), "Start fgc...");
    const auto constructGraphStatus = feedDataOffline(state_timestamps);

    if (constructGraphStatus == fgo::graph::StatusGraphConstruction::FAILED) {
      throw std::invalid_argument("Factor Graph Construction failed");
    }

    double timeCFG = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::system_clock::now() - start).count();
    //lg.unlock();
    //start = std::chrono::system_clock::now();
    if (constructGraphStatus == fgo::graph::StatusGraphConstruction::SUCCESSFUL) {
      RCLCPP_INFO(this->get_logger(), "Start Optimization...");
      irt_nav_msgs::msg::ElapsedTimeFGO elapsedTimeFGO;
      elapsedTimeFGO.ts_start_construction = start_fgo_construction.seconds();
      elapsedTimeFGO.duration_construction = timeCFG;
      elapsedTimeFGO.ts_start_optimization = this->now().seconds();
      elapsedTimeFGO.num_new_factors = graph_->nrFactors();
      double timeOpt = this->optimize();
      isDoingPropagation_ = true;
      //double timeOpt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
      RCLCPP_INFO_STREAM(this->get_logger(), "Finished  optimization with a duration:" << timeOpt);
      elapsedTimeFGO.header.stamp = this->now();
      elapsedTimeFGO.duration_optimization = timeOpt;
      timerPub_->publish(elapsedTimeFGO);
    } else if (constructGraphStatus == fgo::graph::StatusGraphConstruction::NO_OPTIMIZATION) {
      isDoingPropagation_ = false;
    }
  }

  void OfflineFGOBase::propagateIMU(const std::vector<fgo::data::IMUMeasurement> &imus) {
    static const auto transReferenceFromBase = sensorCalibManager_->getTransformationFromBase("reference");

    gtsam::Vector3 gravity_b = gtsam::Vector3::Zero();
    if (paramsPtr_->calibGravity) {
      const auto gravity = fgo::utils::gravity_ecef(currentPredState_.state.position());
      gravity_b = currentPredState_.state.attitude().unrotate(gravity);
    } else {
      std::cout << "NOT CALCULATING GRAVITY" << std::endl;
    }

    for (const auto &imu_meas: imus) {
      //currentPredState_.mutex.lock();
      currentPredState_.timestamp = imu_meas.timestamp;
      currentPredState_.omega = currentPredState_.imuBias.correctGyroscope(imu_meas.gyro);
      currentPredState_.cbd = (gtsam::Matrix22() << 1, imu_meas.dt, 0, 1).finished() * currentPredState_.cbd;
      currentPredState_.accMeasured = (gtsam::Vector6()
        << imu_meas.accRot, currentPredState_.imuBias.correctAccelerometer(imu_meas.accLin + gravity_b)).finished();

      if (isDoingPropagation_) {
        //currentIMUPreIntMutex_.lock();
        //preintegrate till time of newest IMU meas| dt is for the future, but we dont know when the next data will arrive
        currentIMUPreintegrator_->integrateMeasurement(imu_meas.accLin,
                                                       imu_meas.gyro,
                                                       imu_meas.dt);
        //lastOptimizedState_.mutex.lock_shared();
        const auto new_state = currentIMUPreintegrator_->predict(lastOptimizedState_.state,
                                                                 lastOptimizedState_.imuBias);
        //lastOptimizedState_.mutex.unlock_shared();
        //currentIMUPreIntMutex_.unlock();
        // currentPredState_.timestamp += rclcpp::Duration::from_nanoseconds(fgoIMUMeasurement.dt * fgo::constants::sec2nanosec);
        currentPredState_.state = new_state;
      }

      //currentPredState_.mutex.unlock();
      graph_->updatePredictedBuffer(currentPredState_);
      fgoPredStateBuffer_.update_buffer(currentPredState_, imu_meas.timestamp, this->get_clock()->now());
      const auto navfixMsgRef = convertPositionToNavFixMsg(currentPredState_.state, currentPredState_.timestamp,
                                                           transReferenceFromBase.translation());
      const auto FGOStateMsg = this->convertFGOStateToMsg(currentPredState_);
      const std::array<double, 2> llh_rad = {navfixMsgRef.latitude * fgo::constants::deg2rad,
                                             navfixMsgRef.longitude * fgo::constants::deg2rad};
      fgo::data::UserEstimation_T userEstimation = {llh_rad[0], llh_rad[1],
                                                    navfixMsgRef.altitude, FGOStateMsg.cbd[0],
                                                    0., 0., 0.};
      fgoStatePredPub_->publish(FGOStateMsg);

      userEstimationBuffer_.update_buffer(userEstimation, imu_meas.timestamp);
      fgoStatePredNavFixPub_->publish(navfixMsgRef);

      if (this->isStateInited_ && !paramsPtr_->calcErrorOnOpt) {
        calculateErrorOnState(currentPredState_);
      }
    }
  }

  void OfflineFGOBase::cb_kb_start_pause() {
    if (opt_status_ == RUNNING && (opt_mode_ == LOOP || opt_mode_ == BATCH)) {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "KEYBOAD On Start->Pause: Optimization is running, pause will be issued after the current epoch.");
      opt_condition_check_ = false;
      opt_status_ = PAUSED;
    } else if (opt_status_ == IDLE || opt_status_ == PAUSED) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "KEYBOAD On Pause->Start: Optimization is on IDLE or paused, resuming the optimization loop ...");
      if (opt_mode_ == STEP || (opt_mode_ == BATCH && !batch_budget_)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "KEYBOAD On Pause->Start: Change the optimization mode to LOOP...");
        opt_mode_ = LOOP;
      }
      opt_status_ = RUNNING;
      opt_condition_check_ = true;
      opt_condition_.notify_one();
    } else // status == STOPPED
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "KEYBOAD On Start/Pause: Optimization is STOPPED, please start over ...");
    }
  }

  void OfflineFGOBase::cb_kb_step() {
    RCLCPP_INFO_STREAM(this->get_logger(), "KEYBOAD: On STEP pressed!");
    if (opt_status_ == PAUSED || opt_status_ == IDLE) {
      RCLCPP_INFO_STREAM(this->get_logger(), "KEYBOAD On STEP: executing the NEXT ONE optimization epoch... ");
      std::lock_guard<std::mutex> lg(opt_mut_);
      opt_mode_ = STEP;
      opt_condition_check_ = true;
      opt_condition_.notify_one();
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "KEYBOAD On STEP: optimization is not paused, please pause the optimization first using the space key");
    }
  }

  void OfflineFGOBase::cb_kb_batch(KeyboardHandler::KeyCode key_code) {
    RCLCPP_INFO_STREAM(this->get_logger(), "KEYBOAD: On Number Key pressed! Processing optimization epochs in a batch");

    if (opt_status_ == PAUSED || opt_status_ == IDLE) {
      // we only support 5 or 9 epochs in a batch
      std::lock_guard<std::mutex> lg(opt_mut_);
      opt_mode_ = BATCH;
      if (key_code == KeyboardHandler::KeyCode::NUMBER_5) {
        batch_budget_ = 5;
      } else {
        batch_budget_ = 9;
      }
      opt_condition_check_ = true;
      opt_condition_.notify_one();
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "KEYBOAD On Batch: optimization is not paused, please pause the optimization first using the space key or wait until paused");
    }
  }

  void OfflineFGOBase::cb_kb_stop() {
    RCLCPP_WARN_STREAM(this->get_logger(), "KEYBOAD: On EXIT! ");
    opt_mode_ = BREAK;
    std::lock_guard<std::mutex> lg(opt_mut_);
    if (opt_status_ == RUNNING)
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "KEYBOAD On EXIT: The optimization is running... Breaking after the current epoch... ");
    else {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "KEYBOAD On EXIT: The optimization is on IDLE or PAUSED... Breaking now... ");
      opt_condition_check_ = true;
      opt_condition_.notify_one();
    }
  }


}
