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


// STD
#include <iostream>
#include "offline_process/OfflineTimeCentric.h"

namespace offline_fgo {

  OfflineFGOTimeCentric::OfflineFGOTimeCentric(const rclcpp::NodeOptions &opt) : gnss_fgo::GNSSFGOLocalizationBase(
    "OfflineFGOTimeCentricNode", opt) {
    //Load databags
    RCLCPP_INFO(this->get_logger(),
                "---------------------  OfflineFGOTimeCentricNode initializing! --------------------- ");
    utils::RosParameter<std::string> bag_path("bagfile_path", "/mnt/DataBig/fgo/AC_with_GNSS/AC_with_GNSS_0.db3",
                                              *this);
    utils::RosParameter<std::vector<std::string>> topic_filter("OfflineFGO.topic_filter",
                                                               {"/imu/data",
                                                                "/irt_gnss_preprocessing/gnss_obs_preprocessed",
                                                                "/irt_gnss_preprocessing/PVT"}, *this);
    RCLCPP_INFO_STREAM(this->get_logger(), "Data Container initializing");
    reader_ = std::make_unique<BagReader>(*this, bag_path.value(), topic_filter.value());
    Integrator_ = std::make_shared<GNSSTCIntegrator>();
    reader_->readDataFromBag();
    RCLCPP_INFO_STREAM(this->get_logger(), "Data Container initialized");
    paramsPtr_ = std::make_shared<gnss_fgo::GNSSFGOParams>();

    if (!initializeParameter()) {
      RCLCPP_ERROR(this->get_logger(), "initializeParameters went wrong. Please check the parameters in config.");
      return;
    }

    graph_ = std::make_shared<fgo::graph::GraphTimeCentric>(*this, paramsPtr_);
    graph_->registerIntegrator("GNSSTCIntegrator", Integrator_);
    Integrator_->initialize(*this, *graph_, "GNSSTCIntegrator");
    //
    optThread_ = std::make_unique<std::thread>([this]() -> void {
      this->doOfflineFGOProcess();
    });
    RCLCPP_INFO(this->get_logger(),
                "---------------------  OfflineFGOTimeCentricNode initialized! --------------------- ");
  }


  bool OfflineFGOTimeCentric::initializeParameter() {
    //load Ros Parameter

    if (!this->initializeCommonParameter())
      return false;

    //Using Gaussian Prior and Gaussian motion prior to interpolate pose.
    std::cout << "check status of useGPInterpolatedFactor: " << paramsPtr_->useGPInterpolatedFactor << std::endl;
    std::cout << "check status of useGPPriorFactor: " << paramsPtr_->useGPPriorFactor << std::endl;
    utils::RosParameter<bool> pub_pose_vel("OnlineFGO.Publish.pubPoseVel", true, *this);
    if (pub_pose_vel.value()) {
      posePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("onlineFGO/pose",
                                                                                       rclcpp::SystemDefaultsQoS());

      velPub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("onlineFGO/vel",
                                                                                       rclcpp::SystemDefaultsQoS());
    }
    utils::RosParameter<bool> pub_fgo_state("OnlineFGO.Publish.pubFGOState", true, *this);
    if (pub_fgo_state.value()) {
      fgoStatePredPub_ = this->create_publisher<irt_msgs::msg::FGOState>("statePredicted",
                                                                         rclcpp::SystemDefaultsQoS());

      fgoStateOptPub_ = this->create_publisher<irt_msgs::msg::FGOState>("stateOptimized",
                                                                        rclcpp::SystemDefaultsQoS());
      fgoStateExtrapolatedPub_ = this->create_publisher<irt_msgs::msg::FGOState>("stateExtrapolated",
                                                                                 rclcpp::SystemDefaultsQoS());
      fgoStatePredNavFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("statePredictedNavFix",
                                                                                   rclcpp::SystemDefaultsQoS());
      fgoStateOptNavFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("offline_stateOptmizedNavFix",
                                                                                  rclcpp::SystemDefaultsQoS());
    }

    pvtInterpolatedPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("PVTInterpolated",
                                                                              rclcpp::SystemDefaultsQoS());
    pvtTestPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("PVT",
                                                                      rclcpp::SystemDefaultsQoS());
    pvtErrorFromPredPub_ = this->create_publisher<irt_msgs::msg::ERROR2GT>("errorPred",
                                                                           rclcpp::SystemDefaultsQoS());
    utils::RosParameter<bool> pub_pvt_error("OnlineFGO.Publish.pubPVTError", true, *this);
    if (pub_pvt_error.value()) {
      pvtErrorPub_ = this->create_publisher<irt_msgs::msg::ERROR2GT>(
        "onlineFGO/error",
        rclcpp::SystemDefaultsQoS());
    }
    //resize data
    pvtInterpolatedPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("PVTInterpolated",
                                                                              rclcpp::SystemDefaultsQoS());
    pvtDataBuffer_.resize_buffer(50000);

    return true;
  }


  void OfflineFGOTimeCentric::doOfflineFGOProcess() {
    RCLCPP_INFO(this->get_logger(), "--------------------- doOfflineFGOProcess! --------------------- ");

    GNSSMeasurement first_gnss = reader_->getNextGNSSMeasurement();

    //store in fg graph?
    Integrator_->storeGNSSMessage(first_gnss);

    //get imu corresponding to first_GNSS
    const rclcpp::Time &initial_time = first_gnss.measMainAnt.timestamp;
    auto first_imu = reader_->getNextIMUMeasurement();
    while (first_imu.timestamp.seconds() < initial_time.seconds()) {
      first_imu = reader_->getNextIMUMeasurement();
    }

    // After we got first GNSS, we need to find a PVA measurement which can be associated with the GNSS using tow
    auto first_pva = reader_->getNextPVASolution();
    while (first_pva.tow != first_gnss.measMainAnt.tow)
      first_pva = reader_->getNextPVASolution();
    std::cout << "first_gnss.measMainAnt.tow: " << first_gnss.measMainAnt.tow << std::endl;
    if (first_pva.tow != first_gnss.measMainAnt.tow)
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Can't get PVA associated with the first GNSS obs. at " << std::fixed
                                                                                  << first_gnss.measMainAnt.tow);
    pvtDataBuffer_.update_buffer(first_pva, first_pva.timestamp);
    // now we can initialize the FGO with PVA solution
    /**
     * Init FGO
     */

    imuDataContainer_.push_back(first_imu);

    // initial lastOptimizedState with PVA and imu data
    gtsam::Rot3 init_nRb = gtsam::Rot3::Yaw(first_pva.heading);  //roll_pitch=?
    gtsam::Rot3 init_nedRe(fgo::utils::nedRe_Matrix_asLLH(first_pva.llh));  //fgo paper eq 4
    gtsam::Rot3 init_eRb = init_nedRe.inverse() * init_nRb;
    lastOptimizedState_.timestamp = first_imu.timestamp;
    lastOptimizedState_.imuBias = gtsam::imuBias::ConstantBias((gtsam::Vector3() << 0., 0., 0.).finished(),
                                                               (gtsam::Vector3() << 0., 0., 0.).finished());

    // clock bias and clock drift
    lastOptimizedState_.cbd = (gtsam::Vector2() << first_pva.clock_bias, first_pva.clock_drift).finished();

    //
    lastOptimizedState_.omega = first_imu.gyro;

    //initial position of the antenna in Earth-Centered Earth-Fixed (ECEF)
    auto init_ant_pos = fgo::utils::llh2xyz(
      (gtsam::Point3() << first_pva.llh[0], first_pva.llh[1], first_pva.llh[2]).finished());

    //initial position of the IMU in ECEF coordinates
    auto init_imu_pos = init_ant_pos - init_eRb.rotate(paramsPtr_->transIMUToAnt1);
    auto vecGrav = fgo::utils::gravity_ecef(init_imu_pos);
    auto gravity = fgo::utils::gravity_ecef(first_pva.xyz_ecef);
    auto gravity_b = init_eRb.unrotate(vecGrav);
    lastOptimizedState_.accMeasured = (gtsam::Vector6() << first_imu.accRot, first_imu.accLin +
                                                                             gravity_b).finished();
    lastOptimizedState_.state = gtsam::NavState(init_eRb, init_imu_pos,
                                                init_nedRe.inverse() * (gtsam::Velocity3() <<
                                                                                           first_pva.vel_n[0], first_pva.vel_n[1], first_pva.vel_n[2]).finished());
    //set up imu_preIntegrator
    preIntegratorParams_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(vecGrav);
    //std::cout << std::fixed << "vecGrav: " << vecGrav << std::endl;
    //std::cout << std::fixed << "vecGravBody: " << gravity_b << std::endl;
    //imu_params->setBodyPSensor() possible
    preIntegratorParams_->accelerometerCovariance =
      pow(paramsPtr_->accelerometerSigma, 2) * gtsam::I_3x3; //Covariance of Sensor
    preIntegratorParams_->integrationCovariance = pow(paramsPtr_->integrationSigma, 2) * gtsam::I_3x3;
    preIntegratorParams_->gyroscopeCovariance = pow(paramsPtr_->gyroscopeSigma, 2) * gtsam::I_3x3;
    preIntegratorParams_->biasAccCovariance = pow(paramsPtr_->biasAccSigma, 2) * gtsam::I_3x3; //Covariance of Bias
    preIntegratorParams_->biasOmegaCovariance = pow(paramsPtr_->biasOmegaSigma, 2) * gtsam::I_3x3;
    preIntegratorParams_->biasAccOmegaInt = paramsPtr_->biasAccOmegaInt * gtsam::I_6x6;
    preIntegratorParams_->omegaCoriolis = gtsam::Vector3(0, 0, fgo::constants::earthRot); //Coriolis force from earth
    preIntegratorParams_->setUse2ndOrderCoriolis(true);
    currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                                          lastOptimizedState_.imuBias);

    graph_->initGraph(lastOptimizedState_, initial_time.seconds(), preIntegratorParams_);

    lastOptimizedState_.poseVar.block<3, 3>(3, 3) =
      init_nedRe.transpose() * lastOptimizedState_.poseVar.block<3, 3>(3, 3) * init_nedRe.inverse().matrix();

    currentPredState_ = lastOptimizedState_;
    //add first state time
    statetimeContainer_.insert({0, first_imu.timestamp.seconds()});
    fgoPredStateBuffer_.update_buffer(currentPredState_, initial_time);
    fgoOptStateBuffer_.update_buffer(currentPredState_, initial_time);
    //publish
    fgoStateOptPub_->publish(this->convertFGOStateToMsg(lastOptimizedState_));
    fgoStateOptNavFixPub_->publish(
      this->convertPositionToNavFixMsg(lastOptimizedState_.state, lastOptimizedState_.timestamp, true));


    /**
     * Main Loop
     */
    //set up graph
    std::cout << "--------------main loop-------------" << std::endl;
    /* irt_msgs::msg::FactorResiduals resMsg;
     irt_msgs::msg::FactorResidual res;
     res.name = "sdfklsd";
     resMsg.residuals.resize(10);
     resMsg.residuals.emplace_back(res);
     pubRes_->publish(resMsg);
     std::cout << "complete publish res in offline" << std::endl;*/
    static uint notifyCounter = paramsPtr_->IMUMeasurementFrequency / paramsPtr_->optFrequency;

    while (reader_->hasNextIMUMeasurement()) {
      const auto imuMeasurement = reader_->getNextIMUMeasurement();
      imuDataContainer_.push_back(imuMeasurement);


      currentIMUPreintegrator_->integrateMeasurement(imuMeasurement.accLin,
                                                     imuMeasurement.gyro,
                                                     imuMeasurement.dt);
      auto prop_state = currentIMUPreintegrator_->predict(lastOptimizedState_.state, lastOptimizedState_.imuBias);

      currentPredState_.state = prop_state;
      currentPredState_.timestamp = imuMeasurement.timestamp;
      currentPredState_.omega = currentPredState_.imuBias.correctGyroscope(imuMeasurement.gyro);


      gravity = fgo::utils::gravity_ecef(currentPredState_.state.position());
      gravity_b = currentPredState_.state.attitude().unrotate(gravity);


      currentPredState_.accMeasured = (gtsam::Vector6() << imuMeasurement.accRot, currentPredState_.imuBias.
        correctAccelerometer(imuMeasurement.accLin + gravity_b)).finished();
      graph_->updatePredictedBuffer(currentPredState_); // update to currentPredictedBuffer_


      auto FGOStateMsg = this->convertFGOStateToMsg(currentPredState_);
      fgoStatePredPub_->publish(FGOStateMsg);
      fgoStatePredNavFixPub_->publish(convertPositionToNavFixMsg(FGOStateMsg, true));

      //calculateErrorOnState with frequency 100hz
      calculateErrorOnState(currentPredState_);

      if (!imuDataContainer_.empty() && imuDataContainer_.size() % notifyCounter == 0) {
        std::cout << "---------------10th imu, add gnss factor-----------" << std::endl;

        // store new pva data and the 10th imu data
        //？ pvt and gnss seems not added to the factor graph? only imu measurements?
        std::cout << "lastOptimizedState_.timestamp = " << lastOptimizedState_.timestamp.seconds()
                  << std::endl;
        std::cout << "10 th imuMeasurement.timestamp = " << imuMeasurement.timestamp.seconds() << std::endl;
        auto pvaMeasurement = reader_->getPVAMeasurementsBetween(lastOptimizedState_.timestamp,
                                                                 imuMeasurement.timestamp);
        if (!pvaMeasurement.empty()) {
          for (const auto &pva: pvaMeasurement) {

            pvtDataBuffer_.update_buffer(pva, pva.timestamp);
            std::cout << "find " << pvaMeasurement.size() << " pva measurement at "
                      << pva.timestamp.seconds() << std::endl;
          }
        } else {
          std::cout << "No pva measurement between time period" << std::endl;
        }
        //find all gnss between this time period
        auto gnssMeasurement = reader_->getGNSSMeasurementsBetween(lastOptimizedState_.timestamp,
                                                                   imuMeasurement.timestamp);
        if (!gnssMeasurement.empty()) {
          for (GNSSMeasurement gnss: gnssMeasurement) {
            Integrator_->storeGNSSMessage(gnss);
            std::cout << "find " << gnssMeasurement.size() << " gnss measurement at "
                      << gnss.measMainAnt.timestamp.seconds() << std::endl;
          }
        } else {
          std::cout << "No gnss measurement between time period" << std::endl;

        }
        if (gnssMeasurement.empty() && pvaMeasurement.empty()) {
          continue;
        }

        //integrator_map
        auto constructGraphStatus = graph_->constructFactorGraph(imuDataContainer_);
        if (constructGraphStatus == fgo::graph::ConstructFGStatus::FAILED) {
          throw std::invalid_argument("FGC failed");
        } else if (constructGraphStatus == fgo::graph::ConstructFGStatus::SUCCESSFUL) {
          std::cout << "---------------start optimization-----------" << std::endl;
          fgo::data::State newOptState;

          //??
          newOptState.ddIntAmb = currentPredState_.ddIntAmb;
          newOptState.ddIntAmbVar = currentPredState_.ddIntAmbVar;

          double optTime = graph_->optimize(newOptState);

          RCLCPP_INFO_STREAM(this->get_logger(), "Finished  optimization with a duration:" << optTime);
          std::cout << "newOptState.timestamp " << newOptState.timestamp.seconds() << std::endl;
          auto fgoStateMsg = this->convertFGOStateToMsg(newOptState);
          fgoStateOptPub_->publish(fgoStateMsg);
          fgoStateOptNavFixPub_->publish(convertPositionToNavFixMsg(fgoStateMsg, true));

          //? gravity?
          preIntegratorParams_->n_gravity = fgo::utils::gravity_ecef(newOptState.state.position());

          currentIMUPreintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
            preIntegratorParams_, newOptState.imuBias);
          currentPredState_ = newOptState;
          lastOptimizedState_ = newOptState;
          imuDataContainer_.clear();
          calculateErrorOnState(newOptState);
        }
      }
    }


  }


//？

  void OfflineFGOTimeCentric::calculateErrorOnState(const fgo::data::State &stateIn) {

    static const auto l_b_ant_main = paramsPtr_->transIMUToAnt1;
    static const auto l_b_ant_aux = paramsPtr_->transIMUToAnt2;
    static std::vector<fgo::data::State> stateCached;


    //auto labelBuffer = gnssLabelingMsgBuffer_.get_all_buffer();
    auto pvtBuffer = pvtDataBuffer_.get_all_buffer();
    auto lastPVTTime = pvtBuffer.back().timestamp.seconds();
    stateCached.emplace_back(stateIn);

    auto stateIter = stateCached.begin();
    while (stateIter != stateCached.end()) {
      double stateTime = stateIter->timestamp.seconds();
      auto stateTimeNanoSec = stateIter->timestamp.nanoseconds();
      if (stateTime < lastPVTTime) {
        // data is sorted
        auto itAfter = std::lower_bound(pvtBuffer.begin(), pvtBuffer.end(), stateTime,
                                        [](const PVASolution &pvt, double timestamp) -> bool {
                                          // true, ... true, true, false(HERE), false, ... false
                                          return pvt.timestamp.seconds() < timestamp;
                                        });

        auto itBefore = itAfter - 1;

        const auto coeff = (stateTime - itBefore->timestamp.seconds()) /
                           (itAfter->timestamp.seconds() - itBefore->timestamp.seconds());
        const double tiffDiff = stateTime - itBefore->timestamp.seconds();


        //const auto pvt_phi = gtsam::interpolate(itBefore->phi, itAfter->phi, coeff);
        const auto pvt_phi_std = gtsam::interpolate(itBefore->llh_var[0], itAfter->llh_var[0], coeff);
        //const auto pvt_lambda = gtsam::interpolate(itBefore->lambda, itAfter->lambda, coeff);
        const auto pvt_lambda_std = gtsam::interpolate(itBefore->llh_var[1], itAfter->llh_var[1], coeff);
        //const auto pvt_h = gtsam::interpolate(itBefore->h, itAfter->h, coeff);
        const auto pvt_h_std = gtsam::interpolate(itBefore->llh_var[2], itAfter->llh_var[2], coeff);
        const auto pvt_Yaw = gtsam::interpolate(itBefore->yaw, itAfter->yaw, coeff);
        const auto pvt_Yaw_std = gtsam::interpolate(itBefore->yaw_var, itAfter->yaw_var, coeff);
        //const auto pvt_COG = gtsam::interpolate(itBefore->COG, itAfter->COG, coeff);
        const auto pvt_tow = gtsam::interpolate(itBefore->tow, itAfter->tow, coeff);
        const auto pvt_RxClkBias = gtsam::interpolate(itBefore->clock_bias, itAfter->clock_bias, coeff);
        const auto pvt_RxClkDrift = gtsam::interpolate(itBefore->clock_drift, itAfter->clock_drift, coeff);
        const auto pvt_Vn = gtsam::interpolate(itBefore->vel_n[0], itAfter->vel_n[0], coeff);
        const auto pvt_Ve = gtsam::interpolate(itBefore->vel_n[1], itAfter->vel_n[1], coeff);
        const auto pvt_Vu = gtsam::interpolate(-itBefore->vel_n[2], -itAfter->vel_n[2], coeff);

        const auto ref_pos_llh = fgo::utils::WGS84InterpolationLLH(
          gtsam::Point3(itBefore->llh[0], itBefore->llh[1], itBefore->llh[2]),
          gtsam::Point3(itAfter->llh[0], itAfter->llh[1], itAfter->llh[2]),
          coeff);

        sensor_msgs::msg::NavSatFix pvtMsg;
        pvtMsg.header.stamp = stateIter->timestamp;
        pvtMsg.altitude = ref_pos_llh.z();
        pvtMsg.latitude = ref_pos_llh.x() * fgo::constants::rad2deg;
        pvtMsg.longitude = ref_pos_llh.y() * fgo::constants::rad2deg;

        pvtInterpolatedPub_->publish(pvtMsg);

        // calculate residue error

        irt_msgs::msg::ERROR2GT error2Gt;

        const gtsam::Vector2 ref_cbd(pvt_RxClkBias, pvt_RxClkDrift);
        const gtsam::Vector3 ref_pos_llh_std(pvt_phi_std, pvt_lambda_std, pvt_h_std);

        const auto &fgo_pos_ecef = stateIter->state.position();
        const auto &fgo_ori_ecef = stateIter->state.attitude();
        const auto &fgo_omega = stateIter->omega;
        const auto &fgo_bias = stateIter->imuBias;
        const auto &fgo_cbd = stateIter->cbd;
        const auto &fgo_vel_ecef = stateIter->state.v();
        const auto fgo_pose_ecef_std = stateIter->poseVar.diagonal().cwiseSqrt();
        const auto fgo_vel_ecef_std = stateIter->velVar.diagonal().cwiseSqrt();
        const auto fgo_omega_std = stateIter->omegaVar.diagonal().cwiseSqrt();
        const auto fgo_cbd_std = stateIter->cbdVar.diagonal().cwiseSqrt();
        const auto fgo_bias_std = stateIter->imuBiasVar.diagonal().cwiseSqrt();

        const auto pos_ant_main_ecef = fgo_pos_ecef + fgo_ori_ecef.rotate(l_b_ant_main);
        const auto pos_ant_llh = fgo::utils::xyz2llh(pos_ant_main_ecef);
        const gtsam::Rot3 nRe(fgo::utils::nedRe_Matrix(pos_ant_main_ecef));
        const auto fgo_pose_std_ned = (gtsam::Vector6()
          << nRe.rotate(fgo_pose_ecef_std.block<3, 1>(0, 0)), nRe.rotate(
          fgo_pose_ecef_std.block<3, 1>(3, 0))).finished();
        const auto fgo_vel_ned = nRe.rotate(
          fgo_vel_ecef + fgo_ori_ecef.rotate(gtsam::skewSymmetric((-l_b_ant_main)) * fgo_omega));
        const auto fgo_vel_ned_std = nRe.rotate(fgo_vel_ecef_std);
        const auto nRb_rpy = fgo::utils::func_DCM2EulerAngles((nRe.compose(fgo_ori_ecef)).matrix());

        auto fgo_yaw = nRb_rpy(2) * 180. / M_PI;
        fgo_yaw = (fgo_yaw >= 0. ? fgo_yaw : (fgo_yaw + 360.));
        const auto fgo_pitch = nRb_rpy(1) * 180. / M_PI;
        const auto fgo_roll = nRb_rpy(0) * 180. / M_PI;

        const auto ref_pos_ecef = fgo::utils::llh2xyz(ref_pos_llh);
        const gtsam::Rot3 nReGT(fgo::utils::nedRe_Matrix_asLLH(ref_pos_llh));
        const gtsam::Vector3 ref_vel_ned(pvt_Vn, pvt_Ve, -pvt_Vu);

        fgo::utils::eigenMatrix2stdVector(ref_pos_llh, error2Gt.ref_llh);
        fgo::utils::eigenMatrix2stdVector(ref_pos_llh_std, error2Gt.ref_llh_std);
        fgo::utils::eigenMatrix2stdVector(fgo_pose_ecef_std, error2Gt.pose_std_ecef);
        fgo::utils::eigenMatrix2stdVector(fgo_pose_std_ned, error2Gt.pose_std_ned);
        fgo::utils::eigenMatrix2stdVector(fgo_vel_ecef_std, error2Gt.vel_std_ecef);
        fgo::utils::eigenMatrix2stdVector(fgo_vel_ned_std, error2Gt.vel_std_ned);
        fgo::utils::eigenMatrix2stdVector(fgo_cbd, error2Gt.cbd);
        fgo::utils::eigenMatrix2stdVector(fgo_cbd_std, error2Gt.cbd_std);
        fgo::utils::eigenMatrix2stdVector(ref_cbd, error2Gt.ref_cbd);
        fgo::utils::eigenMatrix2stdVector(fgo_vel_ned, error2Gt.vel_ned);
        fgo::utils::eigenMatrix2stdVector(ref_vel_ned, error2Gt.ref_vel);
        fgo::utils::eigenMatrix2stdVector(fgo_bias.accelerometer(), error2Gt.acc_bias);
        fgo::utils::eigenMatrix2stdVector(fgo_bias.gyroscope(), error2Gt.gyro_bias);
        fgo::utils::eigenMatrix2stdVector(fgo_bias_std.head(3), error2Gt.acc_bias_std);
        fgo::utils::eigenMatrix2stdVector(fgo_bias_std.tail(3), error2Gt.gyro_bias_std);
        fgo::utils::eigenMatrix2stdVector(fgo_omega, error2Gt.omega_body);
        fgo::utils::eigenMatrix2stdVector(fgo_omega_std, error2Gt.omega_body_std);

        if (itAfter->type == fgo::data::GNSSPVTSolutionType::RTKFIX) {
          const auto pos_diff_ecef = pos_ant_main_ecef - ref_pos_ecef;
          const auto pos_error_ned = nRe.rotate(pos_diff_ecef);//
          const auto pos_error_body = fgo_ori_ecef.unrotate(pos_diff_ecef);
          const auto vel_error_ned = fgo_vel_ned - ref_vel_ned;

          const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
          double dist;
          geod.Inverse(ref_pos_llh.x() * fgo::constants::rad2deg,
                       ref_pos_llh.y() * fgo::constants::rad2deg,
                       pos_ant_llh.x() * fgo::constants::rad2deg,
                       pos_ant_llh.y() * fgo::constants::rad2deg, dist);

          error2Gt.pos_2d_error_geographic = dist;
          std::cout << "error2Gt.pos_2d_error_geographic is : " << dist << std::endl;
          error2Gt.pos_3d_error_geographic = std::sqrt(
            dist * dist + std::pow((ref_pos_llh.z() - pos_ant_llh.z()), 2));
          error2Gt.pos_1d_error_ned = abs(pos_error_ned(1));//fgo_ori.unrotate(pos_error_ecef)(1)
          error2Gt.pos_2d_error_ned = (pos_error_ned).block<2, 1>(0, 0).norm();
          error2Gt.pos_3d_error_ned = pos_error_ned.norm();
          error2Gt.pos_1d_error_body = abs(pos_error_body(1));
          error2Gt.pos_2d_error_body = (pos_error_body).block<2, 1>(0, 0).norm();
          error2Gt.pos_3d_error_body = pos_error_body.norm();
          error2Gt.pos_2d_error_ecef = (pos_diff_ecef).block<2, 1>(0, 0).norm();
          error2Gt.pos_3d_error_ecef = pos_diff_ecef.norm();
          error2Gt.vel_2d_error = vel_error_ned.block<2, 1>(0, 0).norm();
          error2Gt.vel_3d_error = vel_error_ned.norm();
          if ((pvt_Yaw == 270. && pvt_Yaw == 0.) || pvt_Yaw_std > 30.)
            error2Gt.yaw_error = std::numeric_limits<double>::max();
          else {
            double yaw_error = fgo_yaw - pvt_Yaw;
            while (yaw_error > 180.0 || yaw_error < -180.0) {
              if (yaw_error > 180.0)
                yaw_error -= 360.0;
              if (yaw_error < -180.0)
                yaw_error += 360.0;
            }
            yaw_error = abs(yaw_error);
            error2Gt.yaw_error = yaw_error;
          }
        }

        error2Gt.tow = pvt_tow;
        error2Gt.ref_tow_before = itBefore->tow;
        error2Gt.ref_tow_after = itAfter->tow;
        error2Gt.ref_error = itAfter->error;
        error2Gt.ref_mode = itAfter->type;
        error2Gt.yaw = fgo_yaw;
        error2Gt.ref_yaw = pvt_Yaw;
        error2Gt.ref_yaw_std = pvt_Yaw_std;
        error2Gt.pitch = fgo_pitch;
        error2Gt.roll = fgo_roll;
        error2Gt.ref_pitch_roll = itAfter->roll_pitch;
        error2Gt.ref_pitch_roll_std = itAfter->roll_pitch_var;
        error2Gt.header.stamp = stateIter->timestamp;
        //RCLCPP_INFO(this->get_logger(), "calculateError successful");
        pvtErrorPub_->publish(error2Gt);
        stateIter = stateCached.erase(stateIter);
        continue;
      }
      stateIter++;
    }

  }

}
