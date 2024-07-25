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

#include "integrator/GNSSTCIntegrator.h"


namespace fgo::integrator {
  void GNSSTCIntegrator::initialize(rclcpp::Node &node,
                                    graph::GraphBase &graphPtr,
                                    const std::string &integratorName,
                                    bool isPrimarySensor) {
    IntegratorBase::initialize(node, graphPtr, integratorName, isPrimarySensor);
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "--------------------- " << integratorName_
                                                                           << ": start initialization... ---------------------");
    paramPtr_ = std::make_shared<IntegratorGNSSTCParams>(integratorBaseParamPtr_);

    /*
     * Init Parameters
     */

    RosParameter<std::string> sensorAntMainName("GNSSFGO." + integratorName_ + ".sensorAntMain", node);
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "sensorAntMainName: " << sensorAntMainName.value());
    RosParameter<std::string> sensorAntAuxName("GNSSFGO." + integratorName_ + ".sensorAntAux", node);
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "sensorAntAuxName: " << sensorAntAuxName.value());
    baseToAntMainTrans_ = sensorCalibManager_->getTransformationFromBase(sensorAntMainName.value());
    baseToAntAuxTrans_ = sensorCalibManager_->getTransformationFromBase(sensorAntAuxName.value());


    RosParameter<int> GNSSMeasurementFrequency("GNSSFGO." + integratorName_ + ".GNSSMeasurementFrequency", node);
    paramPtr_->GNSSMeasurementFrequency = GNSSMeasurementFrequency.value();

    RosParameter<bool> boolUseDualAntenna("GNSSFGO." + integratorName_ + ".useDualAntenna", false, node);
    paramPtr_->useDualAntenna = boolUseDualAntenna.value();

    RosParameter<bool> boolUseDualAntennaDD("GNSSFGO." + integratorName_ + ".useDualAntennaDD", false, node);
    paramPtr_->useDualAntennaDD = boolUseDualAntennaDD.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "useDualAntennaDD:" << (paramPtr_->useDualAntennaDD ? "true" : "false"));

    RosParameter<bool> boolPseudoRangeDoppler("GNSSFGO." + integratorName_ + ".usePseudoRangeDoppler", false, node);
    paramPtr_->usePseudoRangeDoppler = boolPseudoRangeDoppler.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "usePseudoRangeDoppler:" << (paramPtr_->usePseudoRangeDoppler ? "true" : "false"));

    RosParameter<bool> boolPseudoRange("GNSSFGO." + integratorName_ + ".usePseudoRange", false, node);
    paramPtr_->usePseudoRange = boolPseudoRange.value(); // only if PRDR activated
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "usePseudoRange: " << (paramPtr_->usePseudoRange ? "true" : "false"));

    RosParameter<bool> boolDopplerRange("GNSSFGO." + integratorName_ + ".useDopplerRange", false, node);
    paramPtr_->useDopplerRange = boolDopplerRange.value(); // only if PRDR activated
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "useDopplerRange:" << (paramPtr_->useDopplerRange ? "true" : "false"));

    RosParameter<bool> boolDDCarrierPhase("GNSSFGO." + integratorName_ + ".useDDCarrierPhase", false, node);
    paramPtr_->useDDCarrierPhase = boolDDCarrierPhase.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "useDDCarrierPhase:" << (paramPtr_->useDDCarrierPhase ? "true" : "false"));

    RosParameter<bool> boolDDPseudoRange("GNSSFGO." + integratorName_ + ".useDDPseudoRange", false, node);
    paramPtr_->useDDPseudoRange = boolDDPseudoRange.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "useDDPseudoRange:" << (paramPtr_->useDDPseudoRange ? "true" : "false"));

    RosParameter<bool> boolTimeDCarrierPhase("GNSSFGO." + integratorName_ + ".useTDCarrierPhase", false, node);
    paramPtr_->useTDCarrierPhase = boolTimeDCarrierPhase.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "useTDCarrierPhase:" << (paramPtr_->useTDCarrierPhase ? "true" : "false"));

    RosParameter<bool> boolUseRTCMDD("GNSSFGO." + integratorName_ + ".useRTCMDD", false, node);
    paramPtr_->useRTCMDD = boolUseRTCMDD.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "useRTCMDD:" << (paramPtr_->useRTCMDD ? "true" : "false"));

    RosParameter<bool> boolPRangeUseRawStd("GNSSFGO." + integratorName_ + ".pseudorangeUseRawStd", false, node);
    paramPtr_->pseudorangeUseRawStd = boolPRangeUseRawStd.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "pseudorangeUseRawStd: " << (paramPtr_->pseudorangeUseRawStd ? "true" : "false"));

    RosParameter<int> intNumPRangeFactor("GNSSFGO." + integratorName_ + ".pseudorangeFactorTil", 0, node);
    paramPtr_->pseudorangeFactorTil = intNumPRangeFactor.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "pseudorangeFactorTil: " << paramPtr_->pseudorangeFactorTil);

    RosParameter<double> pseudorangeStd("GNSSFGO." + integratorName_ + ".pseudorangeVarScaleAntMain", 2400., node);
    paramPtr_->pseudorangeVarScaleAntMain = pseudorangeStd.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "pseudorangeVarScaleAntMain:" << paramPtr_->pseudorangeVarScaleAntMain);

    RosParameter<double> dopplerStd("GNSSFGO." + integratorName_ + ".dopplerrangeVarScaleAntMain", 16., node);
    paramPtr_->dopplerrangeVarScaleAntMain = dopplerStd.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "dopplerrangeVarScaleAntMain:" << paramPtr_->dopplerrangeVarScaleAntMain);

    RosParameter<double> pseudorangeVarScaleAntAux("GNSSFGO." + integratorName_ + ".pseudorangeVarScaleAntAux", 2400.,
                                                   node);
    paramPtr_->pseudorangeVarScaleAntAux = pseudorangeVarScaleAntAux.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "pseudorangeVarScaleAntAux:" << paramPtr_->pseudorangeVarScaleAntAux);

    RosParameter<double> dopplerrangeVarScaleAntAux("GNSSFGO." + integratorName_ + ".dopplerrangeVarScaleAntAux", 16.,
                                                    node);
    paramPtr_->dopplerrangeVarScaleAntAux = dopplerrangeVarScaleAntAux.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "dopplerrangeVarScaleAntAux:" << paramPtr_->dopplerrangeVarScaleAntAux);

    RosParameter<double> doublecarrierStdScale("GNSSFGO." + integratorName_ + ".carrierStdScale", false, node);
    paramPtr_->carrierStdScale = doublecarrierStdScale.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "carrierStdScale:" << paramPtr_->carrierStdScale);

    RosParameter<double> carrierphaseStd("GNSSFGO." + integratorName_ + ".carrierphaseStd", 0.1, node);
    paramPtr_->carrierphaseStd = carrierphaseStd.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "carrierphaseStd:" << paramPtr_->carrierphaseStd);

    RosParameter<int> thresholdSatNumToCreateTDCP("GNSSFGO." + integratorName_ + ".thresholdSatNumToCreateTDCP", 3,
                                                  node);
    paramPtr_->thresholdSatNumToCreateTDCP = thresholdSatNumToCreateTDCP.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "thresholdSatNumToCreateTDCP: " << paramPtr_->thresholdSatNumToCreateTDCP);

    RosParameter<double> ddCPStart("GNSSFGO." + integratorName_ + ".ddCPStart", 0.1, node);
    paramPtr_->ddCPStart = ddCPStart.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "ddCPStart:" << paramPtr_->ddCPStart);

    RosParameter<double> initCovforIntAmb("GNSSFGO." + integratorName_ + ".initCovforIntAmb", 0.1, node);
    paramPtr_->initCovforIntAmb = initCovforIntAmb.value();
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "initCovforIntAmb:" << paramPtr_->initCovforIntAmb);

    RosParameter<std::string> weightingModel("GNSSFGO." + integratorName_ + ".weightingModel", "STD", node);
    //paramPtr_->weightingModel = weightingModel.value();
    //RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "weightingModel:" << paramPtr_->weightingModel);

    RosParameter<std::string> noiseModelPRDR("GNSSFGO." + integratorName_ + ".noiseModelPRDR", "gaussian", node);
    setNoiseModelFromParam(noiseModelPRDR.value(), paramPtr_->noiseModelPRDR, "GNSSTC PRDR");
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModel PRDR:" << noiseModelPRDR.value());

    RosParameter<std::string> noiseModelDDCP("GNSSFGO." + integratorName_ + ".noiseModelDDCP", "gaussian", node);
    setNoiseModelFromParam(noiseModelDDCP.value(), paramPtr_->noiseModelDDCP, "GNSSTC PRDR");
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModel DDCP:" << noiseModelDDCP.value());

    RosParameter<std::string> noiseModelTDCP("GNSSFGO." + integratorName_ + ".noiseModelTDCP", "gaussian", node);
    setNoiseModelFromParam(noiseModelTDCP.value(), paramPtr_->noiseModelTDCP, "GNSSTC TDCP");
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "noiseModel TDCP:" << noiseModelTDCP.value());

    RosParameter<double> robustParameterPRDR("GNSSFGO." + integratorName_ + ".robustParamPRDR", .5, node);
    paramPtr_->robustParameterPRDR = robustParameterPRDR.value();
    RosParameter<double> robustParameterDDCP("GNSSFGO." + integratorName_ + ".robustParamDDCP", .5, node);
    paramPtr_->robustParameterDDCP = robustParameterDDCP.value();
    RosParameter<double> robustParameterTDCP("GNSSFGO." + integratorName_ + ".robustParamTDCP", .5, node);
    paramPtr_->robustParameterTDCP = robustParameterTDCP.value();

    ::utils::RosParameter<bool> notIntegrating("GNSSFGO." + integratorName_ + ".notIntegrating", false, *rosNodePtr_);
    paramPtr_->notIntegrating = notIntegrating.value();

    ::utils::RosParameter<double> zeroVelocityThreshold("GNSSFGO." + integratorName_ + ".zeroVelocityThreshold", node);
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "zeroVelocityThreshold: " << zeroVelocityThreshold.value());
    paramPtr_->zeroVelocityThreshold = zeroVelocityThreshold.value();

    RosParameter<bool> delayFromPPS("GNSSFGO." + integratorName_ + ".delayFromPPS", false, node);
    paramPtr_->delayFromPPS = delayFromPPS.value(); // only if PRDR activated
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "delayFromPPS: " << (paramPtr_->delayFromPPS ? "true" : "false"));

    RosParameter<bool> addCBDPriorFactorOnNoGNSS("GNSSFGO." + integratorName_ + ".addCBDPriorFactorOnNoGNSS", node);
    paramPtr_->addCBDPriorFactorOnNoGNSS = addCBDPriorFactorOnNoGNSS.value(); // only if PRDR activated
    RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                       "addCBDPriorFactorOnNoGNSS: " << (paramPtr_->addCBDPriorFactorOnNoGNSS ? "true" : "false"));


    /*
     * Init ROS Ultilities
     */

    if (!paramPtr_->offlineProcess) {
      callbackGroupMap_.insert(std::make_pair("GNSSDelayCalculator", rosNodePtr_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive)));
      callbackGroupMap_.insert(
        std::make_pair("GNSS", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));
      callbackGroupMap_.insert(
        std::make_pair("PPS", rosNodePtr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)));

      gnssDataBuffer_.resize_buffer(300);
      gnssLabelingMsgBuffer_.resize_buffer(300);

      RosParameter<bool> add_local_pps_delay("GNSSFGO.addLocalPPSDelay", false, node);
      delayCalculator_ = std::make_unique<fgo::utils::MeasurementDelayCalculator>(node,
                                                                                  callbackGroupMap_["GNSSDelayCalculator"],
                                                                                  add_local_pps_delay.value());


      gnssLabelingPub_ = rosNodePtr_->create_publisher<irt_nav_msgs::msg::GNSSLabeling>("gnss_labeling",
                                                                                        rclcpp::SystemDefaultsQoS());

      gnssLabelingPubRaw_ = rosNodePtr_->create_publisher<irt_nav_msgs::msg::GNSSLabeling>("gnss_labeling_raw",
                                                                                           rclcpp::SystemDefaultsQoS());

      auto subPPSopt = rclcpp::SubscriptionOptions();
      subPPSopt.callback_group = callbackGroupMap_["PPS"];
      subPPS_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::PPS>("/irt_gpio/novatel/pps",
                                                                         rclcpp::SystemDefaultsQoS(),

                                                                         [this](
                                                                           const irt_nav_msgs::msg::PPS::ConstSharedPtr msg) -> void {
                                                                           delayCalculator_->setPPS(msg);
                                                                         },
                                                                         subPPSopt);

      auto subGNSSOpt = rclcpp::SubscriptionOptions();
      subGNSSOpt.callback_group = callbackGroupMap_["GNSS"];
      subGNSS_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::GNSSObsPreProcessed>(
        "/irt_gnss_preprocessing/gnss_obs_preprocessed",
        rclcpp::SensorDataQoS(),
        std::bind(&GNSSTCIntegrator::onGNSSMsgCb,
                  this, std::placeholders::_1),
        subGNSSOpt);

      subPVA_ = rosNodePtr_->create_subscription<irt_nav_msgs::msg::PVAGeodetic>("/irt_gnss_preprocessing/PVT",
                                                                                 rclcpp::SensorDataQoS(),
                                                                                 std::bind(
                                                                                   &GNSSTCIntegrator::onIRTPVTMsgCb,
                                                                                   this, std::placeholders::_1));
    } else
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), integratorName_ + ": in offline process ...");

    RCLCPP_INFO(rosNodePtr_->get_logger(),
                "--------------------- GNSSTCIntegrator READING NLOS LOOKUPTABLE! ---------------------");
    RosParameter<std::string> NLOSCSVFilePath("GNSSFGO." + integratorName_ + ".NLOSCSVFilePath", node);

    if (!NLOSCSVFilePath.value().empty()) {
      rapidcsv::Document doc(NLOSCSVFilePath.value(), rapidcsv::LabelParams(0, -1));
      const auto rowNum = doc.GetRowCount();
      for (size_t i = 0; i < rowNum; i++) {
        const auto tow = int(doc.GetCell<double>(0, i) * 10);
        const auto prn = doc.GetCell<int>(1, i);
        const auto isLOS = doc.GetCell<int>(2, i);

        auto tsPos = LOSLoopUpTable_.find(tow);
        if (tsPos == LOSLoopUpTable_.end()) {
          std::map<int, bool> newLOSVec;
          newLOSVec.insert(std::make_pair(prn, isLOS == 1));
          LOSLoopUpTable_.insert(std::make_pair(tow, newLOSVec));
        } else {
          LOSLoopUpTable_[tow].insert(std::make_pair(prn, isLOS == 1));
        }
      }
    }

    RCLCPP_INFO(rosNodePtr_->get_logger(), "--------------------- GNSSTCIntegrator initialized! ---------------------");
  }

  bool GNSSTCIntegrator::addFactors(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                                    const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> &stateIDAccMap,
                                    const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                                    std::vector<std::pair<rclcpp::Time, fgo::data::State>> &timePredStates,
                                    gtsam::Values &values,
                                    fgo::solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
                                    gtsam::KeyVector &relatedKeys) {
    static uint notifyCounter = paramPtr_->IMUMeasurementFrequency / paramPtr_->optFrequency;
    static double halfStateBetweenTime = 1. / (double) notifyCounter / 2.;

    static gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j,/*ddAmb_key_j,*/ omega_key_j, tdAmb_key_i,
      pose_key_i, vel_key_i, bias_key_i, cbd_key_i,/*ddAmb_key_i,*/ omega_key_i, tdAmb_key_j,
      pose_key_sync, vel_key_sync, bias_key_sync, cbd_key_sync, omega_key_sync, tdAmb_key_sync;
    static std::list<std::pair<uint32_t, bool>> notSlippedSatellites; //TODO not needed without DD
    static uint consecutiveSyncs = 1;
    static bool lastGNSSInterpolated = false;
    static gtsam::Key lastStateJ = -1;
    static boost::circular_buffer<fgo::data::GNSSMeasurement> restGNSSMeas(10);
    static uint64_t lastPriorCbdNState = nState_;
    nState_ = currentKeyIndexTimestampMap.end()->first;

    auto dataSensor = gnssDataBuffer_.get_all_buffer_and_clean();

    if (dataSensor.empty() && paramPtr_->addCBDPriorFactorOnNoGNSS) {
      const auto &cbd_prior = timePredStates.back().second.cbd;
      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                         integratorName_ << ": no GNSS obs., adding cbd prior " << cbd_prior);
      graphPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(C(nState_), cbd_prior,
                                                                    gtsam::noiseModel::Diagonal::Variances(
                                                                      gtsam::Vector2(
                                                                        std::pow(paramPtr_->constBiasStd, 2),
                                                                        std::pow(paramPtr_->constDriftStd, 2))));
      lastPriorCbdNState = nState_;
      return true;
    }

    // we copy the data of last optimized state for safe, because this is used in imu cb.
    if (!restGNSSMeas.empty()) {
      dataSensor.insert(dataSensor.begin(), restGNSSMeas.begin(), restGNSSMeas.end());
      restGNSSMeas.clear();
    }
    //create GP interpolators for the factor
    if (paramPtr_->gpType == fgo::data::GPModelType::WNOJ) {
      interpolatorI_ = std::make_shared<fgo::models::GPWNOJInterpolator>(
        gtsam::noiseModel::Diagonal::Variances(paramPtr_->QcGPInterpolatorFull), 0, 0,
        paramPtr_->AutoDiffGPInterpolatedFactor, paramPtr_->GPInterpolatedFactorCalcJacobian);
    } else if (paramPtr_->gpType == fgo::data::GPModelType::WNOA) {
      interpolatorI_ = std::make_shared<fgo::models::GPWNOAInterpolator>(
        gtsam::noiseModel::Diagonal::Variances(paramPtr_->QcGPInterpolatorFull), 0, 0,
        paramPtr_->AutoDiffGPInterpolatedFactor, paramPtr_->GPInterpolatedFactorCalcJacobian);
    } else {
      RCLCPP_WARN(rosNodePtr_->get_logger(), "NO gpType chosen. Please choose.");
      return false;
    }

    auto gnssIter = dataSensor.begin();

    while (gnssIter != dataSensor.end()) {
      // we iterate then though all gnss measurements, trying to find the corresponding one, this could be exhausted,
      // thus it must be considered carefully.
      //RCLCPP_INFO(appPtr_->get_logger(), "1");
      // firstly, we check, whether there are gnss measurements, which are delayed and should be integrated in LAST optimization epochs!

      auto corrected_time_gnss_meas =
        gnssIter->measMainAnt.timestamp.seconds() - gnssIter->measMainAnt.delay;   // in double
      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), std::fixed << "Current GNSS ts: " << corrected_time_gnss_meas);

      auto current_pred_state = timePredStates.back().second; //graph::querryCurrentPredictedState(timePredStates, corrected_time_gnss_meas);

      //this_gyro = current_pred_state.imuBias.correctGyroscope(this_gyro);

      auto syncResult = findStateForMeasurement(currentKeyIndexTimestampMap, corrected_time_gnss_meas, paramPtr_);

      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "Found State I: " << std::fixed <<
                                                                      syncResult.keyIndexI << " at: "
                                                                      << syncResult.timestampI << " and J: "
                                                                      << syncResult.keyIndexJ << " at: "
                                                                      << syncResult.timestampJ
                                                                      << " DurationToI: "
                                                                      << syncResult.durationFromStateI);

      if (!syncResult.foundI) {
        RCLCPP_WARN(rosNodePtr_->get_logger(), "State I or J couldn't be found in varIDTimestampMap.");
        gnssIter++;
        continue;
      }

      pose_key_i = X(syncResult.keyIndexI);
      vel_key_i = V(syncResult.keyIndexI);
      omega_key_i = W(syncResult.keyIndexI);
      bias_key_i = B(syncResult.keyIndexI);
      cbd_key_i = C(syncResult.keyIndexI);
      tdAmb_key_i = N(syncResult.keyIndexI);

      pose_key_j = X(syncResult.keyIndexJ);
      vel_key_j = V(syncResult.keyIndexJ);
      omega_key_j = W(syncResult.keyIndexJ);
      bias_key_j = B(syncResult.keyIndexJ);
      cbd_key_j = C(syncResult.keyIndexJ);
      tdAmb_key_j = N(syncResult.keyIndexJ);

      const double delta_t = syncResult.timestampJ - syncResult.timestampI;
      const double taui = syncResult.durationFromStateI;
      if (!syncResult.stateJExist()) {
        RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "GP GNSS: NO state J found !!! ");
      }
      if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I ||
          syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J) {
        const auto [foundGyro, this_gyro] = findOmegaToMeasurement(corrected_time_gnss_meas, timestampGyroMap);
        consecutiveSyncs++; //we were able to sync
        double time_synchronized;
        // now we found a state which is synchronized with the GNSS obs
        if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I) {
          pose_key_sync = pose_key_i;
          vel_key_sync = vel_key_i;
          bias_key_sync = bias_key_i;
          cbd_key_sync = cbd_key_i;
          omega_key_sync = omega_key_i;
          tdAmb_key_sync = tdAmb_key_i;
          time_synchronized = syncResult.timestampI;
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                             "[GNSSObs.] Found time synchronized state at I " << gtsam::symbolIndex(pose_key_sync)
                                                                              <<
                                                                              " with time difference: "
                                                                              << syncResult.durationFromStateI);
        } else {
          pose_key_sync = pose_key_j;
          vel_key_sync = vel_key_j;
          bias_key_sync = bias_key_j;
          cbd_key_sync = cbd_key_j;
          omega_key_sync = omega_key_j;
          tdAmb_key_sync = tdAmb_key_j;
          time_synchronized = syncResult.timestampJ;
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                             "[GNSSObs.] Found time synchronized state at J " << gtsam::symbolIndex(pose_key_sync)
                                                                              <<
                                                                              " with time difference: "
                                                                              << syncResult.durationFromStateI);
        }

        //PSEUDORANGE DOPPLER SYNCED
        if (paramPtr_->usePseudoRangeDoppler &&
            (paramPtr_->pseudorangeFactorTil == 0 || paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "PRDR1 n: " << gnssIter->measMainAnt.obs.size());
          this->addGNSSPrDrFactor(pose_key_sync, vel_key_sync, bias_key_sync, cbd_key_sync, gnssIter->measMainAnt.obs,
                                  this_gyro, 1);
        } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->usePseudoRange &&
                   (paramPtr_->pseudorangeFactorTil == 0 || paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "PR1 n: " << gnssIter->measMainAnt.obs.size());
          this->addGNSSPrFactor(pose_key_sync, cbd_key_sync, gnssIter->measMainAnt.obs, 1);
        } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->useDopplerRange) {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "DR1 n: " << gnssIter->measMainAnt.obs.size());
          this->addGNSSDrFactor(pose_key_sync, vel_key_sync, cbd_key_sync, gnssIter->measMainAnt.obs, this_gyro, 1);
        } else {
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "No Pr Dr integrated!");
        }

        if (paramPtr_->useDualAntenna && gnssIter->hasDualAntenna) {
          if (paramPtr_->usePseudoRangeDoppler &&
              (paramPtr_->pseudorangeFactorTil == 0 || paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "PRDR2 n: " << gnssIter->measAuxAnt.obs.size());
            this->addGNSSPrDrFactor(pose_key_sync, vel_key_sync, bias_key_sync, cbd_key_sync, gnssIter->measAuxAnt.obs,
                                    this_gyro, 2);
          } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->usePseudoRange &&
                     (paramPtr_->pseudorangeFactorTil == 0 ||
                      paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "PR2 n: " << gnssIter->measAuxAnt.obs.size());
            this->addGNSSPrFactor(pose_key_sync, cbd_key_sync, gnssIter->measAuxAnt.obs, 2);
          } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->useDopplerRange) {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "DR2 n: " << gnssIter->measAuxAnt.obs.size());
            this->addGNSSDrFactor(pose_key_sync, vel_key_sync, cbd_key_sync, gnssIter->measAuxAnt.obs, this_gyro, 2);
          } else {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Aux ant: No Pr Dr integrated!");
          }
        }

        //DOUBLE DIFFERENCE PSEUDORANGE SYNCED
        if (paramPtr_->usePseudoRangeDoppler && paramPtr_->useRTCMDD && gnssIter->hasRTK) {
          this->addGNSSDDPrDrFactor(pose_key_sync, vel_key_sync, gnssIter->measRTCMDD.obs,
                                    gnssIter->measRTCMDD.refSatGPS, gnssIter->measRTCMDD.basePosRTCM, this_gyro, 1);
        }

        if (paramPtr_->usePseudoRangeDoppler && paramPtr_->useDualAntenna && gnssIter->hasDualAntennaDD) {
          this->addGNSSDDPrDrFactor(pose_key_sync, vel_key_sync, gnssIter->measDualAntennaDD.obs,
                                    gnssIter->measDualAntennaDD.refSatGPS, gnssIter->measRTCMDD.basePosRTCM, this_gyro,
                                    2);
        }

        //DISABLED
        if (paramPtr_->useDDCarrierPhase && paramPtr_->useRTCMDD && gnssIter->hasRTK) {
          this->addGNSSDDCPFactor(pose_key_sync, /*TODO put ambiguity key here*/ pose_key_sync,
                                  gnssIter->measRTCMDD.obs, gnssIter->measRTCMDD.refSatGPS,
                                  gnssIter->measRTCMDD.basePosRTCM, 1, current_pred_state);
          //because we create normal not GP we reset GP notSLippedlist
          notSlippedSatellites.clear();
          for (auto &obs: gnssIter->measRTCMDD.obs) {
            notSlippedSatellites.emplace_back(obs.satId, false);
          }
          gtsam::Vector xVec;
          xVec.resize(gnssIter->measRTCMDD.obs.size());
          try {
            //values_.insert(ddAmb_key_i, xVec);
          } catch (const gtsam::ValuesKeyAlreadyExists &e) {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "The value already exist at: " /*<< ddAmb_key_i*/);
          }
        }

        if (paramPtr_->useTDCarrierPhase) {
          //new Version of TDCP
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "TDNCP n: " << gnssIter->measMainAnt.obs.size());
          this->addGPInterpolatedTDNormalCPFactor(pose_key_i, vel_key_i, omega_key_i, cbd_key_sync, pose_key_j,
                                                  vel_key_j, omega_key_j,
                                                  gnssIter->measMainAnt.obs, consecutiveSyncs, time_synchronized,
                                                  interpolatorI_,
                                                  interpolatorJ_, delta_t, 0, syncResult.status, values,
                                                  keyTimestampMap); //TODO Time - TIme doesnt work atm hardcoded
        }
      } else if (syncResult.status == StateMeasSyncStatus::INTERPOLATED && paramPtr_->addGPInterpolatedFactor) {
        //recalculate interpolator // set up interpolator
        //corrected_time_gnss_meas - timestampI;
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "GP delta: " << delta_t << " tau: " << taui);
        //ALSO NEEDED FOR DDCP TDCP, AND THATS IN SYNCED CASE AND IN NOT SYNCED CASE
        if (paramPtr_->gpType == fgo::data::GPModelType::WNOJ) {
          const auto [foundI, accI, foundJ, accJ] = findAccelerationToState(syncResult.keyIndexI, stateIDAccMap);
          interpolatorI_->recalculate(delta_t, taui, accI, accJ);
        } else
          interpolatorI_->recalculate(delta_t, taui);

        // RCLCPP_ERROR_STREAM(appPtr_->get_logger(), "accI.head(3): " << accI.head(3) << "\n" << accI.tail(3));



        // here, there is no time synchronized state found, we need to use GP interpolated GNSS factor with j-1 and j
        consecutiveSyncs = 0; //we werent able to sync
        lastGNSSInterpolated = false;
        RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                           "[GNSSObs.] Found not synchronized between " << syncResult.keyIndexI << " and "
                                                                        << syncResult.keyIndexJ <<
                                                                        " with time difference: "
                                                                        << syncResult.durationFromStateI);
        uint32_t biasCbdKeyOffset = 0;
        if (taui > halfStateBetweenTime)
          biasCbdKeyOffset = 1;

        //PSEUDORANGE DOPPLER GP
        if (paramPtr_->usePseudoRangeDoppler &&
            (paramPtr_->pseudorangeFactorTil == 0 || paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "GPPRDR1 n: " << gnssIter->measMainAnt.obs.size());
          this->addGPInterpolatedGNSSPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                                cbd_key_i, gnssIter->measMainAnt.obs, interpolatorI_, 1);
        } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->usePseudoRange &&
                   (paramPtr_->pseudorangeFactorTil == 0 || paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "GPPR1 n: " << gnssIter->measMainAnt.obs.size());
          this->addGPInterpolatedPrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                          cbd_key_i + biasCbdKeyOffset,
                                          gnssIter->measMainAnt.obs, interpolatorI_, 1);
        } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->useDopplerRange) {
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "GPDR1 n: " << gnssIter->measMainAnt.obs.size());
          this->addGPInterpolatedDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                          cbd_key_i + biasCbdKeyOffset, gnssIter->measMainAnt.obs, interpolatorI_, 1);
        } else {
          RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "No Pr Dr integrated!");
        }

        if (paramPtr_->useDualAntenna && gnssIter->hasDualAntenna) {
          if (paramPtr_->usePseudoRangeDoppler &&
              (paramPtr_->pseudorangeFactorTil == 0 || paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "GPPRDR2 n: " << gnssIter->measAuxAnt.obs.size());
            this->addGPInterpolatedGNSSPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j,
                                                  omega_key_j,
                                                  cbd_key_i + biasCbdKeyOffset, gnssIter->measMainAnt.obs,
                                                  interpolatorI_, 2);
          } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->usePseudoRange &&
                     (paramPtr_->pseudorangeFactorTil == 0 ||
                      paramPtr_->pseudorangeFactorTil >= syncResult.keyIndexJ)) {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "GPPR2 n: " << gnssIter->measAuxAnt.obs.size());
            this->addGPInterpolatedPrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                            cbd_key_i + biasCbdKeyOffset,
                                            gnssIter->measAuxAnt.obs, interpolatorI_, 2);
          } else if (!paramPtr_->usePseudoRangeDoppler && paramPtr_->useDopplerRange) {
            RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "GPDR2 n: " << gnssIter->measAuxAnt.obs.size());
            this->addGPInterpolatedDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                            cbd_key_i + biasCbdKeyOffset, gnssIter->measMainAnt.obs, interpolatorI_, 2);
          } else {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "Aux Ant: No Pr Dr integrated!");
          }

        }

        //DOUBLE DIFFERENCE PSEUDORANGE GP
        if (paramPtr_->useDDPseudoRange && paramPtr_->useRTCMDD && gnssIter->hasRTK) {
          this->addGPInterpolatedDDPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                              gnssIter->measRTCMDD.obs, gnssIter->measRTCMDD.refSatGPS,
                                              gnssIter->measRTCMDD.basePosRTCM, interpolatorI_, 1);
        }
        if (paramPtr_->useDDPseudoRange && paramPtr_->useDualAntenna && gnssIter->hasDualAntennaDD) {
          this->addGPInterpolatedDDPrDrFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                              gnssIter->measDualAntennaDD.obs, gnssIter->measDualAntennaDD.refSatGPS,
                                              gnssIter->measRTCMDD.basePosRTCM, interpolatorI_, 2);
        }
        //DOUBLE DIFFERENCE CARRIERPHASE
        //resets if lastStateJ != keyIndexJ
        //DISABLED
        if (paramPtr_->useDDCarrierPhase && paramPtr_->useRTCMDD && gnssIter->hasRTK && 0) {
          this->addGPInterpolatedDDCPFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j,
                                            omega_key_j/*TODO put ambiguity key*/, gnssIter->measRTCMDD.obs,
                                            gnssIter->measRTCMDD.refSatGPS.refSatPos,
                                            gnssIter->measRTCMDD.basePosRTCM,
                                            interpolatorI_, notSlippedSatellites, lastStateJ != syncResult.keyIndexJ);
        }
        //DISABLED
        if (lastStateJ != syncResult.keyIndexJ && paramPtr_->useDDCarrierPhase && gnssIter->measRTCMDD.obs.size() &&
            0) {
          gtsam::Vector xVec;
          xVec.resize(gnssIter->measRTCMDD.obs.size());
          try {
            //values_.insert(ddAmb_key_i, xVec);
          } catch (const gtsam::ValuesKeyAlreadyExists &e) {
            RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "The value already exist at: " /*<< ddAmb_key_j*/);
          }
        }
        //TIME DIFFERENCE DD CARRIERPHASE
        if (paramPtr_->useTDCarrierPhase) {
          //new Version of TDCP
          RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "TDNCP n: " << gnssIter->measMainAnt.obs.size());
          this->addGPInterpolatedTDNormalCPFactor(pose_key_i, vel_key_i, omega_key_i, //tdAmb_key_i,
                                                  cbd_key_i + biasCbdKeyOffset, pose_key_j, vel_key_j,
                                                  omega_key_j, gnssIter->measMainAnt.obs,
                                                  consecutiveSyncs, syncResult.timestampJ, interpolatorI_,
                                                  interpolatorJ_,
                                                  delta_t, taui, syncResult.status,
                                                  values, keyTimestampMap,
                                                  lastGNSSInterpolated); //TODO Time - TIme doesnt work atm hardcoded

          /* old version of TDCP
          this->addGPInterpolatedTDCPFactor(pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j,
                                      omega_key_j, gnssIter->measRTCMDD.obs, gnssIter->measRTCMDD.refSatGPS.refSatPos,
                                      gnssIter->measRTCMDD.refSatGPS.refSatSVID, gnssIter->measRTCMDD.basePosRTCM,
                                      interpolatori, interpolatorj, keyIndexJ);*/
        }
        //TIME DIFFERENCE CARRIERPHASE

        lastStateJ = syncResult.keyIndexJ;
        lastGNSSInterpolated = true;
      } else if (syncResult.status == StateMeasSyncStatus::CACHED) {
        lastGNSSInterpolated = false;
        RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), "[GNSSObs.] CAN not synchronize backup GNSS");
        // In this case, this gnss meas is in front of all imu meas, then we cache it
        // NOTICES: this shouldn't happen, if so, there must be sth wrong!
        restGNSSMeas.push_back(*gnssIter);
        //ToDo: Attention! Because the data timestamp in the container is monotone increasing,
        // it makes no sense to iterate further if the current data is in front of the last state!
        break;
      }

      //corrected_time_last_gnss = corrected_time_gnss_meas;
      if (syncResult.stateJExist()) {
        interpolatorJ_ = interpolatorI_;
      }
      gnssIter++;
    }

    return true;
  }

  bool GNSSTCIntegrator::fetchResult(const gtsam::Values &result, const gtsam::Marginals &martinals,
                                     const solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                                     data::State &optState) {


    return true;
  }

  void GNSSTCIntegrator::onGNSSMsgCb(irt_nav_msgs::msg::GNSSObsPreProcessed::ConstSharedPtr gnssMsg) {
    static rclcpp::Time last_gnss_time = rclcpp::Time(gnssMsg->header.stamp.sec, gnssMsg->header.stamp.nanosec,
                                                      RCL_ROS_TIME);
    static rclcpp::Time last_gnss_time_came = rosNodePtr_->now();
    static auto start = rosNodePtr_->now();
    static double lastDelay = 0.;
    static uint64_t gnssCounter = 0;
    const auto thisGNSSTime = rclcpp::Time(gnssMsg->header.stamp.sec, gnssMsg->header.stamp.nanosec, RCL_ROS_TIME);
    static bool first_measurement = true;

    irt_nav_msgs::msg::SensorProcessingReport thisProcessingReport;
    thisProcessingReport.sensor_name = "GNSSTC";
    thisProcessingReport.ts_measurement = thisGNSSTime.seconds();
    thisProcessingReport.ts_start_processing = start.seconds();
    thisProcessingReport.observation_available = true;

    delayCalculator_->setTOW(gnssMsg->gnss_obs_ant_main.time_receive);
    // we wait here for 0.001s so that the the integrator in delay calculator can set the new tow 1000000 nanosec = 0.001 sec

    auto now_ = rosNodePtr_->now();
    auto start_time = std::chrono::system_clock::now();

    //change datatype
    fgo::data::GNSSMeasurement gnssMeasurement = sensor::gnss::convertGNSSObservationMsg(*gnssMsg,
                                                                                         paramPtr_); //convertGNSSMsg(gnssMsg);
    const auto measurement_delay = delayCalculator_->getDelay();


    thisProcessingReport.num_measurements = gnssMeasurement.measMainAnt.obs.size();

    //RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(), "last gnss " << std::fixed << last_gnss_time.seconds());
    //RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
    //                   "this gnss " << std::fixed << rclcpp::Time(gnssMsg->header.stamp).seconds());
    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "this gnss tow " << std::fixed << gnssMsg->gnss_obs_ant_main.time_receive);
    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "last gnss came:" << std::fixed << last_gnss_time_came.seconds());
    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "this gnss came: " << std::fixed << now_.seconds());
    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "last gnss to this gnss: " << (rclcpp::Time(gnssMsg->header.stamp) - last_gnss_time).seconds());
    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "last gnss came to this gnss came: " << (now_ - last_gnss_time_came).seconds());
    ///RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
    //                  "Calculated delay: " << std::fixed << gnssMeasurement.measMainAnt.delay);
    //RCLCPP_WARN_STREAM(appPtr_->get_logger(), "Time passed: " << std::fixed << (appPtr_->now() - start).seconds() );

   auto delayFromMsg = thisGNSSTime.seconds() - last_gnss_time.seconds() - 0.1; //

    // we check if we need to consider last delay

    if (delayFromMsg < -0.001 && lastDelay > 0)
      delayFromMsg += lastDelay;

    // + lastDelay;
    delayFromMsg = delayFromMsg > 0.3 ? gnssMeasurement.measMainAnt.delay : delayFromMsg;

    if (first_measurement) {
      first_measurement = false;
      delayFromMsg = 0.;
      gnssMeasurement.measMainAnt.delay = 0;
    }

    if (delayFromMsg < 0.)
      delayFromMsg = 0.;

    if (!paramPtr_->delayFromPPS) {
      //RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
     //                   "delay from msg: " << std::fixed << delayFromMsg << " last delay : " << lastDelay);
      //gnssMeasurement.measMainAnt.delay = delayFromMsg;
      gnssMeasurement.measDualAntennaDD.delay = delayFromMsg;
      gnssMeasurement.measRTCMDD.delay = delayFromMsg;
      gnssMeasurement.measAuxAnt.delay = delayFromMsg;
    } else {
      //gnssMeasurement.measMainAnt.delay = measurement_delay;
      gnssMeasurement.measAuxAnt.delay = measurement_delay;
      gnssMeasurement.measDualAntennaDD.delay = measurement_delay;
      gnssMeasurement.measRTCMDD.delay = measurement_delay; //TODO
    }

    if (abs(delayFromMsg) < 0.005 || delayFromMsg > 0.3)
      lastDelay = 0.;
    else
      lastDelay = delayFromMsg;

    //data saved in preprocessor
    auto correctedGNSSTimeNanoSec = int64_t(
      (thisGNSSTime.seconds() - gnssMeasurement.measMainAnt.delay) * fgo::constants::sec2nanosec);
    auto correctedGNSSTime = rclcpp::Time(correctedGNSSTimeNanoSec, RCL_ROS_TIME);
    //new_label_msg.header.stamp = correctedGNSSTime;
    //new_label_msg.related_state_timestamp_nanosec = correctedGNSSTimeNanoSec;
    //gnssLabelingPubRaw_->publish(new_label_msg);
    // gnssLabelingMsgBuffer_.update_buffer(new_label_msg, correctedGNSSTime);
    gnssDataBuffer_.update_buffer(gnssMeasurement, thisGNSSTime, rosNodePtr_->get_clock()->now());
    if (0 && gnssMeasurement.hasRTCMDD) {
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                         "==" << unsigned(gnssMeasurement.measRTCMDD.refSatGPS.refSatSVID) << "==");
      for (auto &x: gnssMeasurement.measRTCMDD.obs) {
        RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(),
                           "Sat: " << x.satId << " pr: " << x.pr << " cp: " << paramPtr_->lambdaL1 * x.cp << " LT: "
                                   << x.locktime
                                   << " CS: " << x.cycleSlip << " Diff: " << x.pr - paramPtr_->lambdaL1 * x.cp);
      }
    }

    double duration_cb = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::system_clock::now() - start_time).count();

    if (duration_cb > 0.1) {
      RCLCPP_WARN_STREAM(rosNodePtr_->get_logger(),
                         "onGNSSMsgCb: cb takes " << duration_cb << "s, more than expected 0.1s");
    }

    last_gnss_time = thisGNSSTime;
    last_gnss_time_came = now_;

    if (!graphPtr_->isGraphInitialized()) {
      graphPtr_->updateReferenceMeasurementTimestamp(gnssMsg->gnss_obs_ant_main.time_receive, correctedGNSSTime);
      RCLCPP_WARN(rosNodePtr_->get_logger(), "onGNSSMsgCb: graph not initialized, waiting ...");
    }

    thisProcessingReport.duration_processing = duration_cb;
    thisProcessingReport.measurement_delay = gnssMeasurement.measMainAnt.delay;
    thisProcessingReport.header.stamp = rosNodePtr_->now();
    if (pubSensorReport_)
      pubSensorReport_->publish(thisProcessingReport);

  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::integrator::GNSSTCIntegrator, fgo::integrator::IntegratorBase)
