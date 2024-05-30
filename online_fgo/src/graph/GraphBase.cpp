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

#include <algorithm>
#include "graph/GraphBase.h"
#include "integrator/IntegratorBase.h"
#include "gnss_fgo/GNSSFGOLocalizationBase.h"
#include "data/sampling/UncentedSampler.h"
#include "utils/AlgorithmicUtils.h"

namespace fgo::graph {
  using namespace std::chrono_literals;

  GraphBase::GraphBase(gnss_fgo::GNSSFGOLocalizationBase &node) :
    appPtr_(&node),
    integratorLoader_("online_fgo", "fgo::integrator::IntegratorBase") {
    RCLCPP_INFO(appPtr_->get_logger(), "---------------------  GraphBase initializing! --------------------- ");
    graphBaseParamPtr_ = std::make_shared<GraphParamBase>(node.getParamPtr());
    referenceSensorTimestampBuffer_.resize_buffer(50);
    referenceStateBuffer_.resize_buffer(50);
    fgoOptStateBuffer_.resize_buffer(50);
    accBuffer_.resize_buffer(20000);
    currentPredictedBuffer_.resize_buffer(50);

    RosParameter<bool> publishResiduals("GNSSFGO.Graph.publishResiduals", true, node);
    graphBaseParamPtr_->publishResiduals = publishResiduals.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "publishResiduals:" << graphBaseParamPtr_->publishResiduals);

    RosParameter<bool> publishResidualsOnline("GNSSFGO.Graph.publishResidualsOnline", true, node);
    graphBaseParamPtr_->publishResidualsOnline = publishResidualsOnline.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "publishResidualsOnline:" << graphBaseParamPtr_->publishResidualsOnline);

    if (graphBaseParamPtr_->publishResiduals) {
      RosParameter<bool> onlyLastResiduals("GNSSFGO.Graph.onlyLastResiduals", true, node);
      graphBaseParamPtr_->onlyLastResiduals = onlyLastResiduals.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "onlyLastResiduals:" << graphBaseParamPtr_->onlyLastResiduals);
      factorBuffer_.resize_buffer(100);
      pubResiduals_ = appPtr_->create_publisher<irt_nav_msgs::msg::FactorResiduals>("residuals",
                                                                                    rclcpp::SystemDefaultsQoS());

      std::vector<std::string> factorSkipped = {};
      RosParameter<std::vector<std::string>> factorSkippedParam("GNSSFGO.Graph.factorSkipped", factorSkipped, node);

      for (const auto &name: factorSkippedParam.value()) {
        try {
          graphBaseParamPtr_->skippedFactorsForResiduals.emplace_back(fgo::factor::FactorNameIDMap.at(name));
          RCLCPP_INFO_STREAM(appPtr_->get_logger(), "Skipping: " << name << " on residual publishing...");
        }
        catch (std::exception &ex) {
          RCLCPP_WARN_STREAM(appPtr_->get_logger(),
                             name << " not available while skipping factors on residual publishing! " << ex.what());
        }
      }
      if (graphBaseParamPtr_->publishResidualsOnline) {
        resultMarginalBuffer_.resize_buffer(100);
        pubResidualsThread_ = std::make_shared<std::thread>(
          [this]() -> void {
            while (rclcpp::ok()) {
              const auto result_marginal_pairs = resultMarginalBuffer_.get_all_time_buffer_pair_and_clean();
              if (result_marginal_pairs.empty()) {
                rclcpp::sleep_for(10ms);
                continue;
              }
              for (const auto &result_marginal_pair: result_marginal_pairs) {
                const auto &timestamp = result_marginal_pair.first;
                const auto &result = result_marginal_pair.second.first;
                const auto &marginals = result_marginal_pair.second.second;
                this->calculateResiduals(timestamp, result, marginals);
              }
            }
          }
        );
      }
    }

    RosParameter<double> smootherLag("GNSSFGO.Optimizer.smootherLag", 0.1, node);
    graphBaseParamPtr_->smootherLag = smootherLag.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "smootherLag:" << graphBaseParamPtr_->smootherLag);

    RosParameter<std::string> smootherType("GNSSFGO.Optimizer.smootherType", "undefined", node);
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "smootherType:" << smootherType.value());

    if (smootherType.value() == "BatchFixedLag") {
      graphBaseParamPtr_->smootherType = SmootherType::BatchFixedLag;
      //not all param
      gtsam::LevenbergMarquardtParams lmp = gtsam::LevenbergMarquardtParams::LegacyDefaults();
      //The initial Levenberg-Marquardt damping term (default: 1e-5)
      RosParameter<double> lambdaInitial("Optimizer.LM.lambdaInitial", 1e-5, node);
      lmp.lambdaInitial = lambdaInitial.value();
      //The amount by which to multiply or divide lambda when adjusting lambda (default: 10.0)
      RosParameter<double> lambdaFactor("Optimizer.LM.lambdaFactor", 10.0, node);
      lmp.lambdaFactor = lambdaFactor.value();
      //The maximum lambda to try before assuming the optimization has failed (default: 1e5)
      RosParameter<double> lambdaUpperBound("Optimizer.LM.lambdaUpperBound", 1e5, node);
      lmp.lambdaUpperBound = lambdaUpperBound.value();
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "lmp.lambdaUpperBound: " << lmp.lambdaUpperBound);
      //The minimum lambda used in LM (default: 0)
      RosParameter<double> lambdaLowerBound("Optimizer.LM.lambdaLowerBound", 0, node);
      lmp.lambdaLowerBound = lambdaLowerBound.value();
      //if true, use diagonal of Hessian
      RosParameter<bool> diagonalDamping("Optimizer.LM.diagonalDamping", false, node);
      lmp.diagonalDamping = diagonalDamping.value();
      //if true applies constant increase (or decrease) to lambda according to lambdaFactor
      RosParameter<bool> useFixedLambdaFactor("Optimizer.LM.useFixedLambdaFactor", true, node);
      lmp.useFixedLambdaFactor = useFixedLambdaFactor.value();
      //when using diagonal damping saturates the minimum diagonal entries (default: 1e-6)
      RosParameter<double> minDiagonal("Optimizer.LM.minDiagonal", 1e-6, node);
      lmp.minDiagonal = minDiagonal.value();
      //when using diagonal damping saturates the maximum diagonal entries (default: 1e32)
      RosParameter<double> maxDiagonal("Optimizer.LM.maxDiagonal", 1e32, node);
      lmp.maxDiagonal = maxDiagonal.value();
      this->initSolver(lmp);
    } else {
      if (smootherType.value() == "IncrementalFixedLag") {
        graphBaseParamPtr_->smootherType = SmootherType::ISAM2FixedLag;
        gtsam::ISAM2Params isam2Params;
        //Optimization parameters, this both selects the nonlinear optimization method and specifies its parameters, either ISAM2GaussNewtonParams or ISAM2DoglegParams.
        RosParameter<std::string> optimizationParams("Optimizer.ISAM2.optimizationParams", "GN", node);
        if (optimizationParams.value() == "DOGLEG") {
          isam2Params.optimizationParams = gtsam::ISAM2DoglegParams();
        } else {
          isam2Params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
        }
        //Only relinearize variables whose linear delta magnitude is greater than this threshold (default: 0.1).
        RosParameter<double> relinearizeThreshold("Optimizer.ISAM2.relinearizeThreshold", 0.1, node);
        isam2Params.relinearizeThreshold = relinearizeThreshold.value();//0.1 or 0.01
        //Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10)
        RosParameter<int> relinearizeSkip("Optimizer.ISAM2.relinearizeSkip", 10, node);
        isam2Params.relinearizeSkip = relinearizeSkip.value(); //10
        //Specifies whether to use QR or CHOESKY numerical factorization (default: CHOLESKY).
        RosParameter<std::string> factorization("Optimizer.ISAM2.factorization", "CHOLESKY", node);
        if (factorization.value() == "QR") {
          isam2Params.factorization = gtsam::ISAM2Params::QR;
        } else {
          isam2Params.factorization = gtsam::ISAM2Params::CHOLESKY;
        }
        //Whether to compute and return ISAM2Result::detailedResults, this can increase running time (default: false)
        RosParameter<bool> enableDetailedResults("Optimizer.ISAM2.enableDetailedResults", false, node);
        isam2Params.enableDetailedResults = enableDetailedResults.value();
        //When you will be removing many factor when using ISAM2 as a fixed-lag smoother, enable this option to add factor in the first available factor slots,
        RosParameter<bool> findUnusedFactorSlots("Optimizer.ISAM2.findUnusedFactorSlots", false, node);
        isam2Params.findUnusedFactorSlots = findUnusedFactorSlots.value();
        this->initSolver(isam2Params);
      } else {
        RCLCPP_ERROR(appPtr_->get_logger(), "Smoothertype undefined. Please choose Smoothertype.");
        return;
      }
    }

    const auto testDt = 1. / graphBaseParamPtr_->stateFrequency;

    if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJ) {
      RosParameter<std::vector<double>> QcGPMotionPriorFull("GNSSFGO.Optimizer.QcGPWNOJMotionPriorFull", node);
      graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFull.value().data());
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOJMotionPrior:" << graphBaseParamPtr_->QcGPMotionPriorFull);

      RosParameter<std::vector<double>> QcFull("GNSSFGO.Optimizer.QcGPWNOJInterpolatorFull", node);
      graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFull.value().data());
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOJInterpolator:" << graphBaseParamPtr_->QcGPInterpolatorFull);
      gtsam::SharedNoiseModel qcModel = gtsam::noiseModel::Diagonal::Variances(graphBaseParamPtr_->QcGPMotionPriorFull);
      const auto qc = gtsam::noiseModel::Gaussian::Covariance(
        fgo::utils::calcQ3<6>(fgo::utils::getQc(qcModel), testDt));
      std::cout << fgo::utils::getQc(qcModel) << std::endl;
      std::cout << "###" << std::endl;
      qc->print("Q3:");
    } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOA) {
      RosParameter<std::vector<double>> QcGPMotionPriorFull("GNSSFGO.Optimizer.QcGPWNOAMotionPriorFull", node);
      graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFull.value().data());
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOAMotionPrior:" << graphBaseParamPtr_->QcGPMotionPriorFull);

      RosParameter<std::vector<double>> QcFull("GNSSFGO.Optimizer.QcGPWNOAInterpolatorFull", node);
      graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFull.value().data());
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPWNOAInterpolator:" << graphBaseParamPtr_->QcGPInterpolatorFull);
      gtsam::SharedNoiseModel qcModel = gtsam::noiseModel::Diagonal::Variances(graphBaseParamPtr_->QcGPMotionPriorFull);
      auto qc = gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ<6>(fgo::utils::getQc(qcModel), testDt));

      std::cout << fgo::utils::getQc(qcModel) << std::endl;
      std::cout << "###" << std::endl;
      qc->print("Q:");

    } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::Singer) {

      // ToDo @ Zirui, please try to read the Ad matrix for configuration and make a screening here
      RosParameter<std::vector<double>> QcGPMotionPriorFull("GNSSFGO.Optimizer.QcGPSingerMotionPriorFull", node);
      graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFull.value().data());
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPSingerMotionPrior:" << graphBaseParamPtr_->QcGPMotionPriorFull);

      RosParameter<std::vector<double>> QcFull("GNSSFGO.Optimizer.QcGPSingerInterpolatorFull", node);
      graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFull.value().data());
      RCLCPP_INFO_STREAM(appPtr_->get_logger(), "QcGPSingerInterpolator:" << graphBaseParamPtr_->QcGPInterpolatorFull);
      gtsam::SharedNoiseModel qcModel = gtsam::noiseModel::Diagonal::Variances(graphBaseParamPtr_->QcGPMotionPriorFull);
      auto qc = gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ<6>(fgo::utils::getQc(qcModel), testDt));

      std::cout << fgo::utils::getQc(qcModel) << std::endl;
      std::cout << "###" << std::endl;
      qc->print("Q:");

    }

    // PARAM: Using automatic differencing or analytical Jacobians
    RosParameter<bool> AutoDiffNormalFactor("GNSSFGO.Graph.AutoDiffNormalFactor", node);
    graphBaseParamPtr_->AutoDiffNormalFactor = AutoDiffNormalFactor.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(),
                       "AutoDiffNormalFactor: " << (graphBaseParamPtr_->AutoDiffNormalFactor ? "true" : "false"));

    RosParameter<bool> AutoDiffGPInterpolatedFactor("GNSSFGO.Graph.AutoDiffGPInterpolatedFactor", node);
    graphBaseParamPtr_->AutoDiffGPInterpolatedFactor = AutoDiffGPInterpolatedFactor.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "AutoDiffGPInterpolatedFactor: "
      << (graphBaseParamPtr_->AutoDiffGPInterpolatedFactor ? "true" : "false"));

    RosParameter<bool> AutoDiffGPMotionPriorFactor("GNSSFGO.Graph.AutoDiffGPMotionPriorFactor", node);
    graphBaseParamPtr_->AutoDiffGPMotionPriorFactor = AutoDiffGPMotionPriorFactor.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "AutoDiffGPMotionPriorFactor: "
      << (graphBaseParamPtr_->AutoDiffGPMotionPriorFactor ? "true" : "false"));

    RosParameter<bool> GPInterpolatedFactorCalcJacobian("GNSSFGO.Graph.GPInterpolatedFactorCalcJacobian", node);
    graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian = GPInterpolatedFactorCalcJacobian.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "GPInterpolatedFactorCalcJacobian: "
      << (graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian ? "true" : "false"));

    // PARAM: result handling
    RosParameter<bool> useEstimatedVarianceAfterInit("GNSSFGO.Graph.useEstimatedVarianceAfterInit", node);
    graphBaseParamPtr_->useEstimatedVarianceAfterInit = useEstimatedVarianceAfterInit.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "useEstimatedVarianceAfterInit: "
      << (graphBaseParamPtr_->useEstimatedVarianceAfterInit ? "true" : "false"));

    // ToDo: @Haoming, move this to TC GNSS Integrator
    RosParameter<bool> addConstDriftFactor("GNSSFGO.Graph.addConstDriftFactor", node);
    graphBaseParamPtr_->addConstDriftFactor = addConstDriftFactor.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(),
                       "addConstDriftFactor: " << (graphBaseParamPtr_->addConstDriftFactor ? "true" : "false"));
    RosParameter<std::string> noiseModelClockFactor("GNSSFGO.Graph.noiseModelClockFactor", node);
    integrator::IntegratorBase::setNoiseModelFromParam(noiseModelClockFactor.value(),
                                                       graphBaseParamPtr_->noiseModelClockFactor,
                                                       "ClockFactor");
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "clockDriftNoiseModel: " << noiseModelClockFactor.value());
    RosParameter<double> robustParamClockFactor("GNSSFGO.Graph.robustParamClockFactor", node);
    graphBaseParamPtr_->robustParamClockFactor = robustParamClockFactor.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "robustParamClockFactor: " << graphBaseParamPtr_->robustParamClockFactor);
    RosParameter<double> constDriftStd("GNSSFGO.Graph.constDriftStd", 1, node);
    graphBaseParamPtr_->constDriftStd = constDriftStd.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "constDriftStd: " << graphBaseParamPtr_->constDriftStd);

    // PARAM: Factors
    RosParameter<bool> addConstantAccelerationFactor("GNSSFGO.Graph.addConstantAccelerationFactor", false, node);
    graphBaseParamPtr_->addConstantAccelerationFactor = addConstantAccelerationFactor.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "addConstantAccelerationFactor: "
      << (graphBaseParamPtr_->addConstantAccelerationFactor ? "true" : "false"));

    RosParameter<double> angularRateStd("GNSSFGO.Graph.angularRateStd", 1, node);
    graphBaseParamPtr_->angularRateStd = angularRateStd.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "angularRateStd: " << graphBaseParamPtr_->angularRateStd);

    RosParameter<double> constBiasStd("GNSSFGO.Graph.constBiasStd", 1, node);
    graphBaseParamPtr_->constBiasStd = constBiasStd.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "constBiasStd: " << graphBaseParamPtr_->constBiasStd);

    RosParameter<double> motionModelStd("GNSSFGO.Graph.motionModelStd", 1, node);
    graphBaseParamPtr_->motionModelStd = motionModelStd.value();

    RosParameter<double> magnetometerStd("GNSSFGO.Graph.magnetometerStd", 1, node);
    graphBaseParamPtr_->magnetometerStd = gtsam::Vector3(magnetometerStd.value(), magnetometerStd.value(),
                                                         magnetometerStd.value());

    RosParameter<double> StateMeasSyncUpperBound("GNSSFGO.Graph.StateMeasSyncUpperBound", 0.02, node);
    graphBaseParamPtr_->StateMeasSyncUpperBound = StateMeasSyncUpperBound.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(),
                       "StateMeasSyncUpperBound: " << graphBaseParamPtr_->StateMeasSyncUpperBound);

    RosParameter<double> StateMeasSyncLowerBound("GNSSFGO.Graph.StateMeasSyncLowerBound", -0.02, node);
    graphBaseParamPtr_->StateMeasSyncLowerBound = StateMeasSyncLowerBound.value();
    RCLCPP_INFO_STREAM(appPtr_->get_logger(),
                       "StateMeasSyncLowerBound: " << graphBaseParamPtr_->StateMeasSyncLowerBound);

    // PARAM: strategies for no Optimization
    RosParameter<bool> NoOptimizationWhileNoMeasurement("GNSSFGO.Graph.NoOptimizationWhileNoMeasurement", node);
    graphBaseParamPtr_->NoOptimizationWhileNoMeasurement = NoOptimizationWhileNoMeasurement.value();
    RosParameter<bool> NoOptimizationNearZeroVelocity("GNSSFGO.Graph.NoOptimizationNearZeroVelocity", node);
    graphBaseParamPtr_->NoOptimizationNearZeroVelocity = NoOptimizationNearZeroVelocity.value();
    RosParameter<double> VoteNearZeroVelocity("GNSSFGO.Graph.VoteNearZeroVelocity", 0.5, node);
    graphBaseParamPtr_->VoteNearZeroVelocity = VoteNearZeroVelocity.value();
    RosParameter<int> NoOptimizationAfterStates("GNSSFGO.Graph.NoOptimizationAfterStates", node);
    graphBaseParamPtr_->NoOptimizationAfterStates = NoOptimizationAfterStates.value();

    // PARAM: Initializing integrators
    RosParameter<std::vector<std::string>> integratorNames("GNSSFGO.Integrators", node);

    bool primarySensorSet = false;
    for (const auto &integratorName: integratorNames.value()) {
      RosParameter<std::string> thisIntegratorPlugin("GNSSFGO." + integratorName + ".integratorPlugin", "", node);
      const auto plugin = thisIntegratorPlugin.value();
      if (integratorLoader_.isClassAvailable(plugin)) {
        RCLCPP_INFO_STREAM(appPtr_->get_logger(),
                           "GraphTimeCentric: initializing integrator: " << integratorName << " with plugin: "
                                                                         << plugin);
        auto integrator = integratorLoader_.createSharedInstance(plugin);
        integrator->initialize(node, *this, integratorName);
        integratorMap_.insert(std::make_pair(integratorName, integrator));
        RosParameter<bool> isPrimarySensor("GNSSFGO." + integratorName + ".isPrimarySensor", false, node);
        if (isPrimarySensor.value()) {
          if (primarySensorSet)
            RCLCPP_WARN_STREAM(appPtr_->get_logger(),
                               "GraphTimeCentric: primary sensor already set, reset to: " << integratorName
                                                                                          << " with plugin: "
                                                                                          << plugin);
          primarySensor_ = integrator;
          primarySensorSet = true;

        }
      } else {
        RCLCPP_WARN_STREAM(appPtr_->get_logger(),
                           "GraphTimeCentric: cannot find integrator: " << integratorName << " with plugin: "
                                                                        << plugin);
      }
    }

    RCLCPP_INFO(appPtr_->get_logger(), "---------------------  GraphBase initialized! --------------------- ");
  };

  void GraphBase::calculateResiduals(const rclcpp::Time &timestamp, const gtsam::Values &result,
                                     const gtsam::Marginals &marginals) {
    uint64_t maxStateIndex = 0;
    std::for_each(result.keys().begin(), result.keys().end(), [&](const gtsam::Key &key) -> void {
      const auto keyIndex = gtsam::symbolIndex(key);
      if (keyIndex > maxStateIndex)
        maxStateIndex = keyIndex;
    });
    const auto factorBuffer = factorBuffer_.get_all_time_buffer_pair();
    for (const auto &factorVectorTimePair: factorBuffer) {
      if (factorVectorTimePair.first > timestamp)
        continue;
      irt_nav_msgs::msg::FactorResiduals resMsg;
      resMsg.header.stamp = timestamp;
      const auto &factorVec = factorVectorTimePair.second;
      std::for_each(factorVec.begin(), factorVec.end(), [&](const gtsam::NonlinearFactor::shared_ptr &factor) -> void {
        bool sampleResiduals = false;
        bool factorImplemented = true;
        gtsam::NoiseModelFactor::shared_ptr thisFactorCasted;
        const auto factorTypeID = factor->getTypeID();
        if (std::find(this->graphBaseParamPtr_->skippedFactorsForResiduals.begin(),
                      this->graphBaseParamPtr_->skippedFactorsForResiduals.end(), factorTypeID) !=
            this->graphBaseParamPtr_->skippedFactorsForResiduals.end()) {
          return;
        }
        switch (factorTypeID) {
          case fgo::factor::FactorTypeID::GPWNOAMotionPrior: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPWNOAPriorPose3>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPWNOJMotionPrior: {
            sampleResiduals = true;
            if (graphBaseParamPtr_->fullGPPrior)
              thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPWNOJPriorPose3Full>(factor);
            else
              thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPWNOJPriorPose3>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPSingerMotionPrior: {
            sampleResiduals = true;
            if (graphBaseParamPtr_->fullGPPrior)
              thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPSingerPriorPose3Full>(factor);
            else
              thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPSingerPriorPose3>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::CombinedIMU: {
            thisFactorCasted = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::BetweenPose: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::BetweenPose3Factor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPDoubleBetweenPose: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedDoublePose3BetweenFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPSingleBetweenPose: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedSinglePose3BetweenFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::ReceiverClock: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::ConstDriftFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::PRDR: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::PrDrFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPPRDR: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedPrDrFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::PR: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::PrFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPPR: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedPrFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPS: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPSFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPGPS: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedGPSFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::PVT: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::PVTFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPPVT: {
            sampleResiduals = true;
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedPVTFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::NavAttitude: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::NavAttitudeFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPNavAttitude: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedNavAttitudeFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::NavVelocity: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::NavVelocityFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPNavVelocity: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedNavVelocityFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::NavPose: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::NavPoseFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::GPNavPose: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPInterpolatedNavPoseFactor>(factor);
            break;
          }
          case fgo::factor::FactorTypeID::ConstAngularVelocity: {
            thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::ConstAngularRateFactor>(factor);
            break;
          }
          default: {
            RCLCPP_ERROR_STREAM(appPtr_->get_logger(),
                                "onResidualPublishing factor " << factor->getName() << " not implemented");
            factorImplemented = false;
            break;
          }
        }

        if (thisFactorCasted && factorImplemented) {
          irt_nav_msgs::msg::FactorResidual res;
          res.current_state_key = maxStateIndex;
          res.factor_name = thisFactorCasted->getName();
          gtsam::Values values;
          const gtsam::KeyVector keys = thisFactorCasted->keys();

          bool allKeyValued = true;
          std::for_each(keys.cbegin(), keys.cend(), [&](size_t key) -> void {
            if (result.exists(key))
              values.insert(key, result.at(key));
            else
              allKeyValued = false;
          });

          std::transform(keys.begin(), keys.end(), std::back_inserter(res.related_keys),
                         [](const gtsam::Key &key) -> std::string {
                           return gtsam::DefaultKeyFormatter(key);
                         });

          res.current_state_key = gtsam::symbolIndex(keys.back());

          if (sampleResiduals) {
            const auto keyPreOrdered = keys;
            const auto [keyOrdered, keyIndicesAfterSorting] = fgo::utils::sortedKeyIndexes(keyPreOrdered);
            // we calculate the joint information matrix of all related keys
            const auto jointMarginalCovariance = marginals.jointMarginalCovariance(keys);
            const auto jointCovNotOrdered = fgo::utils::rebuildJointMarginalMatrix(jointMarginalCovariance,
                                                                                   keyOrdered,
                                                                                   keyIndicesAfterSorting);
            // and use this matrix to create a noise model. When using the noise model, the information matrix has been already trangularized using e.g., cholesky
            const auto mean = thisFactorCasted->liftValuesAsVector(values);
            auto sampleParam = std::make_shared<fgo::data::sampler::SamplerConfig>();
            auto residualSampler = boost::make_shared<fgo::data::sampler::UnscentedSampler>(sampleParam, mean,
                                                                                            jointCovNotOrdered);
            const auto sigma_points = residualSampler->samples();

            for (size_t i = 0; i < sigma_points.rows(); i++) {
              irt_nav_msgs::msg::ResidualSample sample;
              sample.id = i;
              if (i == 0)
                sample.type = sample.ESTIMATE;
              else
                sample.type = sample.SAMPLED_FROM_ESTIMATE;

              const auto state = sigma_points.block(i, 0, 1, mean.size()).transpose();
              gtsam::Values sampledValues = thisFactorCasted->generateValuesFromStateVector(state);

              const auto unwhitenedError = thisFactorCasted->unwhitenedError(sampledValues);
              const auto whitenedError = thisFactorCasted->whitenedError(sampledValues);
              sample.unwhitened_error.resize(unwhitenedError.size());
              gtsam::Vector::Map(&sample.unwhitened_error[0], unwhitenedError.size()) = unwhitenedError;

              sample.whitened_error.resize(whitenedError.size());
              gtsam::Vector::Map(&sample.whitened_error[0], whitenedError.size()) = whitenedError;

              sample.noise_model_weight = thisFactorCasted->weight(sampledValues);
              sample.loss_error = thisFactorCasted->error(sampledValues);
              res.samples.emplace_back(sample);
            }
          } else {
            irt_nav_msgs::msg::ResidualSample sample;
            sample.id = 0;
            sample.type = sample.ESTIMATE;
            const auto unwhitenedError = thisFactorCasted->unwhitenedError(values);
            const auto whitenedError = thisFactorCasted->whitenedError(values);
            sample.unwhitened_error.resize(unwhitenedError.size());
            gtsam::Vector::Map(&sample.unwhitened_error[0], unwhitenedError.size()) = unwhitenedError;

            sample.whitened_error.resize(whitenedError.size());
            gtsam::Vector::Map(&sample.whitened_error[0], whitenedError.size()) = whitenedError;

            sample.noise_model_weight = thisFactorCasted->weight(values);
            sample.loss_error = thisFactorCasted->error(values);
            res.samples.emplace_back(sample);
          }
          resMsg.residuals.emplace_back(res);
        }
      });
      pubResiduals_->publish(resMsg);
    }
  };

  void GraphBase::initGraph(const data::State &initState, const double &initTimestamp,
                            boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams) {
    isStateInited_ = false;
    preIntegratorParams_ = preIntegratorParams;

    this->resetGraph();

    auto priorPoseFactor = gtsam::PriorFactor<gtsam::Pose3>(X(0), initState.state.pose(), initState.poseVar);
    priorPoseFactor.setTypeID(999);
    priorPoseFactor.setName("priorPoseFactor");
    this->push_back(priorPoseFactor);
    auto priorVelFactor = gtsam::PriorFactor<gtsam::Vector3>(V(0), initState.state.v(), initState.velVar);
    priorVelFactor.setTypeID(999);
    priorVelFactor.setName("priorVelFactor");
    this->push_back(priorVelFactor);
    auto priorBiasFactor = gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), initState.imuBias,
                                                                            initState.imuBiasVar);
    priorBiasFactor.setTypeID(999);
    priorBiasFactor.setName("priorBiasFactor");
    this->push_back(priorBiasFactor);

    values_.insert(X(0), initState.state.pose());
    values_.insert(V(0), initState.state.v());
    values_.insert(B(0), initState.imuBias);

    keyTimestampMap_[X(0)] =
    keyTimestampMap_[V(0)] =
    keyTimestampMap_[B(0)] = initTimestamp;

    // ToDo:  @haoming, move this to TC GNSS integrator
    if (graphBaseParamPtr_->addConstDriftFactor) {
      auto priorClockErrorFactor = gtsam::PriorFactor<gtsam::Vector2>(C(0), initState.cbd, initState.cbdVar);
      priorClockErrorFactor.setTypeID(999);
      priorClockErrorFactor.setName("priorClockErrorFactor");
      this->push_back(priorClockErrorFactor);
      values_.insert(C(0), initState.cbd);
      keyTimestampMap_[C(0)] = initTimestamp;
    }

    currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, initTimestamp));

    if (graphBaseParamPtr_->addGPPriorFactor || graphBaseParamPtr_->addGPInterpolatedFactor) {
      auto priorOmegaFactor = gtsam::PriorFactor<gtsam::Vector3>(W(0), initState.omega, initState.omegaVar);
      priorOmegaFactor.setTypeID(999);
      priorOmegaFactor.setName("priorOmegaFactor");
      this->push_back(priorOmegaFactor);
      values_.insert(W(0), initState.omega);
      keyTimestampMap_[W(0)] = initTimestamp;

      if (graphBaseParamPtr_->fullGPPrior) {
        auto priorAccFactor = gtsam::PriorFactor<gtsam::Vector6>(A(0), initState.accMeasured,
                                                                 initState.accVar);
        priorAccFactor.setTypeID(999);
        priorAccFactor.setName("priorAccFactor");
        this->push_back(priorAccFactor);
        values_.insert(A(0), gtsam::Vector6(initState.accMeasured));
        keyTimestampMap_[A(0)] = initTimestamp;
      }
    }

    if (!graphBaseParamPtr_->addGPPriorFactor && graphBaseParamPtr_->addGPInterpolatedFactor) {
      fgo::data::IMUMeasurement tmpIMU;
      tmpIMU.gyro = initState.omega;
      // this->addAngularFactor(W(0), B(0), tmpIMU);
    }

    for (const auto &integrator: integratorMap_) {
      integrator.second->dropMeasurementBefore(initTimestamp);
    }

    isStateInited_ = true;
    RCLCPP_WARN(appPtr_->get_logger(), "------------- GraphBase: Init.FGO Done! -------------");
  }

  void GraphBase::notifyOptimization() {
    appPtr_->notifyOptimization();
  }


}
