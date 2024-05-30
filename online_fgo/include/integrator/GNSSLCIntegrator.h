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

#ifndef ONLINE_FGO_INTEGRATEGNSSLC_H
#define ONLINE_FGO_INTEGRATEGNSSLC_H

#pragma once

#include <irt_nav_msgs/msg/pps.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <irt_nav_msgs/msg/fgo_state.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <ublox_msgs/msg/nav_clock.hpp>

#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <novatel_oem7_msgs/msg/bestvel.hpp>
#include <novatel_oem7_msgs/msg/dualantennaheading.hpp>
#include <novatel_oem7_msgs/msg/clockmodel.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <novatel_oem7_msgs/msg/inscov.hpp>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include "IntegratorBase.h"

#include "factor/gnss/GPSFactor.h"
#include "factor/gnss/GPInterpolatedGPSFactor.h"
#include "factor/gnss/PVTFactor.h"
#include "factor/gnss/GPInterpolatedPVTFactor.h"

#include "sensor/GNSS/GNSSDataParser.h"

#include "utils/GNSSUtils.h"
#include "CalculateMeasurementDelay_ert_rtw/CalculateMeasurementDelay.h"

#include "third_party/matlab_utils.h"

namespace fgo::integrator {
  using namespace ::utils;

  class GNSSLCIntegrator : public IntegratorBase {
    IntegratorGNSSLCParamsPtr paramPtr_;
    std::shared_ptr<fgo::models::GPInterpolator> interpolator_;

    fgo::data::CircularDataBuffer<fgo::data::PVASolution> GNSSPVABuffer_;
    fgo::data::CircularDataBuffer<fgo::data::PVASolution> referencePVTBuffer_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subNavfix_;

    typedef message_filters::sync_policies::ApproximateTime<novatel_oem7_msgs::msg::BESTPOS, novatel_oem7_msgs::msg::BESTVEL,
      novatel_oem7_msgs::msg::DUALANTENNAHEADING> OEM7DualAntennaSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<novatel_oem7_msgs::msg::BESTPOS, novatel_oem7_msgs::msg::BESTVEL> OEM7SyncPolicy;

    message_filters::Subscriber<ublox_msgs::msg::NavPVT> subUbloxNavPVT_;
    message_filters::Subscriber<novatel_oem7_msgs::msg::BESTPOS> subNovatelBestpos_;
    message_filters::Subscriber<novatel_oem7_msgs::msg::BESTVEL> subNovatelBestvel_;
    message_filters::Subscriber<novatel_oem7_msgs::msg::DUALANTENNAHEADING> subNovatelHeading_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subNovatelBestposAlone_;
    rclcpp::Publisher<irt_nav_msgs::msg::FGOState>::SharedPtr pubPVAInFGOStata_;

    rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr subUbloxPVT_;
    std::unique_ptr<message_filters::Synchronizer<OEM7DualAntennaSyncPolicy>> novatelPVTDualAntennaSync_;
    std::unique_ptr<message_filters::Synchronizer<OEM7SyncPolicy>> novatelPVTSync_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subPVAOdom_;

    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr subNovatelPVA_;
    rclcpp::Subscription<irt_nav_msgs::msg::PVAGeodetic>::SharedPtr subPVA_;
    rclcpp::Subscription<irt_nav_msgs::msg::PPS>::SharedPtr subPPS_;
    std::unique_ptr<fgo::utils::MeasurementDelayCalculator> PVTDelayCalculator_;

    std::atomic_bool zeroVelocity_{};

  public:
    explicit GNSSLCIntegrator() = default;

    ~GNSSLCIntegrator() override = default;

    //Todo: @haoming this is ugly!
    fgo::integrator::param::IntegratorGNSSLCParamsPtr getIntegratorParamPtr() { return paramPtr_; };

    void initialize(rclcpp::Node &node, fgo::graph::GraphBase &graphPtr, const std::string &integratorName,
                    bool isPrimarySensor = false) override;

    bool addFactors(const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
                    const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> &stateIDAccMap,
                    const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
                    std::vector<std::pair<rclcpp::Time, fgo::data::State>> &timePredStates,
                    gtsam::Values &values,
                    fgo::solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
                    gtsam::KeyVector &relatedKeys) override;

    bool fetchResult(
      const gtsam::Values &result,
      const gtsam::Marginals &martinals,
      const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
      fgo::data::State &optState
    ) override;

    void dropMeasurementBefore(double timestamp) override {
      GNSSPVABuffer_.cleanBeforeTime(timestamp);
    }

    std::vector<fgo::data::PVASolution> getPVAData() {
      return GNSSPVABuffer_.get_all_buffer();
    }

    std::vector<fgo::data::PVASolution> getPVADataAndClean() {
      return GNSSPVABuffer_.get_all_buffer_and_clean();
    }

    bool checkZeroVelocity() override {
      return zeroVelocity_;
    }

    void cleanBuffers() override {
      const auto buffer = GNSSPVABuffer_.get_all_buffer_and_clean();
    }

    bool checkHasMeasurements() override {
      return referencePVTBuffer_.size() != 0;
    }


    void feedRAWData(data::PVASolution &pva,
                     data::State &state) {
      const auto thisPVATime = pva.timestamp;
      static double lastDelay = 0.;
      static auto lastPVATime = thisPVATime;
      static bool first_measurement = true;
      static size_t calcZeroVelocityCounter = 1;
      static gtsam::Vector3 sumVelocity = gtsam::Z_3x1;

      irt_nav_msgs::msg::SensorProcessingReport thisProcessingReport;
      thisProcessingReport.ts_measurement = thisPVATime.seconds();
      const auto ts_start_processing = rosNodePtr_->now();
      thisProcessingReport.ts_start_processing = ts_start_processing.seconds();
      thisProcessingReport.sensor_name = "GNSSLC";
      thisProcessingReport.observation_available = true;

      //sensor_msgs::msg::NavSatFix pvtNavMsg;
      //pvtNavMsg.header.stamp = thisPVTTime;
      //pvtNavMsg.latitude = pvtMsg->phi_geo * fgo::constants::rad2deg;
      //pvtNavMsg.longitude = pvtMsg->lambda_geo * fgo::constants::rad2deg;
      //pvtNavMsg.altitude= pvtMsg->h_geo;
      //pvtTestPub_->publish(pvtNavMsg);


      sumVelocity += pva.vel_n;
      //rclcpp::sleep_for(std::chrono::nanoseconds(1000000));  // 10000000
      auto pvtDelay = 0; //this->PVTDelayCalculator_->getDelay()  + paramPtr_->pvtMeasTimeOffset;
      auto delayFromMsg = (thisPVATime - lastPVATime).seconds() - 0.1;

      if (first_measurement) {
        first_measurement = false;
        delayFromMsg = 0.;
        pvtDelay = 0.;
      }

      if (delayFromMsg < -0.005 && lastDelay > 0)
        delayFromMsg += lastDelay;

      if (delayFromMsg < 0.)
        delayFromMsg = 0.;
      //std::cout << std::fixed << "pvt delay from msg: " << delayFromMsg << std::endl;

      double pvtTimeCorrected = thisPVATime.seconds();
      if (!paramPtr_->delayFromPPS) {
        pvtTimeCorrected -= delayFromMsg;
        pva.delay = delayFromMsg;
      } else {
        pvtTimeCorrected -= pvtDelay;
        pva.delay = pvtDelay;
      }

      state.timestamp = rclcpp::Time(pvtTimeCorrected * fgo::constants::sec2nanosec, RCL_ROS_TIME);
      graphPtr_->updateReferenceState(state, state.timestamp);
      irt_nav_msgs::msg::FGOState state_msg;
      state_msg.header.frame_id = "antmain";
      state_msg.header.stamp = state.timestamp;
      state_msg.amb_var.emplace_back(pva.tow);
      state_msg.amb_var.emplace_back(thisPVATime.seconds());
      state_msg.pose.position.x = state.state.t().x();
      state_msg.pose.position.y = state.state.t().y();
      state_msg.pose.position.z = state.state.t().z();
      state_msg.vel.linear.x = state.state.v().x();
      state_msg.vel.linear.y = state.state.v().y();
      state_msg.vel.linear.z = state.state.v().z();
      state_msg.vel.angular.x = state.omega.x();
      state_msg.vel.angular.y = state.omega.y();
      state_msg.vel.angular.z = state.omega.z();
      state_msg.cbd.resize(state.cbd.size());
      state_msg.cog = pva.heading;
      gtsam::Vector::Map(&state_msg.cbd[0], state.cbd.size()) = state.cbd;
      pubPVAInFGOStata_->publish(state_msg);
      GNSSPVABuffer_.update_buffer(pva, pva.timestamp);

      if (paramPtr_->NoOptimizationNearZeroVelocity) {
        if (calcZeroVelocityCounter > 6) {
          const auto avgVelocity = (gtsam::Vector3() << sumVelocity.x() / calcZeroVelocityCounter,
            sumVelocity.y() / calcZeroVelocityCounter,
            sumVelocity.z() / calcZeroVelocityCounter).finished();
          RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ << " avg: velocity " << avgVelocity.norm());
          if (avgVelocity.norm() < paramPtr_->zeroVelocityThreshold) {
            zeroVelocity_ = true;
            GNSSPVABuffer_.clean();
          } else
            //zeroVelocity_ = false;
            calcZeroVelocityCounter = 1;
          sumVelocity.setZero();
        }
      }
      calcZeroVelocityCounter++;

      if (abs(delayFromMsg) < 0.005 || delayFromMsg > 0.3)
        lastDelay = 0.;
      else
        lastDelay = delayFromMsg;

      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(pva.tow, state.timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onIRTPVTMsgCb: graph not initialized, waiting ...");
      }

      //std::cout << std::fixed << "pvt lsat delay : " << delayFromMsg << std::endl;
      thisProcessingReport.measurement_delay = pva.delay;
      thisProcessingReport.header.stamp = rosNodePtr_->now();
      thisProcessingReport.duration_processing = (rosNodePtr_->now() - ts_start_processing).seconds();
      thisProcessingReport.num_measurements = 1;
      if (pubSensorReport_)
        pubSensorReport_->publish(thisProcessingReport);

      lastPVATime = thisPVATime;
    }

    void feedRAWData(std::vector<data::PVASolution> &pva_vec,
                     std::vector<data::State> &state_vec) {
      if (!state_vec.empty())
        assert(pva_vec.size() != state_vec.size());
      for (size_t i = 0; i < pva_vec.size(); i++) {
        feedRAWData(pva_vec[i], state_vec[i]);
      }
    }

  protected:

    void addGNSSFactor(const gtsam::Key &poseKey,
                       const gtsam::Point3 &posMeasured,
                       const gtsam::Vector3 &posVar,
                       const gtsam::Vector3 &lb) {
      const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                      posVar, paramPtr_->robustParamPosition, "GPS");
      graphPtr_->emplace_shared<fgo::factor::GPSFactor>(poseKey, posMeasured, lb, noiseModel,
                                                        paramPtr_->AutoDiffNormalFactor);
    }

    void addGPInterpolatedGNSSFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                                     const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                                     const gtsam::Point3 &posMeasured, const gtsam::Vector3 &posVar,
                                     const gtsam::Vector3 &lb,
                                     const std::shared_ptr<fgo::models::GPInterpolator> &interpolator) {
      const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                      posVar, paramPtr_->robustParamPosition, "GPInterpolatedGPS");
      graphPtr_->emplace_shared<fgo::factor::GPInterpolatedGPSFactor>(poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ,
                                                                      omegaKeyJ, posMeasured,
                                                                      lb, noiseModel, interpolator, false);
    }

    void addGNSSPVTFactor(const gtsam::Key &poseKey, const gtsam::Key &velKey, const gtsam::Key &omegaKey,
                          const gtsam::Point3 &posMeasured, const gtsam::Vector3 &velMeasured,
                          const gtsam::Vector3 &posVar, const gtsam::Vector3 &velVar, const gtsam::Vector3 &lb) {
      const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                      (gtsam::Vector6() << posVar, velVar).finished(),
                                                      paramPtr_->robustParamPosition);
      graphPtr_->emplace_shared<fgo::factor::PVTFactor>(poseKey, velKey, omegaKey,
                                                        posMeasured, velMeasured,
                                                        lb, paramPtr_->velocityFrame, noiseModel, false);
    }

    void
    addGPInterpolatedGNSSPVTFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                                   const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                                   const gtsam::Point3 &posMeasured, const gtsam::Vector3 &velMeasured,
                                   const gtsam::Vector3 &posVar, const gtsam::Vector3 &velVar, const gtsam::Vector3 &lb,
                                   const std::shared_ptr<fgo::models::GPInterpolator> &interpolator) {
      const auto noiseModel = graph::assignNoiseModel(paramPtr_->noiseModelPosition,
                                                      (gtsam::Vector6() << posVar, velVar).finished(),
                                                      paramPtr_->robustParamPosition);

      graphPtr_->emplace_shared<fgo::factor::GPInterpolatedPVTFactor>(poseKeyI, velKeyI, omegaKeyI,
                                                                      poseKeyJ, velKeyJ, omegaKeyJ,
                                                                      posMeasured, velMeasured, lb,
                                                                      paramPtr_->velocityFrame,
                                                                      noiseModel, interpolator,
                                                                      paramPtr_->AutoDiffGPInterpolatedFactor);
    }

  private:

    void onOdomMsgCb(const nav_msgs::msg::Odometry::ConstSharedPtr pva) {
      // ATTENTION: here we use ENU as tangent frame
      rclcpp::Time ts = rclcpp::Time(pva->header.stamp.sec, pva->header.stamp.nanosec, RCL_ROS_TIME);

      const auto [this_pva, state] = sensor::GNSS::parseOdomMsg(*pva, paramPtr_, ts);
      GNSSPVABuffer_.update_buffer(this_pva, ts);

      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(this_pva.tow, this_pva.timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onOdomMsgCb: graph not initialized, waiting ...");
      }

    }

    void onIRTPVTMsgCb(const irt_nav_msgs::msg::PVAGeodetic::ConstSharedPtr pvaMsg) {
      if (pvaMsg->sol_age > 0.15) {
        RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(),
                            integratorName_ << " onIRTPVTMsgCb solution out of date: " << pvaMsg->sol_age);
        return;
      }
      PVTDelayCalculator_->setTOW(pvaMsg->tow);
      auto [pva, fgoState] = sensor::GNSS::parseIRTPVAMsg(*pvaMsg, paramPtr_);
      this->feedRAWData(pva, fgoState);
    }


    void onINSPVAXMsgCb(const novatel_oem7_msgs::msg::INSPVAX::ConstSharedPtr pva) {
      rclcpp::Time msg_timestamp;
      if (paramPtr_->useHeaderTimestamp)
        msg_timestamp = rclcpp::Time(pva->header.stamp.sec, pva->header.stamp.nanosec, RCL_ROS_TIME);
      else
        msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);


      const auto [sol, state] = sensor::GNSS::parseINSPVAXMsg(*pva,
                                                              paramPtr_,
                                                              msg_timestamp);
      GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onINSPVAXMsgCb: graph not initialized, waiting ...");
      }
    }

    void onOEM7PVTHeadingMsgCb(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos,
                               const novatel_oem7_msgs::msg::BESTVEL::ConstSharedPtr bestvel,
                               const novatel_oem7_msgs::msg::DUALANTENNAHEADING::ConstSharedPtr heading) {
      const auto msg_timestamp = rclcpp::Time(bestpos->header.stamp.sec, bestpos->header.stamp.nanosec, RCL_ROS_TIME);
      fgo::data::PVASolution sol{};
      sol.timestamp = msg_timestamp;
      sol.tow = bestpos->nov_header.gps_week_milliseconds * 0.001;
      sol.has_heading = true;
      sol.has_velocity = true;
      sol.llh = (gtsam::Vector3() << bestpos->lat * fgo::constants::deg2rad, bestpos->lon *
                                                                             fgo::constants::deg2rad, bestpos->hgt).finished();
      sol.xyz_ecef = fgo::utils::llh2xyz(sol.llh);
      sol.xyz_var = (gtsam::Vector3() << std::pow(bestpos->lat_stdev, 2), std::pow(bestpos->lon_stdev, 2), std::pow(
        bestpos->hgt_stdev, 2)).finished() * paramPtr_->posVarScale;
      const auto ecef_R_ned = gtsam::Rot3(fgo::utils::nedRe_Matrix(sol.xyz_ecef).inverse());
      sol.vel_n = (gtsam::Vector3() << bestvel->hor_speed * std::cos(bestvel->trk_gnd * fgo::constants::deg2rad),
        bestvel->hor_speed * std::sin(bestvel->trk_gnd * fgo::constants::deg2rad),
        -bestvel->ver_speed).finished();
      sol.vel_ecef = ecef_R_ned.rotate(sol.vel_n);
      sol.vel_var =
        (gtsam::Vector3() << paramPtr_->fixedVelVar, paramPtr_->fixedVelVar, paramPtr_->fixedVelVar).finished() *
        paramPtr_->velVarScale;
      sol.heading = heading->heading * fgo::constants::deg2rad;
      sol.heading_var = std::pow(heading->heading_std_dev * fgo::constants::deg2rad, 2) * paramPtr_->headingVarScale;
      auto heading_rot = gtsam::Rot3::Yaw(sol.heading);
      sol.heading_ecef = ecef_R_ned.compose(heading_rot).yaw();
      sol.roll_pitch = heading->pitch * fgo::constants::deg2rad;
      sol.roll_pitch_var = std::pow(heading->pitch_std_dev * fgo::constants::deg2rad, 2) * paramPtr_->headingVarScale;

      if (paramPtr_->hasRoll) {
        sol.rot = gtsam::Rot3::Ypr(sol.heading, 0., sol.roll_pitch);
        sol.rot_var = (gtsam::Vector3() << sol.roll_pitch_var, 0., sol.heading_var).finished();
      } else if (paramPtr_->hasPitch) {
        sol.rot = gtsam::Rot3::Ypr(sol.heading, sol.roll_pitch, 0.);
        sol.rot_var = (gtsam::Vector3() << 0, sol.roll_pitch_var, sol.heading_var).finished();
      } else {
        sol.rot = gtsam::Rot3::Yaw(sol.heading);
        sol.rot_var = (gtsam::Vector3() << 0, 0., sol.heading_var).finished();
      }

      sol.type = fgo::utils::GNSS::getOEM7PVTSolutionType(bestpos->pos_type.type);
      sol.num_sat = bestpos->num_sol_svs;
      GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onOEM7PVTHeadingMsgCb: graph not initialized, waiting ...");
      }
    }

    void onOEM7PVTMsgCb(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos,
                        const novatel_oem7_msgs::msg::BESTVEL::ConstSharedPtr bestvel) {
      rclcpp::Time msg_timestamp;
      if (paramPtr_->useHeaderTimestamp)
        msg_timestamp = rclcpp::Time(bestpos->header.stamp.sec, bestpos->header.stamp.nanosec, RCL_ROS_TIME);
      else
        msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);

      const auto [sol, state] = sensor::GNSS::parseNovAtelPVT(*bestpos, *bestvel,
                                                              paramPtr_,
                                                              msg_timestamp);
      GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onOEM7PVTMsgCb: graph not initialized, waiting ...");
      }
    }

    void onOEM7Bestpos(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr bestpos) {
      rclcpp::Time msg_timestamp;
      if (paramPtr_->useHeaderTimestamp)
        msg_timestamp = rclcpp::Time(bestpos->header.stamp.sec, bestpos->header.stamp.nanosec, RCL_ROS_TIME);
      else
        msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);
      const auto [sol, state] = sensor::GNSS::parseNovAtelBestpos(*bestpos, paramPtr_, msg_timestamp);
      GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onOEM7Bestpos: graph not initialized, waiting ...");
      }
    }

    void onUbloxPVTMsgCb(const ublox_msgs::msg::NavPVT::ConstSharedPtr navpvt) {

      static size_t calcZeroVelocityCounter = 1;
      static gtsam::Vector3 sumVelocity = gtsam::Z_3x1;
      const auto now = rosNodePtr_->now();
      rclcpp::Time msg_timestamp = rclcpp::Time(now.nanoseconds(), RCL_ROS_TIME);
      RCLCPP_INFO_STREAM(rosNodePtr_->get_logger(), "on ubloxPVT at " << std::fixed << msg_timestamp.seconds());

      const auto [sol, state] = sensor::GNSS::parseUbloxPVTMsg(*navpvt,
                                                               paramPtr_,
                                                               msg_timestamp);
      sumVelocity += sol.vel_n;
      GNSSPVABuffer_.update_buffer(sol, msg_timestamp);

      if (calcZeroVelocityCounter > 6) {
        const auto avgVelocity = (gtsam::Vector3() << sumVelocity.x() / calcZeroVelocityCounter,
          sumVelocity.y() / calcZeroVelocityCounter,
          sumVelocity.z() / calcZeroVelocityCounter).finished();
        //RCLCPP_ERROR_STREAM(appPtr_->get_logger(), integratorName_ << " avg: velocity " << avgVelocity.norm());
        if (avgVelocity.norm() < paramPtr_->zeroVelocityThreshold) {
          //RCLCPP_ERROR_STREAM(rosNodePtr_->get_logger(), integratorName_ << " onIRTPVTMsgCb reported near zero velocity: " << sol.vel_n);
          zeroVelocity_ = true;
          GNSSPVABuffer_.clean();
        } else
          zeroVelocity_ = false;
        calcZeroVelocityCounter = 0;

        sumVelocity.setZero();
      }

      calcZeroVelocityCounter++;

      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onUbloxPVTMsgCb: graph not initialized, waiting ...");
      }
    }

    void onNavFixMsgCb(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
      rclcpp::Time msg_timestamp;
      if (paramPtr_->useHeaderTimestamp)
        msg_timestamp = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
      else
        msg_timestamp = rclcpp::Time(rosNodePtr_->now(), RCL_ROS_TIME);
      const auto [sol, state] = sensor::GNSS::parseNavFixMsg(*msg, paramPtr_, msg_timestamp);
      GNSSPVABuffer_.update_buffer(sol, msg_timestamp);
      if (paramPtr_->useForInitialization && !graphPtr_->isGraphInitialized()) {
        graphPtr_->updateReferenceMeasurementTimestamp(sol.tow, msg_timestamp);
        RCLCPP_WARN(rosNodePtr_->get_logger(), "onNavFixMsgCb: graph not initialized, waiting ...");
      }
    }
  };
}

#endif //ONLINE_FGO_INTEGRATEGNSSLC_H
