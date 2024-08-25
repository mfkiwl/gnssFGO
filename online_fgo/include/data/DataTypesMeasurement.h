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
// Created by haoming on 08.06.24.
//

#ifndef ONLINE_FGO_DATATYPESMEASUREMENT_H
#define ONLINE_FGO_DATATYPESMEASUREMENT_H

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/NoiseModel.h>
#include "utils/Constants.h"

namespace fgo::data {

  /** Camera **/
  struct StereoPair
  {
    rclcpp::Time timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    cv_bridge::CvImagePtr left, right;
  };

  struct Image
  {
    rclcpp::Time timestamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    cv_bridge::CvImage image;
  };

  /** LiDARs **/
  //ToDo: @Haoming, define all lidar related data here



  /** IMU **/
  struct IMUMeasurement {
    rclcpp::Time timestamp{0};  // timestamp of current imu meas
    double dt{};  // dt between consequent meas
    gtsam::Rot3 AHRSOri{};
    gtsam::Matrix33 AHRSOriCov{};
    gtsam::Vector3 accLin{};
    gtsam::Matrix33 accLinCov{};
    gtsam::Vector3 accRot{};
    gtsam::Matrix33 accRotCov{};
    gtsam::Vector3 gyro{};
    gtsam::Matrix33 gyroCov{};
    gtsam::Vector3 mag{};
    gtsam::Matrix33 magCov{};
  };

  /** GNSS **/
  struct GNSSObs {
    uint32_t satId{};
    gtsam::Vector3 satPos{};
    gtsam::Vector3 satVel{};
    //gtsam::Vector3 refSatPos{};
    double pr{};
    double prVar{};
    double prVarRaw{};
    double dr{};
    double drVar{};
    double cp{};
    double cpVar{};
    double cpVarRaw{};
    double el{};
    double cn0{};
    //uint8_t stats{};
    double locktime{};
    //bool useCP = true;
    bool cycleSlip = true;
    //double timeLastSlip = -1.0;
    bool isLOS = true;
  };

  struct RefSat {
    uint8_t refSatSVID;
    gtsam::Vector3 refSatPos{};
    gtsam::Vector3 refSatVel{};
  };

  struct GNSSMeasurementEpoch {
    double tow{};
    rclcpp::Time timestamp{};
    double delay;
    std::vector<GNSSObs> obs{};
    double timeOffsetGALGPS{};
    bool isGGTOValid = false;
    uint8_t integrityFlag{};
    gtsam::Vector3 basePosRTCM{};
    RefSat refSatGPS{};
    RefSat refSatGAL{};
    uint8_t ddIDXSyncRef;
    uint8_t ddIDXSyncUser;
  };


  enum GNSSSolutionType {
    RTKFIX = 1,
    RTKFLOAT = 2,
    SINGLE = 3,
    NO_SOLUTION = 4
  };

  struct PVASolution {
    uint16_t wnc;
    double tow{};
    rclcpp::Time timestamp{};
    double delay = 0.;
    double sol_age = 0.;
    double diff_age = 0.;
    GNSSSolutionType type;
    uint8_t error;

    gtsam::Vector3 xyz_ecef;
    gtsam::Vector3 xyz_var;
    gtsam::Vector3 llh;
    gtsam::Vector3 vel_n;
    gtsam::Vector3 vel_ecef;
    gtsam::Vector3 vel_var;
    gtsam::Rot3 rot_n;
    gtsam::Rot3 rot_ecef;
    gtsam::Rot3 nRe;
    gtsam::Vector3 rot_var;
    double undulation;
    double heading;  // Todo: to be deleted
    double cog;  // Todo: to be deleted
    double heading_ecef; // Todo: to be deleted
    double heading_var; // Todo: to be deleted
    double roll_pitch; // Todo: to be deleted
    double roll_pitch_var; // Todo: to be deleted
    double trk_gnd;
    double clock_bias;
    double clock_drift;
    double clock_bias_var;
    double clock_drift_var;

    uint8_t num_sat;
    uint8_t num_sat_used;
    uint8_t num_sat_used_l1;
    uint8_t num_sat_used_multi;
    uint8_t num_bases;
    uint32_t reference_id;
    double correction_age;
    double solution_age;

    bool has_rotation_3D = false;
    bool has_heading;
    bool has_velocity;
    bool has_velocity_3D = false;
    bool has_roll_pitch;
  };

  struct GNSSMeasurement {
    // ToDo: 26.03: @Lars: can you adjust this? If it is not done on 28.03, I will take care
    bool hasRTK = false;
    GNSSMeasurementEpoch measMainAnt{};
    bool hasDualAntenna = false;
    GNSSMeasurementEpoch measAuxAnt{};
    bool hasDualAntennaDD = false;
    GNSSMeasurementEpoch measDualAntennaDD{};
    bool hasRTCMDD = false;
    GNSSMeasurementEpoch measRTCMDD{};

    //double tow{};        // timestamp in GPS timing
    //rclcpp::Time timestamp{};  //  timestamp, when the gnss meas is arrived at online FGO
    // gnss -> pre-processing -> delay (system time + PPS)
    //double delay;
    //std::vector<GNSSObs> mainAntenna{};
    //std::vector<GNSSObs> auxAntenna{};
    //std::vector<GNSSObs> ddRTCM{};
    //std::vector<GNSSObs> ddDualAntenna{};
    //double timeOffsetGALGPS{};
    //bool isGGTOValid = false;
    //uint8_t integrity_flag{};

    // uint refSatIdRTCM{};
    //uint refSatIdAntAux{};
    //gtsam::Vector3 refSatPosRTCM{};
    //gtsam::Vector3 refSatVelRTCM{};
    //gtsam::Vector3 refSatPosAntAux{};
    //gtsam::Vector3 refSatVelAntAux{};
    //gtsam::Vector3 basePosRTCM{};
    //gtsam::Vector3 basePosAntAux{};
    //bool hasDDRTCM = false;
  };

  struct CycleSlipStruct {
    uint32_t satID{};
    int N{}; //count of not sliped
    double md{}; //mean
    double md2{}; //squared mean
    double sd2{}; //sigma
    bool connection{}; //can sat be found in newest gnss meas
    //double timeLastSlip = -1;
    //bool cycleSlip{}; //sliped since last meas
  };

  struct CSDataStruct {
    uint32_t satID{};
    gtsam::Vector3 satPos{};
    //gtsam::Vector3 satRefPos{};
    double cp{};
    double cpVar{};
    double el{};
  };

  /** Timing **/
  struct PPS {
    std::atomic_uint_fast64_t counter;
    std::atomic_int_fast64_t localDelay;  // in milliseconds
    rclcpp::Time lastPPSTime;
  };

}






#endif //ONLINE_FGO_DATATYPESMEASUREMENT_H
