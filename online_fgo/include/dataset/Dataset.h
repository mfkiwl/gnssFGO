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
// Created by haoming on 15.04.24.
//

#ifndef ONLINE_FGO_DATASET_H
#define ONLINE_FGO_DATASET_H
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <iterator>
#include <rclcpp/time.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <irt_nav_msgs/msg/gnss_obs_pre_processed.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ublox_msgs/msg/rxm_rawx.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <utility>
#include "sensor/GNSS/GNSSDataParser.h"
#include "data/DataTypes.h"
#include "DatasetParam.h"
#include "BagReader.h"
#include "include/data/BagUtils.h"
#include "utils/GNSSUtils.h"

//ToDo to be deleted
#include "integrator/param/IntegratorParams.h"

namespace fgo::dataset {
  using namespace fgo::data;

  template<typename DataType>
  struct DataBlock {
    typedef std::map<rclcpp::Time, DataType> DataMap;
    std::string data_name;
    rclcpp::Time timestamp_start;
    rclcpp::Time timestamp_end;
    bool fully_loaded = false;
    double max_loading_size = 0.;
    DataMap data;
    typename DataMap::iterator data_iter;

    explicit DataBlock(std::string name,
                       double max_loading_size,
                       std::function<rclcpp::Time(DataType)> cb_time_getter)
        : data_name(std::move(name)), max_loading_size(max_loading_size), cb_get_timestamp(cb_time_getter) {};

    std::function<std::pair<bool, DataMap>(double, double)> cb_load_data;
    std::function<rclcpp::Time(DataType)> cb_get_timestamp;
    std::function<void(const std::vector<DataType> &)> cb_on_data;

    void setCbOnData(std::function<void(const std::vector<DataType> &)> on_data) {
      cb_on_data = std::move(on_data);
    }

    void setCbLoadData(std::function<std::pair<bool, DataMap>(double, double)> load_data) {
      cb_load_data = std::move(load_data);
    }

    void setData(DataMap new_data, bool loaded = true) {
      data = std::move(new_data);
      data_iter = data.begin();
      fully_loaded = loaded;
    }

    void trimData(const rclcpp::Time &start,
                  const rclcpp::Time &end) {
      const auto valid_end_time = end.nanoseconds() > 0;
      auto iter = data.begin();
      while (iter != data.end()) {
        if (iter->first < start || (valid_end_time && iter->first > end))
          iter = data.erase(iter);
        else
          iter++;
      }
      data_iter = data.begin();
      timestamp_start = start;
      if (timestamp_end.nanoseconds() && timestamp_end > end)
        timestamp_end = end;
    }

    bool hasNextMeasurement() {
      return data_iter != data.end() || (data_iter + 1) != data.end();
    }

    DataType getNextData() {
      if (data_iter != data.end()) {
        auto result = data_iter->second;
        if (cb_on_data) {
          std::vector<DataType> data_vec = {result};
          cb_on_data(data_vec);
        }
        data_iter++;
        return result;
      } else if (!fully_loaded) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("offline_process"),
                            "OfflineFGO DataBlock of " << data_name << ": buffer empty reading ... ");
        auto [loaded, new_data] = cb_load_data(timestamp_end.seconds(), max_loading_size);
        setData(new_data, loaded);
        return getNextData();
      } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("offline_process"),
                            "OfflineFGO DataBlock of " << data_name << ": NO MORE DATA ");
        return DataType();
      }
    }

    std::vector<DataType> getDataBefore(const rclcpp::Time &timestamp,
                                        bool erase = false) {
      std::vector<DataType> measurement;
      auto iter = data.begin();
      while (iter != data.end()) {
        const auto time = cb_get_timestamp(iter->second);
        if (time <= timestamp) {
          measurement.emplace_back(iter->second);
          if (cb_on_data) {
            cb_on_data(measurement);
          }
          if (erase) {
            iter = data.erase(iter);
            data_iter = iter;
          } else
            iter++;
        } else
          break;
      }
      return measurement;
    }

    std::vector<DataType> getDataBetween(const rclcpp::Time &lastStateTime,
                                         const rclcpp::Time &currentStateTime) {
      std::vector<DataType> measurement;
      bool findMeasurement = false;
      auto iter = data.begin();

      rclcpp::Time time_last = cb_get_timestamp(std::prev(data.end())->second);
      if (time_last < currentStateTime) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("offline_process"),
                            "OfflineFGO bag_reader: buffer empty reading ... " << data_name);
        auto [loaded, new_data] = cb_load_data(timestamp_end.seconds(), max_loading_size);
        setData(new_data, loaded);
      }

      while (iter != data.end()) {
        rclcpp::Time time = cb_get_timestamp(iter->second);
        if (time < currentStateTime && time >= lastStateTime) {
          measurement.emplace_back(iter->second);
          findMeasurement = true;
          if (cb_on_data) {
            cb_on_data(measurement);
          }
        } else if (findMeasurement || time > lastStateTime) {
          break;
        }
        iter++;
      }
      return measurement;
    }
  };


  template<typename ReferenceType>
  struct DatasetBase {
    std::shared_ptr<DatasetParam> params_;
    std::string name;
    std::unique_ptr<BagReader> reader_;
    DataBlock<IMUMeasurement> data_imu;
    DataBlock<State> data_reference_state;
    DataBlock<ReferenceType> data_reference;

    rclcpp::Time timestamp_start;
    rclcpp::Time timestamp_end;
    State prior_state;

    const std::function<rclcpp::Time(const IMUMeasurement &)> imu_time_getter = IMUDataTimeGetter;
    const std::function<rclcpp::Time(const State &)> state_time_getter = StateTimeGetter;
    const std::function<rclcpp::Time(const ReferenceType &)> reference_time_getter = FGODataTimeGetter<ReferenceType>;


    explicit DatasetBase(std::string name_, const std::shared_ptr<DatasetParam> &param)
        : params_(param),
          name(std::move(name_)),
          data_imu("IMU", 0., IMUDataTimeGetter),
          data_reference_state("ReferenceState", 0., StateTimeGetter),
          data_reference("Reference", 0., PVADataTimeGetter) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": initializing DatasetBase ...");
      reader_ = std::make_unique<BagReader>(params_);
      reader_->readFullDataFromBag();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": DatasetBase initialized!");
    }

    // maybe unuseful
    virtual data::State referenceToState(const ReferenceType &reference) {
      return data_reference_state.data.begin()->second;
    };

    void setTimestampsFromReference(bool reset_first = false) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name
                                                                                  << ": Setting start and end timestamps based on the reference data ...");
      if (timestamp_start.nanoseconds() == 0 || reset_first) {
        size_t first_pos = 0;
        auto iter = data_reference.data.begin();
        auto first_timestamp = iter->first;
        iter = data_reference.data.erase(iter);
        first_pos++;
        if (params_->start_offset != 0.) {
          while (iter != data_reference.data.end()) {
            if ((iter->first - first_timestamp).seconds() >= params_->start_offset)
              break;
            iter = data_reference.data.erase(iter);
            first_pos++;
          }
        }
        first_timestamp = iter->first;
        timestamp_start = first_timestamp;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                           "OfflineFGO Dataset " << name << ": setting start time " << timestamp_start.seconds());
        prior_state = referenceToState(iter->second);
        const auto imu_data_vec = data_imu.getDataBefore(first_timestamp);
        prior_state.omega = imu_data_vec.end()->gyro;
        prior_state.accMeasured = (gtsam::Vector6()
            << imu_data_vec.end()->accRot, imu_data_vec.end()->accLin).finished();

        if (data_reference_state.data.size() > first_pos) {
          auto eraseIter = data_reference_state.data.begin();
          std::advance(eraseIter, first_pos);
          data_reference_state.data.erase(data_reference_state.data.begin(), eraseIter);
          data_reference_state.data_iter = data_reference_state.data.begin();
        }
      }

      if (params_->pre_defined_duration == 0.) {
        timestamp_end = (--data_reference.data.end())->first;
      } else {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                           "OfflineFGO Dataset " << name << ": setting the end time from parameter current");
        timestamp_end = timestamp_start + rclcpp::Duration::from_nanoseconds(params_->pre_defined_duration * 1e9);
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << " started from " << std::fixed << timestamp_start.seconds()
                                               << " end at " << timestamp_end.seconds());
    }

    template<typename ROSMessageObject>
    std::vector<std::pair<rclcpp::Time, ROSMessageObject>> readROSMessages(const std::string &topic) {
      const auto raw = reader_->getAllMessageRAW(topic);

      std::vector<std::pair<rclcpp::Time, ROSMessageObject>> data;
      for (const auto &time_buffer_pair: raw) {
        auto [timestamp, msg] = reader_->deserialize_message<ROSMessageObject>(time_buffer_pair.second,
                                                                               time_buffer_pair.first);
        data.emplace_back(std::make_pair(timestamp, msg));
      }
      std::cout << "data topic: " << topic << " data size: " << data.size() << std::endl;
      return data;
    }

    virtual void parseDataFromBag() = 0;

    virtual void trimDataBlocks() = 0;

    IMUMeasurement getNextIMU() { return data_imu.getNextData(); }

    std::vector<IMUMeasurement>
    getIMUBetween(rclcpp::Time &lastOptStateTime, const rclcpp::Time &currentOptStateTime) {
      return data_imu.getDataBetween(lastOptStateTime, currentOptStateTime);
    }

    std::vector<IMUMeasurement>
    getIMUBefore(double timestamp,
                 bool erase = false) {
      return data_imu.getDataBefore(rclcpp::Time(timestamp * 1e9, RCL_ROS_TIME), erase);
    }

    State getNextReferenceState() { return data_reference_state.getNextData(); }

    std::vector<State>
    getReferenceStateBetween(rclcpp::Time &lastOptStateTime, const rclcpp::Time &currentOptStateTime) {
      return data_reference_state.getDataBetween(lastOptStateTime, currentOptStateTime);
    }

    std::vector<State>
    getReferenceStateBefore(double timestamp,
                            bool erase = false) {
      return data_reference_state.getDataBefore(rclcpp::Time(timestamp * 1e9, RCL_ROS_TIME), erase);
    }

    ReferenceType getNextReference() { return data_reference.getNextData(); }

    std::vector<ReferenceType>
    getReferenceBetween(rclcpp::Time &lastOptStateTime, const rclcpp::Time &currentOptStateTime) {
      return data_reference.getDataBetween(lastOptStateTime, currentOptStateTime);
    }

    std::vector<ReferenceType>
    getReferenceBefore(double timestamp,
                       bool erase = false) {
      return data_reference.getDataBefore(rclcpp::Time(timestamp * 1e9, RCL_ROS_TIME), erase);
    }

  };


}


#endif //ONLINE_FGO_DATASET_H
