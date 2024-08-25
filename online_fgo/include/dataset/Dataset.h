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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <robognss_msgs/msg/pvt.hpp>
#include <image_transport/image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <robognss_msgs/msg/pvt.hpp>
#include <utility>

#include "sensor/gnss/GNSSDataParser.h"
#include "integrator/param/IntegratorParams.h"
#include "sensor/SensorCalibrationManager.h"
#include "DatasetParam.h"
#include "BagReader.h"
#include "BagUtils.h"
#include "utils/GNSSUtils.h"
#include "utils/ROSUtils.h"


//ToDo to be deleted
#include "integrator/param/IntegratorParams.h"

namespace offline_process {
  class OfflineFGOBase;
}

namespace fgo::dataset {
  using namespace fgo::data;

  template<typename DataType>
  struct DataBlock {
    typedef std::map<rclcpp::Time, DataType> DataMap;
    bool excluded = false;
    std::string data_name;
    std::string data_topic;
    rclcpp::Time timestamp_start;
    rclcpp::Time timestamp_end;
    bool fully_loaded = false;
    double max_loading_size = 0.;
    DataMap data;
    typename DataMap::iterator data_iter;

    DataBlock() = default;
    explicit DataBlock(std::string name,
                       std::string topic,
                       double max_loading_size,
                       std::function<rclcpp::Time(DataType)> cb_time_getter)
      : data_name(std::move(name)), data_topic(std::move(topic)), max_loading_size(max_loading_size),
        cb_get_timestamp(cb_time_getter) {};

    std::function<std::tuple<bool, DataMap>(int64_t, double, const std::string &)> cb_load_data;
    std::function<rclcpp::Time(DataType)> cb_get_timestamp;
    std::function<void(const std::vector<DataType> &)> cb_on_data;

    void setExcluded(bool ex) { excluded = ex; }

    void setCbOnData(std::function<void(const std::vector<DataType> &)> on_data) {
      cb_on_data = std::move(on_data);
    }

    void setCbLoadData(std::function<std::tuple<bool, DataMap>(int64_t, double, const std::string &)> load_data) {
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
      if (excluded)
        return DataType();

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
                            "OfflineFGO DataBlock of " << data_name << ": data empty reading ... ");
        auto [has_next, new_data] = cb_load_data(timestamp_end.nanoseconds(), max_loading_size, data_topic);
        setData(new_data, !has_next);
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
      if (excluded)
        return measurement;
      auto iter = data.begin();

      if (cb_load_data && iter == data.end() && !fully_loaded) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("offline_process"),
                            "OfflineFGO DataBlock of " << data_name << ": data empty reading ... ");
        auto [has_next, new_data] = cb_load_data(timestamp.nanoseconds(), max_loading_size, data_topic);
        setData(new_data, !has_next);
        getDataBefore(timestamp, erase);
      }

      while (iter != data.end()) {
        const auto time = cb_get_timestamp(iter->second);
        if (time <= timestamp) {
          measurement.emplace_back(iter->second);
          if (cb_on_data) {
            cb_on_data(measurement);
          }
          if (erase) {
            iter = data.erase(iter);
          } else
            iter++;
          data_iter = iter;
        } else
          break;
      }
      return measurement;
    }

    std::vector<DataType> getDataBetween(const rclcpp::Time &lastStateTime,
                                         const rclcpp::Time &currentStateTime) {
      std::vector<DataType> measurement;
      if (excluded)
        return measurement;
      bool findMeasurement = false;
      auto iter = data.begin();

      rclcpp::Time time_last = cb_get_timestamp(std::prev(data.end())->second);
      if (time_last < currentStateTime) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("offline_process"),
                            "OfflineFGO bag_reader: data empty reading ... " << data_name);
        auto [has_next, new_data] = cb_load_data(timestamp_end.nanoseconds(), max_loading_size, data_topic);
        setData(new_data, !has_next);
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


  struct DataBatch {
    double timestamp_start;
    double timestamp_end;
    std::vector<IMUMeasurement> imu;
    std::vector<PVASolution> reference_pva;
    std::vector<State> reference_state;
  };

  //template<typename PVASolution>
  class DatasetBase {
  public:
    std::shared_ptr<DatasetParam> param_;
    offline_process::OfflineFGOBase *appPtr_;
    std::string name;
    fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager_;
    std::unique_ptr<BagReader> reader_;
    DataBlock<IMUMeasurement> data_imu;
    DataBlock<State> data_reference_state;
    DataBlock<PVASolution> data_reference;

    rclcpp::Time timestamp_start;
    rclcpp::Time timestamp_end;
    State prior_state;

    DatasetBase() {};
    virtual ~DatasetBase() = default;

    virtual void initialize(std::string name_,
                            offline_process::OfflineFGOBase &node,
                            fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager);

    // maybe unuseful
    virtual data::State referenceToState(const PVASolution &reference) {
      return data_reference_state.data.begin()->second;
    };

    void setTimestampsFromReference(bool reset_first = false);

    template<typename ROSMessageObject>
    std::tuple<rclcpp::Time, ROSMessageObject>
    readROSMessage(const std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>> &raw_pair) {
      return reader_->deserialize_message<ROSMessageObject>(raw_pair.second,
                                                            raw_pair.first);
    }

    template<typename ROSMessageObject>
    std::vector<std::pair<rclcpp::Time, ROSMessageObject>>
    readROSMessages(const std::string &topic) {
      const auto raw = reader_->getAllMessageRAW(topic);

      std::vector<std::pair<rclcpp::Time, ROSMessageObject>> data;
      for (const auto &time_buffer_pair: raw) {

        auto [timestamp, msg] = reader_->deserialize_message<ROSMessageObject>(time_buffer_pair.second,
                                                                               time_buffer_pair.first);
        data.emplace_back(std::make_pair(timestamp, msg));

      }
      return data;
    }

    //TODo @Haoming: looks redundant
    template<typename ROSMessageObject>
    std::vector<std::pair<rclcpp::Time, ROSMessageObject>>
    readROSMessagesNoHeader(const std::string &topic) {
      const auto raw = reader_->getAllMessageRAW(topic);

      std::vector<std::pair<rclcpp::Time, ROSMessageObject>> data;
      for (const auto &time_buffer_pair: raw) {
        auto [timestamp, msg] = reader_->deserialize_message_no_header<ROSMessageObject>(time_buffer_pair.second,
                                                                                         time_buffer_pair.first);
        data.emplace_back(std::make_pair(timestamp, msg));

      }
      return data;
    }

    virtual void parseDataFromBag() = 0;

    virtual void trimDataBlocks() = 0;

    virtual fgo::graph::StatusGraphConstruction feedDataToGraph(const std::vector<double> &stateTimestamps) = 0;

    IMUMeasurement getNextIMU() { return data_imu.getNextData(); }

    State getNextReferenceState() { return data_reference_state.getNextData(); }

    PVASolution getNextReference() { return data_reference.getNextData(); }
  };
}


#endif //ONLINE_FGO_DATASET_H
