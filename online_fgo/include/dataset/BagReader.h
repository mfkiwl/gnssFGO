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


#ifndef ONLINE_FGO_BAGREADER_H
#define ONLINE_FGO_BAGREADER_H
#pragma once
#include <map>
#include <mutex>
#include <thread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <irt_nav_msgs/msg/pva_geodetic.hpp>
#include <irt_nav_msgs/msg/gnss_obs_pre_processed.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
///#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <ublox_msgs/msg/rxm_rawx.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

//#include "utils/indicators/indicators.hpp"
//#include "sensor/gnss/GNSSDataParser.h"
#include "data/DataTypesFGO.h"
#include "dataset/DatasetParam.h"
#include "BagUtils.h"
#include "utils/GNSSUtils.h"

namespace fgo::dataset {

  /**
   * Design Idea: because we have different dataset for multiple research works, we only use the bag reader to read raw buffers
   * the raw data will be interpreted / transformed into data object to be used.
   * A problem should be considered: loading data from e.g., images will fire the RAM, so the loading action should be controlled
   * To do so: fetching buffers from the bag will be running in a separate thread,
   *           If the memory_usage_ is larger than a threshold, then the loop runs idle until the the memory_usage_ comes down
   */
  using namespace fgo::data;

  class BagReader {
  public:
    std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>> getAllMessageRAW(const std::string &topic) {
      std::lock_guard<std::mutex> lg(mutex_);
      try {
        auto raw = raw_message_map_.at(topic);
        size_t buffer_size;
        for (const auto &buffer_pair: raw)
          buffer_size += buffer_pair.second->buffer_length;
        memory_usage_ -= buffer_size * 1e-9;
        raw_message_map_.at(topic).clear();
        return raw;
      }
      catch (std::exception &ex) {
        // TODO
      }
    }

    template<typename T>
    std::tuple<rclcpp::Time, T> deserialize_message(std::shared_ptr<rcutils_uint8_array_t> data,
                                                    uint64_t bagtime) {
      T new_msg;
      rclcpp::Serialization<T> serialization;
      rclcpp::SerializedMessage extracted_serialized_msg(*data);
      serialization.deserialize_message(
        &extracted_serialized_msg, &new_msg);
      try {
        auto ros_timestamp = rclcpp::Time(new_msg.header.stamp.sec, new_msg.header.stamp.nanosec, RCL_ROS_TIME);
        return {ros_timestamp, new_msg};
      }
      catch (std::exception &ex) {
        const auto ros_timestamp = rclcpp::Time(bagtime, RCL_ROS_TIME);
        return {ros_timestamp, new_msg};
      }
    }

    //constructor: initial with Path of bag file and the topics of gnss and IMU
    explicit BagReader(const std::shared_ptr<DatasetParam> &param) : param_(param) {}

    ~BagReader() = default;

    //[[nodiscard]] BufferStatus getBufferStatus() const {return status_;};
  private:
    std::mutex mutex_;
    std::shared_ptr<DatasetParam> param_;
    //std::map<DataType, std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>>> raw_message_map_;
    std::map<std::string, std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>>> raw_message_map_;
    //BufferStatus status_;
    double memory_usage_ = 0.;

  public:
    void readFullDataFromBag() {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO: Reading full dataset at " << param_->bag_path);
      auto reader = std::make_shared<rosbag2_cpp::readers::SequentialReader>();
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = param_->bag_path;
      storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";

      try {
        reader->open(storage_options, converter_options);
      }
      catch (std::exception &ex) { std::cout << ex.what() << std::endl; }

      RCLCPP_INFO(rclcpp::get_logger("offline_process"), "OfflineFGO: bag opened");
      const auto bag_meta = reader->get_metadata();
      size_t message_counter;

      if (!param_->bag_fully_loaded_topics.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("offline_process"), "OfflineFGO: fully loaded topics");
        auto filter_ = rosbag2_storage::StorageFilter();
        filter_.topics = param_->bag_fully_loaded_topics;
        reader->set_filter(filter_);

        for (const auto &topic: param_->bag_fully_loaded_topics) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), topic << " | ");
          for (const auto &topic_meta: bag_meta.topics_with_message_count) {
            if (topic_meta.topic_metadata.name == topic)
              message_counter += topic_meta.message_count;
          }
        }
      } else {
        RCLCPP_INFO(rclcpp::get_logger("offline_process"), "OfflineFGO: no fully loaded topics");
        for (const auto &topic_meta: bag_meta.topics_with_message_count) {
          message_counter += topic_meta.message_count;
        }
      }

      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO " << ": start reading ...");
      while (reader->has_next()) {
        std::lock_guard<std::mutex> lg(mutex_);
        auto bag_message = reader->read_next();
        auto message_topic_name = bag_message->topic_name;
        raw_message_map_[bag_message->topic_name].emplace_back(
          std::make_pair(bag_message->time_stamp, bag_message->serialized_data));

      }

      reader->close();

    }

    std::map<std::string, std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>>>
    readPartialDataFromBag(const std::vector<std::string> &topics,
                           const int64_t &start_time_nanosec,
                           double max_size,
                           bool &has_next) {

      std::map<std::string, std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>>> data_map;
      double memory_usage = 0.;
      std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>> raw_data;
      rosbag2_cpp::readers::SequentialReader reader;
      rosbag2_storage::StorageOptions storage_options{};
      storage_options.uri = param_->bag_path;
      storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options{};
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";

      reader.open(storage_options, converter_options);

      auto filter_ = rosbag2_storage::StorageFilter();
      filter_.topics = topics;
      reader.set_filter(filter_);

      while (reader.has_next()) {
        const auto bag_message = reader.read_next();
        if (bag_message->time_stamp < start_time_nanosec)
          continue;
        memory_usage += bag_message->serialized_data->buffer_length * 1e-9;
        data_map[bag_message->topic_name].emplace_back(std::make_pair(bag_message->time_stamp, bag_message->serialized_data));
        if (memory_usage > max_size)
          break;
      }
      has_next = reader.has_next();

      reader.close();
      return data_map;
    };

    std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>>
    readSinglePartialDataFromBag(const std::string &topic,
                                 const int64_t &start_time_nanosec,
                                 double max_size,
                                 bool  &has_next) {
      double memory_usage = 0.;
      std::vector<std::pair<int64_t, std::shared_ptr<rcutils_uint8_array_t>>> raw_data;
      const auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
      rosbag2_storage::StorageOptions storage_options{};
      storage_options.uri = param_->bag_path;
      storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options{};
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";

      reader->open(storage_options, converter_options);
      auto filter_ = rosbag2_storage::StorageFilter();
      filter_.topics.emplace_back(topic);
      reader->set_filter(filter_);

      while (reader->has_next()) {
        auto bag_message = reader->read_next();
        if (bag_message->time_stamp < start_time_nanosec)
          continue;

        memory_usage += bag_message->serialized_data->buffer_length * 1e-9;
        raw_data.emplace_back(std::make_pair(bag_message->time_stamp, bag_message->serialized_data));
        if (memory_usage > max_size) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), topic << " read raw buffer with the size of " << memory_usage << " GB. ");
          break;
        }
      }
      has_next = reader->has_next();
      reader->close();
      return raw_data;
    }
  };
}

#endif //ONLINE_FGO_BAGREADER_H