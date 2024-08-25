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
// Created by haoming on 16.04.24.
//

#ifndef ONLINE_FGO_DATASETPARAM_H
#define ONLINE_FGO_DATASETPARAM_H

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "utils/ROSParameter.h"
#include "BagUtils.h"

namespace fgo::dataset {
  struct DatasetParam {
    std::string bag_path;
    double max_bag_memory_usage = 4.; // in GB,
    std::vector<std::string> bag_fully_loaded_topics;
    std::vector<std::string> excluded_topics;
    std::map<std::string, std::string> sensor_topic_map;
    bool autoLoading = false;
    bool onlyDataPlaying = false;
    double start_offset = 0.;
    double pre_defined_duration = 0.;
    double heading_offset_deg = 0.;
    std::string bag_format = "sqlite";

    std::map<std::string, fgo::data::DataType> topic_type_map;

    DatasetParam(rclcpp::Node *node, const std::string &dataset_name) {
      ::utils::RosParameter<std::string> bag_path_("Dataset." + dataset_name + ".bagPath", *node);
      this->bag_path = bag_path_.value();

      ::utils::RosParameter<std::string> bag_format_("Dataset." + dataset_name + ".bagFormate", *node);
      this->bag_format = bag_format_.value();

      ::utils::RosParameter<double> max_bag_memory_usage_("Dataset." + dataset_name + ".maxMemoryFootprint", *node);
      this->max_bag_memory_usage = max_bag_memory_usage_.value();

      ::utils::RosParameter<std::vector<std::string>> sensors("Dataset." + dataset_name + ".sensors", *node);
      const auto &sensors_ = sensors.value();
      ::utils::RosParameter<std::vector<std::string>> sensor_topics("Dataset." + dataset_name + ".sensorTopics", *node);
      const auto &sensor_topics_ = sensor_topics.value();
      if (sensors_.size() == sensor_topics_.size()) {
        for (size_t i = 0; i < sensors_.size(); i++)
          sensor_topic_map.insert(std::make_pair(sensors_[i], sensor_topics_[i]));
      } else
        throw std::invalid_argument("DatasetParam: sensors and sensor topics of dataset" + dataset_name + " do not match!");

      ::utils::RosParameter<std::vector<std::string>> bag_topic_filter_(
        "Dataset." + dataset_name + ".fullyLoadedTopics", *node);
      this->bag_fully_loaded_topics = bag_topic_filter_.value();

      ::utils::RosParameter<std::vector<std::string>> excludedTopics_("Dataset." + dataset_name + ".excludedTopics",
                                                                      *node);
      this->excluded_topics = excludedTopics_.value();

      ::utils::RosParameter<bool> autoLoading_("Dataset." + dataset_name + ".autoLoading", true, *node);
      this->autoLoading = autoLoading_.value();

      ::utils::RosParameter<bool> onlyData("Dataset." + dataset_name + ".onlyDataPlaying", false, *node);
      this->onlyDataPlaying = onlyData.value();

      ::utils::RosParameter<double> start_offset_("Dataset." + dataset_name + ".startOffset", 0., *node);
      this->start_offset = start_offset_.value();

      ::utils::RosParameter<double> pre_defined_duration_("Dataset." + dataset_name + ".preDefinedDuration", *node);
      this->pre_defined_duration = pre_defined_duration_.value();

      ::utils::RosParameter<double> heading_offset("Dataset." + dataset_name + ".headingOffsetDeg", 0., *node);
      this->heading_offset_deg = heading_offset.value();
    }
  };
}

#endif //ONLINE_FGO_DATASETPARAM_H
