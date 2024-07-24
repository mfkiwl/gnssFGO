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
    bool autoLoading = false;
    double start_offset = 0.;
    double pre_defined_duration = 0.;

    std::map<std::string, fgo::data::DataType> topic_type_map;

    DatasetParam(rclcpp::Node &node, const std::string &dataset_name) {
      ::utils::RosParameter<std::string> bag_path_(dataset_name + ".bagPath", node);
      this->bag_path = bag_path_.value();

      ::utils::RosParameter<double> max_bag_memory_usage_(dataset_name + ".maxMemoryFootprint", node);
      this->max_bag_memory_usage = max_bag_memory_usage_.value();

      ::utils::RosParameter<std::vector<std::string>> bag_topic_filter_(dataset_name + ".fullyLoadedTopics", node);
      this->bag_fully_loaded_topics = bag_topic_filter_.value();

      ::utils::RosParameter<std::vector<std::string>> excludedTopics_(dataset_name + ".excludedTopics", node);
      this->excluded_topics = excludedTopics_.value();

      ::utils::RosParameter<bool> autoLoading_(dataset_name + ".autoLoading", true, node);
      this->autoLoading = autoLoading_.value();

      ::utils::RosParameter<double> start_offset_(dataset_name + ".startOffset", 0., node);
      this->start_offset = start_offset_.value();

      ::utils::RosParameter<double> pre_defined_duration_(dataset_name + ".preDefinedDuration", node);
      this->pre_defined_duration = pre_defined_duration_.value();

    }
  };
}

#endif //ONLINE_FGO_DATASETPARAM_H
