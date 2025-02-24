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
// Created by haoming on 12.06.24.
//

#ifndef ONLINE_FGO_DATASETURBANNAV_H
#define ONLINE_FGO_DATASETURBANNAV_H
#pragma once

#include "dataset/Dataset.h"


namespace fgo::dataset {

  using namespace fgo::data;

  struct UrbanNavBatch : DataBatch {
    std::vector<StereoPair> stereo;
    std::vector<sensor_msgs::msg::PointCloud2> lidar_front;

  };

  class UrbanNav : public DatasetBase {
  protected:
    DataBlock<GNSSMeasurement> data_gnss;
    integrator::param::IntegratorGNSSTCParamsPtr gnss_param_ptr;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_pvt_;

  public:


  }



}




#endif //ONLINE_FGO_DATASETURBANNAV_H
