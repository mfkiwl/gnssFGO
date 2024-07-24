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

#ifndef ONLINE_FGO_DATASETKITTI_H
#define ONLINE_FGO_DATASETKITTI_H
#pragma once
#include <ranges>
#include <image_transport/image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <robognss_msgs/msg/pvt.hpp>
#include "Dataset.h"
#include "sensor/gnss/RoboGNSSParser.h"

namespace fgo::dataset {
  using namespace fgo::data;

  struct KittiDataBatch : DataBatch
  {
    std::vector<Pose> baseline_batch;
    std::vector<StereoPair> stereo_pair_batch;
  };



  struct DatasetKitti : DatasetBase<PVASolution> {




  };


}


#endif //ONLINE_FGO_DATASETKITTI_H
