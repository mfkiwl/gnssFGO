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
// Created by haoming on 22.05.24.
//

#ifndef ONLINE_FGO_DATASETBINLOCO_H
#define ONLINE_FGO_DATASETBINLOCO_H

#include "include/dataset/Dataset.h"

namespace fgo::dataset {

  using namespace fgo::data;

  struct BinLocoDataBatch : DataBatch {
    std::vector<PVASolution> gnss;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_raw;
    std::vector<cv_bridge::CvImagePtr> image;
    std::vector<boreas_msgs::msg::SensorPose> camera_pose;
    std::vector<boreas_msgs::msg::SensorPose> lidar_pose;
  };


  class BinLoco : public DatasetBase {
  public:
    explicit Boreas() = default;
    ~Boreas() override = default;

    void initialize(std::string name_,
                    offline_process::OfflineFGOBase &node,
                    fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager) override;

    void parseDataFromBag() override;

    void trimDataBlocks() override {

    }

    data::State referenceToState(const PVASolution &reference) override {
      static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
      return sensor::gnss::PVASolutionToState(reference, transReferenceFromBase.translation());
    }

    PVASolution getNextGNSSPVA() { return data_pva_gps.getNextData(); }

    fgo::graph::StatusGraphConstruction feedDataToGraph(const std::vector<double> &stateTimestamps) override;

    BinLocoDataBatch getDataBefore(double timestamp,
                                  bool erase = false) {
      BinLocoDataBatch batch;
      return batch;
    }

    BinLocoDataBatch getDataBetween(double timestamp_start,
                                   double timestamp_end) {
      BinLocoDataBatch batch;
      return batch;
    }




  };



}


#endif //ONLINE_FGO_DATASETBINLOCO_H
