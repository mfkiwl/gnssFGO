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
// Created by haoming on 07.06.24.
//

#ifndef ONLINE_FGO_DATASETPOHANG_H
#define ONLINE_FGO_DATASETPOHANG_H

#include <ranges>

#include "include/dataset/Dataset.h"
#include "sensor/gnss/RoboGNSSParser.h"

namespace fgo::dataset {
  using namespace fgo::data;

  struct PohangDataBatch : DataBatch {
    std::vector<PVASolution> gnss;
    std::vector<StereoPair> stereo_pair;
    std::vector<cv_bridge::CvImagePtr> infrared;
    std::vector<cv_bridge::CvImagePtr> radar;
    std::vector<IMUMeasurement> imu_lidar_front;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_front;
  };

  /*
   * (1)  /omni_cam/cam_0/compressed              1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]     NOT USED
	   (2)  /omni_cam/cam_5/compressed              1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]     NOT USED
     (3)  /lidar_starboard/os_cloud_node/points   1030 msgs (10.00 Hz)    : sensor_msgs/msg/PointCloud2 [ros2msg]     NOT USED
     (4)  /lidar_starboard/imu/data              10247 msgs (99.49 Hz)    : sensor_msgs/msg/Imu [ros2msg]     NOT USED
     (5)  /lidar_port/imu/data                   10264 msgs (99.65 Hz)    : sensor_msgs/msg/Imu [ros2msg]     NOT USED
     (6)  /gx5/imu/data                          10300 msgs (100.00 Hz)   : sensor_msgs/msg/Imu [ros2msg]     NOT USED
     (7)  /stereo_cam/left_img/compressed         1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]
     (8)  /radar/image                              82 msgs (0.80 Hz)     : sensor_msgs/msg/Image [ros2msg]
     (9)  /omni_cam/cam_2/compressed              1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]     NOT USED
     (10) /omni_cam/cam_1/compressed              1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]     NOT USED
     (11) /gps/gps_nav                             506 msgs (4.91 Hz)     : sensor_msgs/msg/NavSatFix [ros2msg]     NOT USED
     (12) /stereo_cam/right_img/compressed        1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]
     (13) /gps/gps_nav_raw                         506 msgs (4.91 Hz)     : robognss_msgs/msg/PVT [ros2msg]
     (14) /infrared/image                         1030 msgs (10.00 Hz)    : sensor_msgs/msg/Image [ros2msg]
     (15) /lidar_front/imu/data                   9529 msgs (92.52 Hz)    : sensor_msgs/msg/Imu [ros2msg]
     (16) /lidar_front/os_cloud_node/points       1030 msgs (10.00 Hz)    : sensor_msgs/msg/PointCloud2 [ros2msg]
     (17) /omni_cam/cam_3/compressed              1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]     NOT USED
     (18) /gx5/imu_calib/data                    10300 msgs (100.00 Hz)   : sensor_msgs/msg/Imu [ros2msg]
     (19) /lidar_port/os_cloud_node/points        1030 msgs (10.00 Hz)    : sensor_msgs/msg/PointCloud2 [ros2msg]     NOT USED
     (20) /omni_cam/cam_4/compressed              1030 msgs (10.00 Hz)    : sensor_msgs/msg/CompressedImage [ros2msg]     NOT USED
     (21) /gx5/baseline                            353 msgs (3.43 Hz)     : geometry_msgs/msg/PoseStamped [ros2msg]
   */

  class Pohang : public DatasetBase {
  protected:
    DataBlock<StereoPair> data_stereo_pair;
    DataBlock<PVASolution> data_gnss;
    DataBlock<sensor_msgs::msg::PointCloud2::SharedPtr> data_lidar_front;
    DataBlock<IMUMeasurement> data_imu_lidar_front;
    DataBlock<cv_bridge::CvImagePtr> data_infrared;
    DataBlock<cv_bridge::CvImagePtr> data_radar;

    //DataBlock<sensor_msgs::msg::PointCloud2> data_lidar_port;
    //DataBlock<sensor_msgs::msg::PointCloud2> data_lidar_starboard;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_{};
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_baseline_{};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_front;

    std::shared_ptr<image_transport::ImageTransport> it;
    image_transport::Publisher pub_left_raw;
    image_transport::Publisher pub_right_raw;
    image_transport::Publisher pub_infrared;
    image_transport::Publisher pub_radar;

  public:
    explicit Pohang() = default;
    ~Pohang() override = default;

    void initialize(std::string name_,
                    offline_process::OfflineFGOBase &node,
                    fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager) override;


    void parseDataFromBag() override;
    void trimDataBlocks() override {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": trimming datablocks to start timestamp "
                                               << std::fixed << timestamp_start.seconds() << " and end timestamp "
                                               << timestamp_end.seconds());
      data_imu.trimData(timestamp_start, timestamp_end);
      data_gnss.trimData(timestamp_start, timestamp_end);
      data_reference_state.trimData(timestamp_start, timestamp_end);
      data_reference.trimData(timestamp_start, timestamp_end);
      data_lidar_front.trimData(timestamp_start, timestamp_end);
      data_imu_lidar_front.trimData(timestamp_start, timestamp_end);
      data_infrared.trimData(timestamp_start, timestamp_end);
      data_radar.trimData(timestamp_start, timestamp_end);
      data_stereo_pair.trimData(timestamp_start, timestamp_end);
    }

    data::State referenceToState(const PVASolution &reference) override {
      static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
      return sensor::gnss::PVASolutionToState(reference, transReferenceFromBase.translation());
    }

    fgo::graph::StatusGraphConstruction feedDataToGraph(const std::vector<double> &stateTimestamps) override;

    PohangDataBatch getDataBefore(double timestamp,
                                  bool erase = false) {
      PohangDataBatch batch;
      batch.timestamp_end = timestamp;
      const auto ros_timestamp = utils::secondsToROSTime(timestamp);
      batch.imu = data_imu.getDataBefore(ros_timestamp, erase);
      batch.reference_pva = data_reference.getDataBefore(ros_timestamp, erase);
      batch.reference_state = data_reference_state.getDataBefore(ros_timestamp, erase);
      batch.stereo_pair = data_stereo_pair.getDataBefore(ros_timestamp, erase);
      batch.gnss = data_gnss.getDataBefore(ros_timestamp, erase);
      batch.lidar_front = data_lidar_front.getDataBefore(ros_timestamp, erase);
      batch.imu_lidar_front = data_imu_lidar_front.getDataBefore(ros_timestamp, erase);
      batch.infrared = data_infrared.getDataBefore(ros_timestamp, erase);
      batch.radar = data_radar.getDataBefore(ros_timestamp, erase);
      return batch;
    }

    PohangDataBatch getDataBetween(double timestamp_start,
                                   double timestamp_end) {
      PohangDataBatch batch;
      batch.timestamp_end = timestamp_end;
      const auto ros_timestamp_start = utils::secondsToROSTime(timestamp_start);
      const auto ros_timestamp_end = utils::secondsToROSTime(timestamp_end);
      batch.imu = data_imu.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_pva = data_reference.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.reference_state = data_reference_state.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.stereo_pair = data_stereo_pair.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.gnss = data_gnss.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.lidar_front = data_lidar_front.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.imu_lidar_front = data_imu_lidar_front.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.infrared = data_infrared.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.radar = data_radar.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      return batch;
    }
  };


}
#endif //ONLINE_FGO_DATASETPOHANG_H
