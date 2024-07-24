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

#include "Dataset.h"
#include "sensor/gnss/RoboGNSSParser.h"

namespace fgo::dataset {
  using namespace fgo::data;

  struct PohangDataBatch : DataBatch {
    std::vector<Pose> baseline;
    std::vector<StereoPair> stereo_pair;
    std::vector<cv_bridge::CvImage> infrared;
    std::vector<cv_bridge::CvImage> radar;
    std::vector<PVASolution> gnss;
    std::vector<State> gnss_state;
    std::vector<IMUMeasurement> imu_lidar_front;
    std::vector<sensor_msgs::msg::PointCloud2> lidar_front;
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

  struct DatasetPohang : DatasetBase<PVASolution> {
    DataBlock<StereoPair> data_stereo_pair;
    DataBlock<Pose> data_baseline;
    DataBlock<sensor_msgs::msg::PointCloud2> data_lidar_front;
    DataBlock<IMUMeasurement> data_imu_lidar_front;
    DataBlock<cv_bridge::CvImage> data_infrared;
    DataBlock<cv_bridge::CvImage> data_radar;

    //DataBlock<sensor_msgs::msg::PointCloud2> data_lidar_port;
    //DataBlock<sensor_msgs::msg::PointCloud2> data_lidar_starboard;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_{};
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_baseline_{};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_front;

    image_transport::ImageTransport it;
    image_transport::Publisher pub_left_raw;
    image_transport::Publisher pub_right_raw;
    image_transport::Publisher pub_infrared;
    image_transport::Publisher pub_radar;
    integrator::param::IntegratorBaseParamsPtr gnss_param_ptr;

    explicit DatasetPohang(rclcpp::Node &node,
                           const std::shared_ptr<DatasetParam> &param,
                           integrator::param::IntegratorBaseParamsPtr gnss_integrator_param,
                           fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager)
      : DatasetBase("Pohang", param, sensor_calib_manager, "/gx5/imu_calib/data", "/gps/gps_nav_raw"),
        data_stereo_pair("Stereo", "", 0, FGODataTimeGetter<StereoPair>),
        data_baseline("Baseline", "/gx5/baseline", 0, FGODataTimeGetter<Pose>),
        data_lidar_front("LiDARFront", "/lidar_front/os_cloud_node/points", 0,
                         ROSMessageTimeGetter<sensor_msgs::msg::PointCloud2>),
        data_imu_lidar_front("IMULiDARFront", "/lidar_front/imu/data", 0, IMUDataTimeGetter),
        data_infrared("InfraRed", "/infrared/image", 0, ROSMessageTimeGetter<cv_bridge::CvImage>),
        data_radar("Radar", "/radar/image", 0, ROSMessageTimeGetter<cv_bridge::CvImage>),
        it(rclcpp::Node::SharedPtr(&node)),
        gnss_param_ptr(gnss_integrator_param) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": preparing dataset ...");
      pub_gnss_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/pohang/gnss/navfix",
                                                                     rclcpp::SystemDefaultsQoS());

      pub_baseline_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/pohang/baseline/navfix",
                                                                         rclcpp::SystemDefaultsQoS());

      pub_lidar_front = node.create_publisher<sensor_msgs::msg::PointCloud2>("/pohang/lidar/front",
                                                                             rclcpp::SystemDefaultsQoS());

      auto func_on_gnss = [this](const std::vector<PVASolution> &data) -> void {
        for (const auto &pva: data) {
          sensor_msgs::msg::NavSatFix navfix;
          navfix.header.stamp = pva.timestamp;
          navfix.latitude = pva.llh[0] * constants::rad2deg;
          navfix.longitude = pva.llh[1] * constants::rad2deg;
          navfix.altitude = pva.llh[2];
          pub_gnss_->publish(navfix);
        }
      };
      data_reference.setCbOnData(func_on_gnss);

      auto func_on_baseline = [this](const std::vector<Pose> &data) -> void {
        for (const auto &p: data) {
          sensor_msgs::msg::NavSatFix navfix;
          const auto llh = utils::xyz2llh(p.pose.translation());
          navfix.header.stamp = p.timestamp;
          navfix.header.frame_id = "imu";
          navfix.latitude = llh.x() * constants::rad2deg;
          navfix.longitude = llh.y() * constants::rad2deg;
          navfix.altitude = llh.z();
          pub_baseline_->publish(navfix);
        }
      };
      data_baseline.setCbOnData(func_on_baseline);

      pub_left_raw = it.advertise("pohang/zed/left", 1);
      pub_right_raw = it.advertise("pohang/zed/right", 1);
      pub_infrared = it.advertise("pohang/infrared", 1);
      auto func_on_stereopair = [this](const std::vector<StereoPair> &pairs) -> void {
        for (const auto &pair: pairs) {
          if (pair.left)
            pub_left_raw.publish(pair.left->toImageMsg());

          if (pair.right)
            pub_right_raw.publish(pair.right->toImageMsg());
        }
      };
      //data_stereo_pair.setCbLoadData();
      data_stereo_pair.setCbOnData(func_on_stereopair);

      auto func_load_stereo_data = [this](int64_t time_start_nanosec,
                                          double max_size,
                                          const std::string &data_topic) -> std::tuple<bool, std::map<rclcpp::Time, StereoPair>> {

        std::map<rclcpp::Time, StereoPair> data_map;
        static std::string topic_left = "/stereo_cam/left_img/compressed";
        static std::string topic_right = "/stereo_cam/right_img/compressed";
        static std::vector<std::string> stereo_topics = {topic_left,
                                                         topic_right};

        bool bag_bas_next;
        const auto stereo_raw_buffer_map = this->reader_->readPartialDataFromBag(stereo_topics,
                                                                                 time_start_nanosec,
                                                                                 max_size,
                                                                                 bag_bas_next);
        const auto &left_raw = stereo_raw_buffer_map.at(topic_left);
        const auto &right_raw = stereo_raw_buffer_map.at(topic_right);

        if (left_raw.size() != right_raw.size()) {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("offline_fgo"),
                             "OfflineFGO Dataset " << name << ": left image with size " << left_raw.size()
                                                   << " does not match the right image with size " << right_raw.size());
        } else
          RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                             "OfflineFGO Dataset " << name << ": parsing stereo pairs of size " << left_raw.size());

        for (size_t i = 0; i < std::min(left_raw.size(), right_raw.size()); i++) {
          const auto &left = left_raw[i];
          const auto &right = right_raw[i];

          const auto [left_time, left_img] = this->readROSMessage<sensor_msgs::msg::CompressedImage>(left);
          const auto [right_time, right_img] = this->readROSMessage<sensor_msgs::msg::CompressedImage>(right);

          StereoPair pair;
          pair.timestamp = left_time;
          pair.left = cv_bridge::toCvCopy(left_img);
          pair.right = cv_bridge::toCvCopy(right_img);
          data_map.insert(std::make_pair(left_time, pair));
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                           "OfflineFGO Dataset " << name << ": got stereo pairs done. Is bag finished? "
                                                 << (bag_bas_next ? "Yes" : "No"));
        return {bag_bas_next, data_map};
      };
      data_stereo_pair.setCbLoadData(func_load_stereo_data);

      auto func_load_cvImage_data = [this](int64_t time_start_nanosec,
                                            double max_size,
                                            const std::string &data_topic) -> std::tuple<bool, std::map<rclcpp::Time, cv_bridge::CvImage>> {
        std::map<rclcpp::Time, cv_bridge::CvImage> data_map;
        bool bag_bas_next;
        const auto raw_buffer_map = this->reader_->readSinglePartialDataFromBag(data_topic,
                                                                                time_start_nanosec,
                                                                                max_size,
                                                                                bag_bas_next);
        for (const auto & timestamp_buffer_pair : raw_buffer_map) {
          const auto [timestamp, img] = this->readROSMessage<sensor_msgs::msg::Image>(timestamp_buffer_pair);
          auto cvImgPtr = cv_bridge::toCvCopy(img);
          data_map.insert(std::make_pair(timestamp, *cvImgPtr));
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                           "OfflineFGO Dataset " << name << ": got " << data_topic << " done. Is bag finished? "
                                                 << (bag_bas_next ? "Yes" : "No"));
        return {bag_bas_next, data_map};
      };


      auto func_on_infrared = [this](const std::vector<cv_bridge::CvImage> &images) -> void {
        for (const auto &image: images) {
          //ToDo: @Jiandong, please adjust the color format in the infrared image in order to make it visualizable
          pub_infrared.publish(image.toImageMsg());
        }
      };
      data_infrared.setCbLoadData(func_load_cvImage_data);
      data_infrared.setCbOnData(func_on_infrared);

      auto func_on_radar = [this](const std::vector<cv_bridge::CvImage> &images) -> void {
        for (const auto &image: images)
          pub_radar.publish(image.toImageMsg());
      };
      data_radar.setCbLoadData(func_load_cvImage_data);
      data_radar.setCbOnData(func_on_radar);

      auto func_on_pointcloud = [this](const std::vector<sensor_msgs::msg::PointCloud2> &pcs) -> void {
        for (const auto &pc: pcs)
          pub_lidar_front->publish(pc);
      };
      data_lidar_front.setCbOnData(func_on_pointcloud);
      parseDataFromBag();
    };

    void parseDataFromBag() override {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": Parsing raw data into data blocks ...");
      const auto raw_imu_msg = readROSMessages<sensor_msgs::msg::Imu>(data_imu.data_topic);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got imu");
      const auto raw_imu_baseline_msg = readROSMessages<geometry_msgs::msg::PoseStamped>(data_baseline.data_topic);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got ahrs baseline");
      const auto raw_gps_pvt_msg = readROSMessages<robognss_msgs::msg::PVT>(data_reference.data_topic);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got GNSS raw");
      const auto raw_lidar_front = readROSMessages<sensor_msgs::msg::PointCloud2>(data_lidar_front.data_topic);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": got lidar front raw");
      const auto raw_imu_lidarfront_msg = readROSMessages<sensor_msgs::msg::Imu>(data_imu_lidar_front.data_topic);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": got lidar front imu");

      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": start converting ...");
      static const auto transIMUFromBase = sensor_calib_manager_->getTransformationFromBase("imu");
      DataBlock<IMUMeasurement>::DataMap imu_map;
      for (const auto &time_msg_pair: raw_imu_msg) {
        imu_map.insert(std::make_pair(time_msg_pair.first, msg2IMUMeasurement(time_msg_pair.second, time_msg_pair.first,
                                                                              transIMUFromBase.rotation().matrix())));
      }
      data_imu.setData(imu_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": parsed IMU data ...");

      DataBlock<Pose>::DataMap baseline_map;
      for (const auto &time_msg_pair: raw_imu_baseline_msg) {
        baseline_map.insert(std::make_pair(time_msg_pair.first, msg2FGOPose(time_msg_pair.second)));
      }
      data_baseline.setData(baseline_map, true);

      static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
      DataBlock<PVASolution>::DataMap gps_map;
      DataBlock<State>::DataMap state_map;
      for (const auto &time_msg_pair: raw_gps_pvt_msg) {
        auto [pva, state] = sensor::gnss::parseRoboGNSSPVTMsg(time_msg_pair.second, transReferenceFromBase.translation());
        gps_map.insert(std::make_pair(time_msg_pair.first, pva));
        state_map.insert(std::make_pair(time_msg_pair.first, state));
      }
      data_reference.setData(gps_map, true);
      data_reference_state.setData(state_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": parsed gnss/reference PVA data ...");

      DataBlock<sensor_msgs::msg::PointCloud2>::DataMap lidar_front_map;
      for (const auto &time_msg_pair : raw_lidar_front)
      {
        lidar_front_map.insert(time_msg_pair);
      }
      data_lidar_front.setData(lidar_front_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": parsed lidar front data ...");

      static const auto transLiDARFrontIMUFromBase = sensor_calib_manager_->getTransformationFromBase("lidar_front_imu");
      DataBlock<IMUMeasurement>::DataMap imu_lidarfront_map;
      for (const auto &time_msg_pair: raw_imu_lidarfront_msg) {
        imu_lidarfront_map.insert(std::make_pair(time_msg_pair.first, msg2IMUMeasurement(time_msg_pair.second, time_msg_pair.first,
                                                                                         transLiDARFrontIMUFromBase.rotation().matrix())));
      }
      data_imu_lidar_front.setData(imu_lidarfront_map, true);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": parsed IMU data ...");

      setTimestampsFromReference();
      trimDataBlocks();
    }

    void trimDataBlocks() override {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": trimming datablocks to start timestamp "
                                               << std::fixed << timestamp_start.seconds() << " and end timestamp "
                                               << timestamp_end.seconds());
      data_imu.trimData(timestamp_start, timestamp_end);
      data_baseline.trimData(timestamp_start, timestamp_end);
      data_reference_state.trimData(timestamp_start, timestamp_end);
      data_reference.trimData(timestamp_start, timestamp_end);
    }

    data::State referenceToState(const PVASolution &reference) override {
      static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
      return sensor::gnss::PVASolutionToState(reference, transReferenceFromBase.translation());
    }

    PohangDataBatch getDataBefore(double timestamp,
                                  bool erase = false) {
      PohangDataBatch batch;
      batch.timestamp_end = timestamp;
      const auto ros_timestamp = utils::secondsToROSTime(timestamp);
      batch.imu = data_imu.getDataBefore(ros_timestamp, erase);
      batch.reference_pva = data_reference.getDataBefore(ros_timestamp, erase);
      batch.reference_state = data_reference_state.getDataBefore(ros_timestamp, erase);
      batch.stereo_pair = data_stereo_pair.getDataBefore(ros_timestamp, erase);
      batch.baseline = data_baseline.getDataBefore(ros_timestamp, erase);
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
      batch.baseline = data_baseline.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.lidar_front = data_lidar_front.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.imu_lidar_front = data_imu_lidar_front.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.infrared = data_infrared.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      batch.radar = data_radar.getDataBetween(ros_timestamp_start, ros_timestamp_end);
      return batch;
    }
  };


}
#endif //ONLINE_FGO_DATASETPOHANG_H
