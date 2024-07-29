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

#include "dataset/impl/DatasetKitti.h"
#include "offline_process/OfflineFGOBase.h"

namespace fgo::dataset {

  void Kitti::initialize(std::string name_, offline_process::OfflineFGOBase &node,
                         fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager)  {
    DatasetBase::initialize(name_, node, sensor_calib_manager);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": preparing dataset ...");

    param_->topic_type_map = {
      {"/tf2_msgs/msg/TFMessage",             fgo::data::DataType::Pose},
      {"/kitti/camera_gray_right/image_raw",  fgo::data::DataType::StereoImage},
      {"/kitti/camera_gray_left/image_raw",   fgo::data::DataType::StereoImage},
      {"/kitti/camera_color_left/image_raw",  fgo::data::DataType::StereoImage},
      {"/kitti/camera_color_right/image_raw", fgo::data::DataType::StereoImage},
      {"/kitti/oxts/gps/fix",                 fgo::data::DataType::PVASolution},
      {"/kitti/velo/pointcloud",              fgo::data::DataType::PointCloud2},
      {"/kitti/oxts/imu_unsynced",            fgo::data::DataType::IMU},
    };

    data_stereo_mono = DataBlock<StereoPair>("StereoMono", "", param_->max_bag_memory_usage, FGODataTimeGetter<StereoPair>);
    data_stereo_rgb = DataBlock<StereoPair>("StereoRGB", "", param_->max_bag_memory_usage, FGODataTimeGetter<StereoPair>);
    data_lidar = DataBlock<sensor_msgs::msg::PointCloud2::SharedPtr>("Velodyne", "", param_->max_bag_memory_usage,
                                                                     ROSMessagePtrTimeGetter<sensor_msgs::msg::PointCloud2::SharedPtr>);
    data_tf = DataBlock<Pose>("LocalPose", "/tf", param_->max_bag_memory_usage, FGODataTimeGetter<Pose>);
    it = std::make_shared<image_transport::ImageTransport>(rclcpp::Node::SharedPtr(&node));

    pub_gnss_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/kitti/gnss/navfix",
                                                                   rclcpp::SystemDefaultsQoS());
    pub_lidar_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("/kitti/velodyne",
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

    pub_left_mono = it->advertise("/kitti/mono/left", 1);
    pub_right_mono = it->advertise("/kitti/mono/right", 1);
    pub_left_rgb = it->advertise("/kitti/rgb/left", 1);
    pub_right_rgb = it->advertise("/kitti/rgb/right", 1);

    auto func_on_stereopair_mono = [this](const std::vector<StereoPair> &pairs) -> void {
      for (const auto &pair: pairs) {
        if (pair.left)
          pub_left_mono.publish(pair.left->toImageMsg());

        if (pair.right)
          pub_right_mono.publish(pair.right->toImageMsg());
      }
    };
    auto func_load_stereo_mono_data = [this](int64_t time_start_nanosec,
                                             double max_size,
                                             const std::string &data_topic) -> std::tuple<bool, std::map<rclcpp::Time, StereoPair>> {

      std::map<rclcpp::Time, StereoPair> data_map;
      static std::string topic_left = "/kitti/camera_gray_left/image_raw";
      static std::string topic_right = "/kitti/camera_gray_right/image_raw";
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

        const auto [left_time, left_img] = this->readROSMessage<sensor_msgs::msg::Image>(left);
        const auto [right_time, right_img] = this->readROSMessage<sensor_msgs::msg::Image>(right);

        StereoPair pair;
        pair.timestamp = left_time;
        pair.left = cv_bridge::toCvCopy(left_img);
        pair.right = cv_bridge::toCvCopy(right_img);
        data_map.insert(std::make_pair(left_time, pair));
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": got stereo pairs done. Is bag finished? "
                                               << (!bag_bas_next ? "Yes" : "No"));
      return {bag_bas_next, data_map};
    };
    data_stereo_mono.setCbOnData(func_on_stereopair_mono);
    data_stereo_mono.setCbLoadData(func_load_stereo_mono_data);

    auto func_on_stereopair_rgb = [this](const std::vector<StereoPair> &pairs) -> void {
      for (const auto &pair: pairs) {
        if (pair.left)
          pub_left_rgb.publish(pair.left->toImageMsg());

        if (pair.right)
          pub_right_rgb.publish(pair.right->toImageMsg());
      }
    };
    auto func_load_stereo_rgb_data = [this](int64_t time_start_nanosec,
                                            double max_size,
                                            const std::string &data_topic) -> std::tuple<bool, std::map<rclcpp::Time, StereoPair>> {

      std::map<rclcpp::Time, StereoPair> data_map;
      static std::string topic_left = "/kitti/camera_color_left/image_raw";
      static std::string topic_right = "/kitti/camera_color_left/image_raw";
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

        const auto [left_time, left_img] = this->readROSMessage<sensor_msgs::msg::Image>(left);
        const auto [right_time, right_img] = this->readROSMessage<sensor_msgs::msg::Image>(right);

        StereoPair pair;
        pair.timestamp = left_time;
        pair.left = cv_bridge::toCvCopy(left_img);
        pair.right = cv_bridge::toCvCopy(right_img);
        data_map.insert(std::make_pair(left_time, pair));
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": got stereo pairs done. Is bag finished? "
                                               << (!bag_bas_next ? "Yes" : "No"));
      return {bag_bas_next, data_map};
    };
    data_stereo_rgb.setCbOnData(func_on_stereopair_rgb);
    data_stereo_rgb.setCbLoadData(func_load_stereo_rgb_data);

    auto func_load_lidar_data = [this](int64_t time_start_nanosec,
                                       double max_size,
                                       const std::string &data_topic) -> std::tuple<bool, std::map<rclcpp::Time, sensor_msgs::msg::PointCloud2::SharedPtr>> {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": starting loading lidar data");
      std::map<rclcpp::Time, sensor_msgs::msg::PointCloud2::SharedPtr> data_map;
      bool bag_bas_next;
      const auto raw_buffer_map = this->reader_->readSinglePartialDataFromBag(data_topic,
                                                                              time_start_nanosec,
                                                                              max_size,
                                                                              bag_bas_next);
      for (const auto &timestamp_buffer_pair: raw_buffer_map) {
        const auto [timestamp, pc] = this->readROSMessage<sensor_msgs::msg::PointCloud2>(timestamp_buffer_pair);
        data_map.insert(std::make_pair(timestamp, std::make_shared<sensor_msgs::msg::PointCloud2>(pc)));
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": got " << data_topic << " done. Is bag finished? "
                                               << (!bag_bas_next ? "Yes" : "No"));
      return {bag_bas_next, data_map};
    };
    auto func_on_lidar = [this](const std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> &data) -> void {
      for (const auto &pc: data)
        pub_lidar_->publish(*pc);
    };
    data_lidar.setCbLoadData(func_load_lidar_data);
    data_lidar.setCbOnData(func_on_lidar);

    parseDataFromBag();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": prepared!");
  }

  void Kitti::parseDataFromBag()  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": Parsing raw data into data blocks ...");
    const auto raw_imu_msg = readROSMessages<sensor_msgs::msg::Imu>(data_imu.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got imu");
    const auto raw_gps_fix_msg = readROSMessages<sensor_msgs::msg::NavSatFix>("/kitti/oxts/gps/fix");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got navfix raw");
    const auto raw_gps_vel_msg = readROSMessages<geometry_msgs::msg::TwistStamped>("/kitti/oxts/gps/vel");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": got GNSS velocity raw");
    const auto raw_tf_msg = readROSMessagesNoHeader<tf2_msgs::msg::TFMessage>(data_tf.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": got GNSS velocity raw");

    static const auto preRotateIMU = sensor_calib_manager_->getPreRotation("imu");
    DataBlock<IMUMeasurement>::DataMap imu_map;
    for (const auto &time_msg_pair: raw_imu_msg) {
      imu_map.insert(std::make_pair(time_msg_pair.first,
                                    convertIMUMsgToIMUMeasurement(time_msg_pair.second, time_msg_pair.first,
                                                                  preRotateIMU.matrix())));
    }
    data_imu.setData(imu_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed IMU data ...");

    static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
    DataBlock<PVASolution>::DataMap gps_map;
    DataBlock<State>::DataMap state_map;

    if (raw_gps_fix_msg.size() == raw_gps_vel_msg.size()) {
      for (size_t i = 0; i < raw_gps_fix_msg.size(); i++) {
        auto [pva, state] = sensor::gnss::parseNavFixWithTwist(raw_gps_fix_msg[i].second, raw_gps_vel_msg[i].second,
                                                               raw_gps_fix_msg[i].first,
                                                               transReferenceFromBase.translation());
        gps_map.insert(std::make_pair(raw_gps_fix_msg[i].first, pva));
        state_map.insert(std::make_pair(raw_gps_fix_msg[i].first, state));
      }
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("offline_process"),
                          "OfflineFGO Dataset " << name << ": does not have matched gps nav and velocity ...");

      for (const auto &time_msg_pair: raw_gps_fix_msg) {
        auto [pva, state] = sensor::gnss::parseNavFixMsg(time_msg_pair.second, time_msg_pair.first,
                                                         transReferenceFromBase.translation());
        gps_map.insert(std::make_pair(time_msg_pair.first, pva));
        state_map.insert(std::make_pair(time_msg_pair.first, state));
      }

    }
    data_reference.setData(gps_map, true);
    data_reference_state.setData(state_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed gnss/reference PVA data ...");

    DataBlock<Pose>::DataMap tf_map;
    for (const auto &time_msg_pair: raw_imu_msg) {
      imu_map.insert(std::make_pair(time_msg_pair.first,
                                    convertIMUMsgToIMUMeasurement(time_msg_pair.second, time_msg_pair.first,
                                                                  preRotateIMU.matrix())));
    }
    data_imu.setData(imu_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed IMU data ...");


  }

  fgo::graph::StatusGraphConstruction Kitti::feedDataToGraph(const vector<double> &stateTimestamps) {
    return graph::SUCCESSFUL;
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::dataset::Kitti, fgo::dataset::DatasetBase)