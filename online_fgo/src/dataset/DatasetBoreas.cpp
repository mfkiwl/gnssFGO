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

#include "dataset/impl/DatasetBoreas.h"
#include "offline_process/OfflineFGOBase.h"

namespace fgo::dataset {

  void Boreas::initialize(std::string name_, offline_process::OfflineFGOBase &node,
                          fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager) {
    DatasetBase::initialize(name_, node, sensor_calib_manager);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": preparing dataset ...");

    param_->topic_type_map = {
      {"/boreas/gps_gt",           fgo::data::DataType::Odometry},
      {"/boreas/gps_raw",          fgo::data::DataType::Odometry},
      {"/boreas/imu/data",         fgo::data::DataType::IMU},
      {"/boreas/compressed_image", fgo::data::DataType::MonoImage},
      {"/boreas/velodyne_points",  fgo::data::DataType::PointCloud2},
      {"/boreas/camera_pose",      fgo::data::DataType::Pose},
      {"/boreas/lidar_pose",       fgo::data::DataType::Pose},
    };

    data_pva_gps = DataBlock<PVASolution>("PVAGNSS", "/boreas/gps_raw", param_->max_bag_memory_usage,
                                          PVADataTimeGetter),
    data_image = DataBlock<cv_bridge::CvImagePtr>("Image", "/boreas/compressed_image", param_->max_bag_memory_usage,
                                                  ROSMessagePtrTimeGetter<cv_bridge::CvImagePtr>),
    data_lidar_raw = DataBlock<sensor_msgs::msg::PointCloud2::SharedPtr>("LiDAR", "/boreas/velodyne_points",
                                                                         param_->max_bag_memory_usage,
                                                                         ROSMessagePtrTimeGetter<sensor_msgs::msg::PointCloud2::SharedPtr>),
    data_camera_pose = DataBlock<boreas_msgs::msg::SensorPose>("CameraPose", "/boreas/camera_pose",
                                                               param_->max_bag_memory_usage,
                                                               ROSMessageTimeGetter<boreas_msgs::msg::SensorPose>),
    data_lidar_pose = DataBlock<boreas_msgs::msg::SensorPose>("LiDARPose", "/boreas/lidar_pose",
                                                              param_->max_bag_memory_usage,
                                                              ROSMessageTimeGetter<boreas_msgs::msg::SensorPose>),
    it = std::make_shared<image_transport::ImageTransport>(rclcpp::Node::SharedPtr(&node));

    for (const auto &topic: param_->excluded_topics) {
      if (topic == data_image.data_topic)
        data_image.setExcluded(true);


      if (topic == data_lidar_raw.data_topic)
        data_lidar_raw.setExcluded(true);

      if (topic == data_camera_pose.data_topic)
        data_camera_pose.setExcluded(true);

      if (topic == data_lidar_pose.data_topic)
        data_lidar_pose.setExcluded(true);
    }

    pub_gnss_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/boreas/gps_nav_fix",
                                                                   rclcpp::SystemDefaultsQoS());
    pub_gt_ = node.create_publisher<sensor_msgs::msg::NavSatFix>("/boreas/gps_gt_fix", rclcpp::SystemDefaultsQoS());
    pub_lidar_ = node.create_publisher<sensor_msgs::msg::PointCloud2>("/boreas/lidar", rclcpp::SystemDefaultsQoS());
    pub_image = it->advertise("boreas/image", 1);

    auto func_on_gt = [this](const std::vector<PVASolution> &data) -> void {
      for (const auto &pva: data) {
        sensor_msgs::msg::NavSatFix navfix;
        navfix.header.stamp = pva.timestamp;
        navfix.latitude = pva.llh[0] * constants::rad2deg;
        navfix.longitude = pva.llh[1] * constants::rad2deg;
        navfix.altitude = pva.llh[2];
        pub_gt_->publish(navfix);
      }
    };

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
    data_reference.setCbOnData(func_on_gt);
    data_pva_gps.setCbOnData(func_on_gnss);

    auto func_on_lidar = [this](const std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> &data) -> void {
      for (const auto &pc: data)
        pub_lidar_->publish(*pc);
    };
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
    data_lidar_raw.setCbLoadData(func_load_lidar_data);
    data_lidar_raw.setCbOnData(func_on_lidar);

    auto func_load_cvImage_data = [this](int64_t time_start_nanosec,
                                         double max_size,
                                         const std::string &data_topic) -> std::tuple<bool, std::map<rclcpp::Time, cv_bridge::CvImagePtr>> {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": starting loading image data");
      std::map<rclcpp::Time, cv_bridge::CvImagePtr> data_map;
      bool bag_bas_next;
      const auto raw_buffer_map = this->reader_->readSinglePartialDataFromBag(data_topic,
                                                                              time_start_nanosec,
                                                                              max_size,
                                                                              bag_bas_next);
      for (const auto &timestamp_buffer_pair: raw_buffer_map) {
        const auto [timestamp, img] = this->readROSMessage<sensor_msgs::msg::CompressedImage>(timestamp_buffer_pair);
        auto cvImgPtr = cv_bridge::toCvCopy(img);
        data_map.insert(std::make_pair(timestamp, cvImgPtr));
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                         "OfflineFGO Dataset " << name << ": got " << data_topic << " done. Is bag finished? "
                                               << (bag_bas_next ? "Yes" : "No"));
      return {bag_bas_next, data_map};
    };
    auto func_on_image = [this](const std::vector<cv_bridge::CvImagePtr> &images) -> void {
      for (const auto &image: images) {
        //ToDo: @Jiandong, please adjust the color format in the infrared image in order to make it visualizable
        pub_image.publish(image->toImageMsg());
      }
    };
    data_image.setCbOnData(func_on_image);
    data_image.setCbLoadData(func_load_cvImage_data);
    parseDataFromBag();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": prepared!");
  }

  void Boreas::parseDataFromBag() {

    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": Parsing raw data into data blocks ...");
    const auto raw_imu_msg = readROSMessages<sensor_msgs::msg::Imu>(data_imu.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got imu");
    const auto raw_gps_pva_msg = readROSMessages<nav_msgs::msg::Odometry>(data_pva_gps.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got odometry");
    const auto raw_gt_pva_msg = readROSMessages<nav_msgs::msg::Odometry>(data_reference.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got odometry");
    const auto raw_camera_pose_msg = readROSMessages<boreas_msgs::msg::SensorPose>(data_camera_pose.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got camera pose");
    const auto raw_lidar_pose_msg = readROSMessages<boreas_msgs::msg::SensorPose>(data_lidar_pose.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got lidar pose");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": start converting ...");

    const auto preRotationIMU = sensor_calib_manager_->getPreRotation("imu");
    DataBlock<IMUMeasurement>::DataMap imu_map;
    for (const auto &time_msg_pair: raw_imu_msg) {
      imu_map.insert(std::make_pair(time_msg_pair.first,
                                    convertIMUMsgToIMUMeasurement(time_msg_pair.second, time_msg_pair.first,
                                                                  preRotationIMU.matrix())));
    }
    data_imu.setData(imu_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed IMU data ...");

    const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
    const auto transGNSSFromBase = sensor_calib_manager_->getTransformationFromBase("gnss");

    DataBlock<PVASolution>::DataMap gps_map;
    for (const auto &time_msg_pair: raw_gps_pva_msg) {
      auto [pva, state] = sensor::gnss::parseOdomMsg(time_msg_pair.second, time_msg_pair.first,
                                                     transGNSSFromBase.translation(), preRotationIMU);
      gps_map.insert(std::make_pair(time_msg_pair.first, pva));
    }
    data_pva_gps.setData(gps_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed gnss PVA data ...");

    DataBlock<PVASolution>::DataMap gt_gps_map;
    DataBlock<State>::DataMap gt_state_map;
    for (const auto &time_msg_pair: raw_gt_pva_msg) {
      auto [pva, state] = sensor::gnss::parseOdomMsg(time_msg_pair.second, time_msg_pair.first,
                                                     transReferenceFromBase.translation(),
                                                     preRotationIMU);
      gt_gps_map.insert(std::make_pair(time_msg_pair.first, pva));
      gt_state_map.insert(std::make_pair(time_msg_pair.first, state));
    }
    data_reference_state.setData(gt_state_map, true);
    data_reference.setData(gt_gps_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed Reference gnss data ...");

    DataBlock<boreas_msgs::msg::SensorPose>::DataMap camera_pose;
    DataBlock<boreas_msgs::msg::SensorPose>::DataMap lidar_pose;

    for (const auto &time_msg_pair: raw_camera_pose_msg) {
      camera_pose.insert(time_msg_pair);
    }
    data_camera_pose.setData(camera_pose);

    for (const auto &time_msg_pair: raw_lidar_pose_msg) {
      lidar_pose.insert(time_msg_pair);
    }
    data_lidar_pose.setData(camera_pose);

    setTimestampsFromReference();
    trimDataBlocks();
  }

  fgo::graph::StatusGraphConstruction Boreas::feedDataToGraph(const vector<double> &stateTimestamps) {
    const auto &last_timestamp = stateTimestamps.back();
    RCLCPP_INFO(appPtr_->get_logger(), "Feeding Boreas Data");
    // next step we get all data from the database

    auto data_batch = this->getDataBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got imu of size " << data_batch.imu.size());
    appPtr_->propagateIMU(data_batch.imu);
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got gt of size " << data_batch.reference_pva.size());
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got gps of size " << data_batch.gnss.size());
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got lidar of size " << data_batch.lidar_raw.size());
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got image of size " << data_batch.image.size());
    auto gnss_integrator_base = appPtr_->getGraphPtr()->getIntegrator("BoreasGNSSLCIntegrator");
    auto gnss_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator>&>(gnss_integrator_base);
    gnss_integrator->feedRAWData(data_batch.reference_pva, data_batch.reference_state);
    appPtr_->updateReferenceBuffer(data_batch.reference_pva);
    return appPtr_->getGraphPtr()->constructFactorGraphOnTime(stateTimestamps, data_batch.imu);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::dataset::Boreas, fgo::dataset::DatasetBase)