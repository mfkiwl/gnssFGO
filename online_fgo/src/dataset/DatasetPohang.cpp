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

#include "dataset/impl/DatasetPohang.h"
#include "offline_process/OfflineFGOBase.h"

namespace fgo::dataset {

  void Pohang::initialize(std::string name_, offline_process::OfflineFGOBase &node,
                          fgo::sensor::SensorCalibrationManager::Ptr sensor_calib_manager) {
    DatasetBase::initialize(name_, node, sensor_calib_manager);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": preparing dataset ...");
    data_stereo_pair = DataBlock<StereoPair>("Stereo", "", param_->max_bag_memory_usage,
                                             FGODataTimeGetter<StereoPair>);
    data_baseline = DataBlock<Pose>("Baseline", "/gx5/baseline", 0, FGODataTimeGetter<Pose>);
    data_lidar_front = DataBlock<sensor_msgs::msg::PointCloud2::SharedPtr>("LiDARFront",
                                                                           "/lidar_front/os_cloud_node/points",
                                                                           param_->max_bag_memory_usage,
                                                                           ROSMessagePtrTimeGetter<sensor_msgs::msg::PointCloud2::SharedPtr>);
    data_imu_lidar_front = DataBlock<IMUMeasurement>("IMULiDARFront", "/lidar_front/imu/data", 0, IMUDataTimeGetter);
    data_infrared = DataBlock<cv_bridge::CvImagePtr>("InfraRed", "/infrared/image", param_->max_bag_memory_usage,
                                                     ROSMessagePtrTimeGetter<cv_bridge::CvImagePtr>);
    data_radar = DataBlock<cv_bridge::CvImagePtr>("Radar", "/radar/image", param_->max_bag_memory_usage,
                                                  ROSMessagePtrTimeGetter<cv_bridge::CvImagePtr>);
    it = std::make_shared<image_transport::ImageTransport>(rclcpp::Node::SharedPtr(&node));

    param_->topic_type_map = {
      {"/gps/gps_nav_raw",                  fgo::data::DataType::PVASolution},
      {"/gx5/baseline",                     fgo::data::DataType::Pose},
      {"/gx5/imu_calib/data",               fgo::data::DataType::IMU},
      {"/lidar_front/os_cloud_node/points", fgo::data::DataType::PointCloud2},
      {"/lidar_front/imu/data",             fgo::data::DataType::IMU},
      {"/infrared/image",                   fgo::data::DataType::MonoImage},
      {"/radar/image",                      fgo::data::DataType::MonoImage},
    };

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

    pub_left_raw = it->advertise("pohang/zed/left", 1);
    pub_right_raw = it->advertise("pohang/zed/right", 1);
    pub_infrared = it->advertise("pohang/infrared", 1);
    auto func_on_stereopair = [this](const std::vector<StereoPair> &pairs) -> void {
      for (const auto &pair: pairs) {
        if (pair.left)
          pub_left_raw.publish(pair.left->toImageMsg());

        if (pair.right)
          pub_right_raw.publish(pair.right->toImageMsg());
      }
    };
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
                                               << (!bag_bas_next ? "Yes" : "No"));
      return {bag_bas_next, data_map};
    };
    data_stereo_pair.setCbLoadData(func_load_stereo_data);

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
        pub_infrared.publish(image->toImageMsg());
      }
    };
    data_infrared.setCbLoadData(func_load_cvImage_data);
    data_infrared.setCbOnData(func_on_image);

    data_radar.setCbLoadData(func_load_cvImage_data);
    data_radar.setCbOnData(func_on_image);

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
        pub_lidar_front->publish(*pc);
    };
    data_lidar_front.setCbLoadData(func_load_lidar_data);
    data_lidar_front.setCbOnData(func_on_lidar);
    parseDataFromBag();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": prepared!");
  }

  void Pohang::parseDataFromBag()  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": Parsing raw data into data blocks ...");
    const auto raw_imu_msg = readROSMessages<sensor_msgs::msg::Imu>(data_imu.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got imu");
    const auto raw_imu_baseline_msg = readROSMessages<geometry_msgs::msg::PoseStamped>(data_baseline.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got ahrs baseline");
    const auto raw_gps_pvt_msg = readROSMessages<robognss_msgs::msg::PVT>(data_reference.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"), "OfflineFGO Dataset " << name << ": got GNSS raw");
    const auto raw_imu_lidarfront_msg = readROSMessages<sensor_msgs::msg::Imu>(data_imu_lidar_front.data_topic);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": got lidar front imu");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": start converting ...");
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

    DataBlock<Pose>::DataMap baseline_map;
    for (const auto &time_msg_pair: raw_imu_baseline_msg) {
      baseline_map.insert(std::make_pair(time_msg_pair.first, convertPoseStampedToFGOPose(time_msg_pair.second)));
    }
    data_baseline.setData(baseline_map, true);

    static const auto transReferenceFromBase = sensor_calib_manager_->getTransformationFromBase("reference");
    DataBlock<PVASolution>::DataMap gps_map;
    DataBlock<State>::DataMap state_map;
    for (const auto &time_msg_pair: raw_gps_pvt_msg) {
      auto [pva, state] = sensor::gnss::parseRoboGNSSPVTMsg(time_msg_pair.second,
                                                            transReferenceFromBase.translation());
      gps_map.insert(std::make_pair(time_msg_pair.first, pva));
      state_map.insert(std::make_pair(time_msg_pair.first, state));
    }
    data_reference.setData(gps_map, true);
    data_reference_state.setData(state_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed gnss/reference PVA data ...");

    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed lidar front data ...");

    static const auto transLiDARFrontIMUFromBase = sensor_calib_manager_->getTransformationFromBase(
      "lidar_front_imu");
    DataBlock<IMUMeasurement>::DataMap imu_lidarfront_map;
    for (const auto &time_msg_pair: raw_imu_lidarfront_msg) {
      imu_lidarfront_map.insert(
        std::make_pair(time_msg_pair.first, convertIMUMsgToIMUMeasurement(time_msg_pair.second, time_msg_pair.first,
                                                                          transLiDARFrontIMUFromBase.rotation().matrix())));
    }
    data_imu_lidar_front.setData(imu_lidarfront_map, true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("offline_process"),
                       "OfflineFGO Dataset " << name << ": parsed IMU data ...");

    setTimestampsFromReference();
    trimDataBlocks();
  }

  fgo::graph::StatusGraphConstruction Pohang::feedDataToGraph(const vector<double> &stateTimestamps) {
    const auto &last_timestamp = stateTimestamps.back();
    RCLCPP_INFO(appPtr_->get_logger(), "Feeding Pohang Data");
    auto data_batch = this->getDataBefore(last_timestamp, true);

    // ToDo: @Haoming, move the imu propagation into the IMUIntegrator
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got imu of size " << data_batch.imu.size());
    appPtr_->propagateIMU(data_batch.imu);
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got gt of size " << data_batch.reference_pva.size());
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got gps of size " << data_batch.gnss.size());
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got lidar of size " << data_batch.lidar_front.size());
    RCLCPP_INFO_STREAM(appPtr_->get_logger(), "got stereo_pair of size " << data_batch.stereo_pair.size());
    auto gnss_integrator_base = appPtr_->getGraphPtr()->getIntegrator("PohangGNSSLCIntegrator");
    auto gnss_integrator = reinterpret_cast<const std::shared_ptr<fgo::integrator::GNSSLCIntegrator>&>(gnss_integrator_base);
    gnss_integrator->feedRAWData(data_batch.reference_pva, data_batch.reference_state);
    appPtr_->updateReferenceBuffer(data_batch.reference_pva);
    return appPtr_->getGraphPtr()->constructFactorGraphOnTime(stateTimestamps, data_batch.imu);





    return graph::SUCCESSFUL;
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fgo::dataset::Pohang, fgo::dataset::DatasetBase)