#ifdef ENABLE_BEACON

#include "sensor/camera/Beacon.h"

#include <beacon_module/labeler/labeler.h>

#include "integrator/IntegratorBase.h"

#define BEACON_PIPELINE_USING_ONLINE_SETUP false

namespace sensors::Camera {

/* ##################### BeaconPipeline Helper ############################## */
/* ########################################################################## */
static rclcpp::Time toRosTimestamp(const beacon::Timestamp& time) {
    return rclcpp::Time(time.getData().first,
                        time.getData().second,
                        RCL_ROS_TIME);
}

/* ########################################################################## */
static sensor_msgs::msg::NavSatFix toPublishNavFix(
    const gtsam::Pose3 T_eb,
    const rclcpp::Time timestamp,
    const std::string& frame_id = "Beacon_base",
    const std::string& encoding = "bgr8") {
    gtsam::Point3 llh = fgo::utils::xyz2llh(T_eb.translation());

    sensor_msgs::msg::NavSatFix navfix_msg;
    navfix_msg.header.stamp = timestamp;
    navfix_msg.header.frame_id = frame_id;
    navfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    navfix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    navfix_msg.latitude = llh.x() * fgo::constants::rad2deg;
    navfix_msg.longitude = llh.y() * fgo::constants::rad2deg;
    navfix_msg.altitude = llh.z();

    return navfix_msg;
}

/* ########################################################################## */
static sensor_msgs::msg::Image toPublishImageMsg(
    const cv::Mat img,
    const rclcpp::Time timestamp,
    const std::string& frame_id = "DisplayImage",
    const std::string& encoding = "bgr8") {
    cv_bridge::CvImage cvimage{};
    cvimage.image = img;
    cvimage.header.stamp = timestamp;
    cvimage.header.frame_id = frame_id;
    cvimage.encoding = encoding;

    return *cvimage.toImageMsg();
}

/* ##################### BeaconPipeline ##################################### */
/* ########################################################################## */
void BeaconStereoPipeline::processImpl() {
#if BEACON_PIPELINE_USING_ONLINE_SETUP
    std::unique_lock lk(input_buffer_mtx_);

    input_buffer_cv_.wait(lk, [this]() {
        if (input_buffer_.empty() || optimize_input_state_map_.empty()) {
            // required data unavailable
            return false;
        }
        const auto timestamp = input_buffer_.front().left_timestamp.getSec();
        if (timestamp < optimize_input_state_map_.begin()->second ||
            timestamp > optimize_input_state_map_.rbegin()->second) {
            // output of window
            return false;
        }
        return true;
    });

#else
    if (input_buffer_.empty() || optimize_input_state_map_.empty()) {
        beacon_logger_->info(
            "BeaconStereoPipeline waiting for non-empty input_buffer_ and "
            "optimize_input_state_map_!");
        return;
    }
    const auto tmp_timestamp = input_buffer_.front().left_timestamp.getSec();
    if (tmp_timestamp < optimize_input_state_map_.begin()->second ||
        tmp_timestamp > optimize_input_state_map_.rbegin()->second) {
        return;
    }
#endif

    auto [left_img, left_timestamp, right_img, right_timestamp] =
        input_buffer_.front();
    auto rcl_timestamp = toRosTimestamp(left_timestamp);

    input_buffer_.pop_front();

#if BEACON_PIPELINE_USING_ONLINE_SETUP
    lk.unlock();
#endif

    beacon_logger_->info(
        "----------------------- New Frame -----------------------");

    if (!initialized_) {
        throw std::runtime_error(
            "BeaconPipeline have not been initialized! Call init() first!");
    }

    // query the T_wb for new frame
    auto query_T_wb = queryBasePoseFromOptPose(rcl_timestamp);
    auto T_wb = odom_->getTransformWB();
    if (!query_T_wb.queriedSuccess) {
        beacon_logger_->info("BeaconStereoPipeline query T_wb failed!");

    } else {
        // query success
        T_wb = beacon::Transform::fromGTsamPose(query_T_wb.pose);

        beacon_logger_->info(
            "BeaconStereoPipeline query T_wb for new frame using "
            "Optimal Poses {}!",
            T_wb);
    }

    // feed images to odom
    odom_->processImage(
        preprocessImage(left_img, left_timestamp, left_cam_, T_wb),
        preprocessImage(right_img, right_timestamp, right_cam_, T_wb));

    // get keyframe and save to buffer
    auto left_keyframe = odom_->getLastLeftKeyframe();
    auto right_keyframe = odom_->getLastRightKeyframe();
    if (keyframe_buffer_.empty() ||
        keyframe_buffer_.back().timestamp != left_keyframe->timestamp) {
        // we have a new keyframe
        keyframe_buffer_.emplace_back(left_keyframe,
                                      right_keyframe,
                                      left_keyframe->timestamp);
    }

    // publish curr frame base pose
    base_pub->publish(toPublishNavFix(
        anchor_T_ew.transformPoseFrom(odom_->getTransformWB().getPose3gtsam()),
        rcl_timestamp));
    // base_pub->publish(toPublishNavFix(anchor_T_ew, rcl_timestamp));

    // publish featured image
    if (odom_->getLastLeftImage()) {
        left_img_pub_->publish(toPublishImageMsg(
            left_drawer_.drawKeypoint(*odom_->getLastLeftImage()),
            toRosTimestamp(left_timestamp)));
    }
    if (odom_->getLastRightImage()) {
        right_img_pub_->publish(toPublishImageMsg(
            right_drawer_.drawKeypoint(*odom_->getLastRightImage()),
            toRosTimestamp(right_timestamp)));
    }
    if (odom_->getLastTrack()) {
        mono_img_pub_->publish(toPublishImageMsg(
            left_drawer_.drawMatchSingle(*odom_->getLastTrack()),
            toRosTimestamp(left_timestamp)));
    }
    if (odom_->getLastStereoTrack()) {
        stereo_img_pub_->publish(toPublishImageMsg(
            left_drawer_.drawMatchSingle(*odom_->getLastStereoTrack()),
            toRosTimestamp(left_timestamp)));
    }
}

/* ########################################################################## */
beacon::ImagePtr BeaconStereoPipeline::preprocessImage(
    cv::Mat img,
    const beacon::Timestamp& time,
    const beacon::CameraPtr& cam,
    const beacon::Transform& T_wb) {
    const auto cam_pose = T_wb * cam->getTransformBC();
    auto data = beacon::ImageData::create(img, time, cam, cam_pose);

    cam->undistortImage(*data);

    return data;
}

/* ########################################################################## */
void BeaconStereoPipeline::process(cv::Mat left_img,
                                   const beacon::Timestamp& left_timestamp,
                                   cv::Mat right_img,
                                   const beacon::Timestamp& right_timestamp) {
    std::unique_lock lk(input_buffer_mtx_);

    input_buffer_.emplace_back(left_img,
                               left_timestamp,
                               right_img,
                               right_timestamp);

#if BEACON_PIPELINE_USING_ONLINE_SETUP
    input_buffer_cv_.notify_one();
#else
    processImpl();
#endif
}

/* ########################################################################## */
void BeaconStereoPipeline::loadBeaconParameter(
    const beacon::ParameterHandler& param) {
    beacon_logger_->info("StereoPipeline loading parameters!");

    odom_->loadParameter(param);

    left_cam_ = param.getCameras().at(0);
    {
        std::stringstream ss;
        left_cam_->print(ss);
        beacon_logger_->info("StereoPipeline load Left Cam:");
        beacon_logger_->info("{}", ss.str());
    }

    right_cam_ = param.getCameras().at(1);
    {
        std::stringstream ss;
        right_cam_->print(ss);
        beacon_logger_->info("StereoPipeline load Right Cam:");
        beacon_logger_->info("{}", ss.str());
    }
}

/* ########################################################################## */
bool BeaconStereoPipeline::init() {
    beacon_logger_->info("BeaconPipeline Initializing!");

    bool res = true;

    // camera init
    if (!left_cam_) {
        beacon_logger_->warn("BeaconPipeline require a left camera!");

        res &= false;
    }
    if (!right_cam_) {
        beacon_logger_->warn("BeaconPipeline require a right camera!");

        res &= false;
    }
    T_bl = left_cam_->getTransformBC().getPose3gtsam();
    T_br = right_cam_->getTransformBC().getPose3gtsam();

    // odometry init
    if (!odom_) {
        beacon_logger_->warn("BeaconPipeline require a StereoOdometry!");

        res &= false;
    }
    if (!odom_->init()) {
        beacon_logger_->warn("BeaconPipeline cannot init the StereoOdometry!");

        res &= false;
    }

#if BEACON_PIPELINE_USING_ONLINE_SETUP
    process_thread_ = std::jthread([this]() {
        while (true) {
            processImpl();
        }
    });
#endif

    if (res) {
        initialized_ = true;
    } else {
        initialized_ = false;
        beacon_logger_->info("BeaconPipeline Initialize Failed!");
    }

    return res;
}

/* ########################################################################## */
BeaconStereoPipeline::BeaconStereoPipeline(
    rclcpp::Node& node,
    fgo::integrator::param::IntegratorVisualParamsPtr integratorParamPtr)
    : initialized_(false),
      get_first_data_(false),
      node_(node),
      integratorParamPtr_(integratorParamPtr),
      odom_{beacon::StereoOdometry::create()},
      beacon_logger_{beacon::BeaconLogger::get()} {
    // ------------------------------ init ROS2 --------------------------------
    left_img_pub_ = node_.create_publisher<sensor_msgs::msg::Image>(
        "/gnss_fgo/beacon/left_feature_image",
        1);
    right_img_pub_ = node_.create_publisher<sensor_msgs::msg::Image>(
        "/gnss_fgo/beacon/right_feature_image",
        1);
    mono_img_pub_ = node_.create_publisher<sensor_msgs::msg::Image>(
        "/gnss_fgo/beacon/monocular_match_image",
        1);
    stereo_img_pub_ = node_.create_publisher<sensor_msgs::msg::Image>(
        "/gnss_fgo/beacon/stereo_match_image",
        1);
    base_pub = node_.create_publisher<sensor_msgs::msg::NavSatFix>(
        "/gnss_fgo/beacon/base",
        1);

    // ------------------------------ init gnssfgo ----------------------------
    if (integratorParamPtr_->gpType == fgo::data::GPModelType::WNOJ) {
        interpolator_ = std::make_shared<fgo::models::GPWNOJInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(
                integratorParamPtr_->QcGPInterpolatorFull),
            0,
            0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor,
            integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
    } else if (integratorParamPtr_->gpType == fgo::data::GPModelType::WNOA) {
        interpolator_ = std::make_shared<fgo::models::GPWNOAInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(
                integratorParamPtr_->QcGPInterpolatorFull),
            0,
            0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor,
            integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
    } else {
        RCLCPP_WARN(node_.get_logger(), "NO gpType chosen. Please choose.");
    }

    // ------------------------------ init Beacon ------------------------------
    right_drawer_.registerColorWithFeaturePred(
        beacon::ImageDrawer::blue,
        [](const beacon::VisualFeaturePoint& pt) -> bool {
            return beacon::ConstFeatureLabelAdaptor(pt).is(
                beacon::FeatureLabelEnum::StereoLowParallax);
        });
}

/* ########################################################################## */
void BeaconStereoPipeline::updateOptPose(
    size_t id,
    const rclcpp::Time& timestamp,
    const fgo::data::QueryStateInput& input) {
    std::unique_lock lk(optimize_state_mtx_);

    auto iter = optimized_state_map_.find(id);
    if (iter == optimized_state_map_.end()) {
        optimized_state_map_.insert(
            std::make_pair(id, std::make_pair(timestamp, input)));
    }
}

/* ########################################################################## */
void BeaconStereoPipeline::updateKeyIndexTimestampMap(
    const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap&
        current_all_key_lag) {
    optimize_input_state_map_ = current_all_key_lag;
    std::unique_lock lk(optimize_state_mtx_);

    // remove all optimized_state if it doesnt present in curr input states
    auto optPoseMapIter = optimized_state_map_.begin();
    while (optPoseMapIter != optimized_state_map_.end()) {
        auto currentIndexIter =
            optimize_input_state_map_.find(optPoseMapIter->first);
        if (currentIndexIter == optimize_input_state_map_.end()) {
            optPoseMapIter = optimized_state_map_.erase(optPoseMapIter);
            continue;
        }
        optPoseMapIter++;
    }
}

/* ########################################################################## */
fgo::data::QueryStateOutput BeaconStereoPipeline::queryBasePoseFromOptPose(
    const rclcpp::Time& timestamp) {
    using namespace fgo::integrator;
    using namespace gtsam::symbol_shorthand;

    beacon_logger_->info(
        "BeaconPipeline queryBasePoseFromOptPose query timestamp {}",
        timestamp.seconds());

    std::unique_lock lk(optimize_state_mtx_);

    gtsam::Pose3 query_T_eb;  // transform ecef to base

    auto syncResult = fgo::integrator::IntegratorBase::findStateForMeasurement(
        optimize_input_state_map_,
        timestamp.seconds(),
        integratorParamPtr_);

    if (syncResult.status != StateMeasSyncStatus::DROPPED &&
        syncResult.status != StateMeasSyncStatus::CACHED) {
        beacon_logger_->info(
            "BeaconPipeline queryBasePoseFromOptPose find State I {} with "
            "timestamp {} and synchronized: {}",
            syncResult.keyIndexI,
            syncResult.timestampI,
            syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I);
        beacon_logger_->info(
            "BeaconPipeline queryBasePoseFromOptPose find State J {} with "
            "timestamp {} and synchronized: {}?",
            syncResult.keyIndexJ,
            syncResult.timestampJ,
            syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J);
    }

    fgo::data::QueryStateOutput output;
    output.timestampCurrent = timestamp;
    output.keyIndexI = syncResult.keyIndexI;
    output.keyIndexJ = syncResult.keyIndexJ;
    output.timestampI = syncResult.timestampI;
    output.timestampJ = syncResult.timestampJ;
    output.durationI = syncResult.durationFromStateI;
    output.queriedSuccess = true;
    if (syncResult.status == StateMeasSyncStatus::DROPPED ||
        syncResult.status == StateMeasSyncStatus::CACHED) {
        beacon_logger_->info(
            "BeaconPipeline queryBasePoseFromOptPose query Failed!");

        output.queriedSuccess = false;
        return output;
    }

    if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I) {
        output.keySynchronized = true;
        query_T_eb = optimized_state_map_[syncResult.keyIndexI].second.pose;
    }
    if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J) {
        output.keySynchronized = true;
        query_T_eb = optimized_state_map_[syncResult.keyIndexJ].second.pose;
    }
    if (syncResult.status == StateMeasSyncStatus::INTERPOLATED) {
        output.keySynchronized = false;
        auto inputI = optimized_state_map_[syncResult.keyIndexI].second;
        auto inputJ = optimized_state_map_[syncResult.keyIndexJ].second;

        bool stateJExist =
            syncResult.timestampJ < std::numeric_limits<double>::max();

        if (!stateJExist) {
            output.queriedSuccess = false;
            return output;
        }

        const double delta_t = syncResult.timestampJ - syncResult.timestampI;
        if (integratorParamPtr_->gpType == fgo::data::GPModelType::WNOJ) {
            interpolator_->recalculate(delta_t,
                                       syncResult.durationFromStateI,
                                       inputI.acc,
                                       inputJ.acc);
            output.accI = inputI.acc;
            output.accJ = inputJ.acc;
        } else {
            interpolator_->recalculate(delta_t, syncResult.durationFromStateI);
        }

        query_T_eb = interpolator_->interpolatePose(inputI.pose,
                                                    inputI.vel,
                                                    inputI.omega,
                                                    inputJ.pose,
                                                    inputJ.vel,
                                                    inputJ.omega);
    }

    output.poseIMUECEF = query_T_eb;
    // odomInfoMsg_->associated_state_timestamp_i = syncResult.timestampI;
    // odomInfoMsg_->associated_state_timestamp_j = syncResult.timestampJ;
    // odomInfoMsg_->duration_to_i = syncResult.durationFromStateI;
    // odomInfoMsg_->associated_with_i =
    //     syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I;
    // odomInfoMsg_->associated_with_j =
    //     syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J;

    if (!get_first_data_) {
        anchor_T_ew = query_T_eb;
        anchor_T_we = query_T_eb.inverse();
        anchor_t_ew = anchor_T_ew.translation();
        anchor_R_ne = gtsam::Rot3(fgo::utils::enuRe_Matrix(anchor_t_ew));
        anchor_yaw_nw = anchor_R_ne.compose(query_T_eb.rotation()).yaw();
        anchor_R_wn = anchor_R_ne.compose(query_T_eb.rotation()).inverse();

        // odomInfoMsg_->odom_anchor_ecef_pos.x = posAnchorInitECEF_.x();
        // odomInfoMsg_->odom_anchor_ecef_pos.y = posAnchorInitECEF_.y();
        // odomInfoMsg_->odom_anchor_ecef_pos.z = posAnchorInitECEF_.z();
        // odomInfoMsg_->odom_anchor_ecef_to_enu_rpy.x = nRe.roll();
        // odomInfoMsg_->odom_anchor_ecef_to_enu_rpy.y = nRe.pitch();
        // odomInfoMsg_->odom_anchor_ecef_to_enu_rpy.z = nRe.yaw();
        // odomInfoMsg_->odom_anchor_yaw_offset = yawOffsetInitENU_;

        output.pose = gtsam::Pose3();

        get_first_data_ = true;

        return output;
    }

    // T_wb pose in local world
    output.pose = anchor_T_we.transformPoseFrom(query_T_eb);

    return output;
}

}  // namespace sensors::Camera
#endif
