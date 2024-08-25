#include "integrator/BeaconIntegrator.h"

// #include "factor/visual/GPInterpolatedInvDepthFactor.h"
// #include "factor/visual/GPInterpolatedProjectFactor.h"
#include "factor/visual/GPInterpolatedProjectDepthOnlyFactor.h"
#include "factor/visual/GPInterpolatedSingleProjectDepthOnlyFactor.h"
#include "factor/visual/ProjectDepthOnlyFactor.h"
#include "integrator/BeaconIntegrator.h"

namespace fgo::integrator {

void BeaconIntegrator::processStereo(const fgo::data::StereoPair& data) {
    auto timestamp = beacon::Timestamp(data.timestamp.seconds());

    pipeline_->process(data.left->image,
                       timestamp,
                       data.right->image,
                       timestamp);
}

bool BeaconIntegrator::addFactors(
    const boost::circular_buffer<std::pair<double, gtsam::Vector3>>&
        timestampGyroMap,
    const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>&
        stateIDAccMap,
    const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap&
        currentKeyIndexTimestampMap,
    std::vector<std::pair<rclcpp::Time, fgo::data::State>>& timePredStates,
    gtsam::Values& values,
    fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
    gtsam::KeyVector& relatedKeys) {
    // get all frame within interval
    // build factor from matched result
    return true;
}

bool BeaconIntegrator::fetchResult(
    const gtsam::Values& result,
    const gtsam::Marginals& martinals,
    const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap&
        keyIndexTimestampMap,

    // update all updated state
    fgo::data::State& optState) {
    for (const auto& keyIndexTs : keyIndexTimestampMap) {
        fgo::data::QueryStateInput input;
        input.pose = result.at<gtsam::Pose3>(X(keyIndexTs.first));
        input.vel = result.at<gtsam::Vector3>(V(keyIndexTs.first));
        if (paramPtr_->addGPPriorFactor || paramPtr_->addGPInterpolatedFactor) {
            input.omega = result.at<gtsam::Vector3>(W(keyIndexTs.first));
            input.acc = optState.accMeasured;
        }
        pipeline_->updateOptPose(
            keyIndexTs.first,
            rclcpp::Time(keyIndexTs.second * fgo::constants::sec2nanosec,
                         RCL_ROS_TIME),
            input);
    }

    pipeline_->updateKeyIndexTimestampMap(keyIndexTimestampMap);

    return true;
}

void BeaconIntegrator::initialize(rclcpp::Node& node,
                                  fgo::graph::GraphBase& graphPtr,
                                  const std::string& integratorName,
                                  bool isPrimarySensor) {
    IntegratorBase::initialize(node, graphPtr, integratorName, isPrimarySensor);

    RCLCPP_INFO_STREAM(
        rosNodePtr_->get_logger(),
        "--------------------- "
            << integratorName
            << ": start initialization... ---------------------");

    // init beacon using config file
    ::utils::RosParameter<std::string> beacon_config_path_param(
        "GNSSFGO." + integratorName_ + ".beaconPipelineConfigPath",
        "/path/to/beacon_config",
        *rosNodePtr_);
    auto beacon_config_path = beacon_config_path_param.value();
    beacon::ParameterHandler ph{beacon_config_path};

    paramPtr_ =
        std::make_shared<IntegratorVisualParams>(integratorBaseParamPtr_);
    pipeline_ =
        std::make_shared<sensors::Camera::BeaconStereoPipeline>(node,
                                                                paramPtr_);
    pipeline_->loadBeaconParameter(ph);
    pipeline_->init();
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(fgo::integrator::BeaconIntegrator,
                       fgo::integrator::IntegratorBase)

}  // namespace fgo::integrator
