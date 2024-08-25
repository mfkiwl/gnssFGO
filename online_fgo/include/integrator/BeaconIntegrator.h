#ifdef ENABLE_BEACON

#ifndef ONLINE_FGO_BEACONINTEGRATOR_H
#define ONLINE_FGO_BEACONINTEGRATOR_H

#include <deque>

#include "VisualIntegratorBase.h"
#include "sensor/camera/Beacon.h"

namespace fgo::integrator {

class BeaconIntegrator : public VisualIntegratorBase {
  public:
    void initialize(rclcpp::Node& node,
                    fgo::graph::GraphBase& graphPtr,
                    const std::string& integratorName,
                    bool isPrimarySensor = false) override;

    bool addFactors(
        const boost::circular_buffer<std::pair<double, gtsam::Vector3>>&
            timestampGyroMap,
        const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>&
            stateIDAccMap,
        const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap&
            currentKeyIndexTimestampMap,
        std::vector<std::pair<rclcpp::Time, fgo::data::State>>& timePredStates,
        gtsam::Values& values,
        fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
        gtsam::KeyVector& relatedKeys) override;

    bool fetchResult(const gtsam::Values& result,
                     const gtsam::Marginals& martinals,
                     const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap&
                         keyIndexTimestampMap,
                     fgo::data::State& optState) override;

    void processStereo(const fgo::data::StereoPair& data) override;

    void processMonocular(const fgo::data::Image&) override {
        // do nothing
    }

  protected:
    std::shared_ptr<sensors::Camera::BeaconStereoPipeline> pipeline_{};

};  // class BeaconIntegrator

}  // namespace fgo::integrator

#endif

#endif
