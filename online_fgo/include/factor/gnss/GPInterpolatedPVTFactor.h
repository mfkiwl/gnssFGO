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
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//
//

#ifndef ONLINE_FGO_GPINTERPOLATEDPVTFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDPVTFACTOR_H

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>

#include "model/gp_interpolator/GPWNOAInterpolator.h"
#include "utils/NavigationTools.h"
#include "include/factor/FactorTypes.h"
#include "factor/FactorTypeIDs.h"
#include "third_party/matlab_utils.h"

namespace fgo::factor {
  class GPInterpolatedPVTFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
  protected:
    gtsam::Point3 pos_;
    gtsam::Vector3 vel_;
    gtsam::Vector3 lb_;
    MeasurementFrame velocityFrame_;
    double tau_{};

    typedef GPInterpolatedPVTFactor This;
    typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
    typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
    GPBase GPBase_;
    bool useAutoDiff_ = false;


  public:
    GPInterpolatedPVTFactor() = default;

    GPInterpolatedPVTFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                            const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                            const gtsam::Point3 &positionMeasured,
                            const gtsam::Vector3 &velocityMeasured,
                            const gtsam::Vector3 &lb,
                            MeasurementFrame velocityFrame,
                            const gtsam::SharedNoiseModel &model,
                            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                            bool useAutoDiff = false) : Base(model, poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ,
                                                             omegaKeyJ), pos_(positionMeasured), vel_(velocityMeasured),
                                                        lb_(lb), velocityFrame_(velocityFrame),
                                                        tau_(interpolator->getTau()), GPBase_(interpolator),
                                                        useAutoDiff_(useAutoDiff) {
      factorTypeID_ = FactorTypeID::GPPVT;
      factorName_ = "GPInterpolatedPVTFactor";
    }

    ~GPInterpolatedPVTFactor() override = default;

    /// @return a deep copy of this factor
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector
    evaluateError(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                  const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ,
                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                  boost::optional<gtsam::Matrix &> H6 = boost::none) const override {
      // NOT using auto. diff. due to numerical instability
      /*
    if(useAutoDiff_)
    {
      if(H1)
        *H1 = gtsam::numericalDerivative61<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H2)
        *H2 = gtsam::numericalDerivative62<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H3)
        *H3 = gtsam::numericalDerivative63<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H4)
        *H4 = gtsam::numericalDerivative64<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H5)
        *H5 = gtsam::numericalDerivative65<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H6)
        *H6 = gtsam::numericalDerivative66<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      return evaluateError_(poseI, velI, omegaI, poseJ, velJ, omegaJ);
    }
    else
    {*/
      gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hpose, Hrot, Hrot2, Hvelp, Hvelv;
      gtsam::Matrix Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V;

      gtsam::Pose3 pose;
      gtsam::Vector6 vel;

      if (H1 || H2 || H3 || H4 || H5 || H6) {
        pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ,
                                        Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
        vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, poseJ, velJ, omegaJ,
                                           Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V);
      } else {
        pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
        vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, poseJ, velJ, omegaJ);
      }

      const auto &rot = pose.rotation(Hrot);
      const auto posEva = pose.translation(Hpose) + rot.rotate(lb_, Hrot2);
      const auto lb_skew = gtsam::skewSymmetric(-lb_);
      const auto lbv = lb_skew * vel.head(3);
      const auto vel_e = rot.rotate(vel.tail(3) + lbv, Hvelp, Hvelv);
      const auto ePos = posEva - pos_;

      //  Shape 6x6                        3x6  * 6x6        3x3 *  3x6 *   6x6
      gtsam::Vector3 eVel;

      if (velocityFrame_ == MeasurementFrame::ECEF) {
        eVel = vel_e - vel_;
        if (H1)
          *H1 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint1_P, Hvelp * Hrot * Hint1_V +
                                                                        Hvelv).finished();    // Shape 6 x 6
        if (H4)
          *H4 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint4_P, Hvelp * Hrot * Hint4_V +
                                                                        Hvelv).finished();    // Shape 6 x 6
        if (H2) *H2 = (gtsam::Matrix63() << Hpose * Hint2_P, Hvelv * Hint2_V.block<3, 3>(3, 0)).finished();
        if (H3) *H3 = (gtsam::Matrix63() << Hpose * Hint3_P, Hvelv * (Hint2_V.block<3, 3>(0, 0))).finished();
        if (H5) *H5 = (gtsam::Matrix63() << Hpose * Hint5_P, Hvelv * Hint5_V.block<3, 3>(3, 0)).finished();
        if (H6) *H6 = (gtsam::Matrix63() << Hpose * Hint6_P, Hvelv * (Hint6_V.block<3, 3>(0, 0))).finished();
      } else if (velocityFrame_ == MeasurementFrame::NED) {
        const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(posEva));
        const auto jac = matlab_utils::jacobianECEF2NED(posEva, vel_e);
        eVel = nedRe.rotate(vel_e) - vel_;
        if (H1)
          *H1 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint1_P, jac.block<3, 3>(0, 0) *
                                                                        (Hvelp * Hrot * Hint1_V +
                                                                         Hvelv)).finished();    // Shape 6 x 6
        if (H4)
          *H4 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint4_P, jac.block<3, 3>(0, 0) *
                                                                        (Hvelp * Hrot * Hint4_V +
                                                                         Hvelv)).finished();    // Shape 6 x 6
        if (H2)
          *H2 = (gtsam::Matrix63() << Hpose * Hint2_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint2_V.block<3, 3>(3, 0)).finished();
        if (H3)
          *H3 = (gtsam::Matrix63() << Hpose * Hint3_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (Hint2_V.block<3, 3>(0, 0))).finished();
        if (H5)
          *H5 = (gtsam::Matrix63() << Hpose * Hint5_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint5_V.block<3, 3>(3, 0)).finished();
        if (H6)
          *H6 = (gtsam::Matrix63() << Hpose * Hint6_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (Hint6_V.block<3, 3>(0, 0))).finished();
      } else {
        const auto enuRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(posEva));
        eVel = enuRe.rotate(vel_e) - vel_;
        const auto jac = matlab_utils::jacobianECEF2ENU(posEva, vel_e);
        if (H1)
          *H1 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint1_P, jac.block<3, 3>(0, 0) * Hvelp * Hrot *
                                                                        Hint1_V).finished();    // Shape 6 x 6
        if (H4)
          *H4 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint4_P, jac.block<3, 3>(0, 0) * Hvelp * Hrot *
                                                                        Hint4_V).finished();    // Shape 6 x 6
        if (H2)
          *H2 = (gtsam::Matrix63() << Hpose * Hint2_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint2_V.block<3, 3>(3, 0)).finished();
        if (H3)
          *H3 = (gtsam::Matrix63() << Hpose * Hint3_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (lb_skew + Hint2_V.block<3, 3>(0, 0))).finished();
        if (H5)
          *H5 = (gtsam::Matrix63() << Hpose * Hint5_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint5_V.block<3, 3>(3, 0)).finished();
        if (H6)
          *H6 = (gtsam::Matrix63() << Hpose * Hint6_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (lb_skew + Hint6_V.block<3, 3>(0, 0))).finished();
      }
      return (gtsam::Vector6() << ePos, eVel).finished();
      //}
    }

    [[nodiscard]] gtsam::Vector
    evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ) const {
      const auto pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
      const auto vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, poseJ, velJ, omegaJ);
      const auto &eRb = pose.rotation();
      const auto ePos = pose.translation() + eRb.rotate(lb_) - pos_;
      const auto vel_e = eRb.rotate(vel.tail(3)); // + gtsam::skewSymmetric(-lb_) * vel.head(3));

      if (velocityFrame_ == MeasurementFrame::ECEF) {
        const auto eVel = vel_e - vel_;
        return (gtsam::Vector6() << ePos, eVel).finished();
      } else if (velocityFrame_ == MeasurementFrame::NED) {
        const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
        const auto eVel = nedRe.rotate(vel_e) - vel_;
        return (gtsam::Vector6() << ePos, eVel).finished();
      } else {
        const auto enuRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
        const auto eVel = enuRe.rotate(vel_e) - vel_;
        return (gtsam::Vector6() << ePos, eVel).finished();
      }
    }

    /** lifting all related state values in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) override {
      const auto poseI = values.at<gtsam::Pose3>(key1());
      const auto velI = values.at<gtsam::Vector3>(key2());
      const auto omegaI = values.at<gtsam::Vector3>(key3());
      const auto poseJ = values.at<gtsam::Pose3>(key4());
      const auto velJ = values.at<gtsam::Vector3>(key5());
      const auto omegaJ = values.at<gtsam::Vector3>(key6());
      const auto liftedStates = (gtsam::Vector(24) << poseI.rotation().rpy(),
        poseI.translation(),
        velI, omegaI,
        poseJ.rotation().rpy(),
        poseJ.translation(),
        velJ, omegaJ).finished();
      return liftedStates;
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) override {
      assert(state.size() != 24);
      gtsam::Values values;
      try {
        values.insert(key1(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(0, 0)),
                                           gtsam::Point3(state.block<3, 1>(3, 0))));
        values.insert(key2(), gtsam::Vector3(state.block<3, 1>(6, 0)));
        values.insert(key3(), gtsam::Vector3(state.block<3, 1>(9, 0)));
        values.insert(key4(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(12, 0)),
                                           gtsam::Point3(state.block<3, 1>(15, 0))));
        values.insert(key5(), gtsam::Vector3(state.block<3, 1>(18, 0)));
        values.insert(key6(), gtsam::Vector3(state.block<3, 1>(21, 0)));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << getName() << " cannot generate values from state vector " << state << " due to "
                  << ex.what() << std::endl;
      }
      return values;
    }

    /** return the measured */
    [[nodiscard]] std::pair<gtsam::Point3, gtsam::Vector3> measured() const {
      return {pos_, vel_};
    }

    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Point3() << this->pos_).finished(),
                                          (gtsam::Point3() << e->pos_).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "PVTFactor" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterpolatedPVTFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(pos_);
      ar & BOOST_SERIALIZATION_NVP(vel_);
    }

  };

  class GPInterpolatedPVTFactorFull
    : public NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> {
  protected:
    gtsam::Point3 pos_;
    gtsam::Vector3 vel_;
    gtsam::Vector3 lb_;
    MeasurementFrame velocityFrame_;
    double tau_{};

    typedef GPInterpolatedPVTFactorFull This;
    typedef NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> Base;
    typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
    GPBase GPBase_;
    bool useAutoDiff_ = false;


  public:
    GPInterpolatedPVTFactorFull() = default;

    GPInterpolatedPVTFactorFull(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                                const gtsam::Key &accI,
                                const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                                const gtsam::Key &accJ,
                                const gtsam::Point3 &positionMeasured,
                                const gtsam::Vector3 &velocityMeasured,
                                const gtsam::Vector3 &lb,
                                MeasurementFrame velocityFrame,
                                const gtsam::SharedNoiseModel &model,
                                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                bool useAutoDiff = false) : Base(model, poseKeyI, velKeyI, omegaKeyI, accI,
                                                                 poseKeyJ, velKeyJ, omegaKeyJ, accJ),
                                                            pos_(positionMeasured), vel_(velocityMeasured), lb_(lb),
                                                            velocityFrame_(velocityFrame),
                                                            tau_(interpolator->getTau()), GPBase_(interpolator),
                                                            useAutoDiff_(useAutoDiff) {
      factorTypeID_ = FactorTypeID::GPPVT;
      factorName_ = "GPInterpolatedPVTFactorFull";
    }

    ~GPInterpolatedPVTFactorFull() override = default;

    /// @return a deep copy of this factor
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector
    evaluateError(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                  const gtsam::Vector6 &accI,
                  const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ,
                  const gtsam::Vector6 &accJ,
                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                  boost::optional<gtsam::Matrix &> H4 = boost::none,
                  boost::optional<gtsam::Matrix &> H5 = boost::none,
                  boost::optional<gtsam::Matrix &> H6 = boost::none,
                  boost::optional<gtsam::Matrix &> H7 = boost::none,
                  boost::optional<gtsam::Matrix &> H8 = boost::none) const override {
      // NOT using auto. diff. due to numerical instability
      /*
    if(useAutoDiff_)
    {
      if(H1)
        *H1 = gtsam::numericalDerivative61<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H2)
        *H2 = gtsam::numericalDerivative62<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H3)
        *H3 = gtsam::numericalDerivative63<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H4)
        *H4 = gtsam::numericalDerivative64<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H5)
        *H5 = gtsam::numericalDerivative65<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      if(H6)
        *H6 = gtsam::numericalDerivative66<gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5, boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
      return evaluateError_(poseI, velI, omegaI, poseJ, velJ, omegaJ);
    }
    else
    {*/
      gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hint7_P, Hint8_P, Hpose, Hrot, Hrot2, Hvelp, Hvelv;
      gtsam::Matrix Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V, Hint7_V, Hint8_V;

      gtsam::Pose3 pose;
      gtsam::Vector6 vel;

      if (H1 || H2 || H3 || H4 || H5 || H6 || H7 || H8) {
        pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI,
                                        poseJ, velJ, omegaJ, accJ,
                                        Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hint7_P, Hint8_P);
        vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, accI,
                                           poseJ, velJ, omegaJ, accJ,
                                           Hint1_V, Hint2_V, Hint3_V, Hint4_V, Hint5_V, Hint6_V, Hint7_V, Hint8_V);
      } else {
        pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI,
                                        poseJ, velJ, omegaJ, accJ);
        vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, accI,
                                           poseJ, velJ, omegaJ, accJ);
      }

      const auto &rot = pose.rotation(Hrot);
      const auto posEva = pose.translation(Hpose) + rot.rotate(lb_, Hrot2);
      const auto lb_skew = gtsam::skewSymmetric(-lb_);
      const auto lbv = lb_skew * vel.head(3);
      const auto vel_e = rot.rotate(vel.tail(3) + lbv, Hvelp, Hvelv);
      const auto ePos = posEva - pos_;

      //  Shape 6x6                        3x6  * 6x6        3x3 *  3x6 *   6x6
      gtsam::Vector3 eVel;

      if (velocityFrame_ == MeasurementFrame::ECEF) {
        eVel = vel_e - vel_;
        if (H1)
          *H1 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint1_P, Hvelp * Hrot * Hint1_V +
                                                                        Hvelv).finished();    // Shape 6 x 6
        if (H2) *H2 = (gtsam::Matrix63() << Hpose * Hint2_P, Hvelv * Hint2_V.block<3, 3>(3, 0)).finished();
        if (H3) *H3 = (gtsam::Matrix63() << Hpose * Hint3_P, Hvelv * (Hint3_V.block<3, 3>(0, 0))).finished();
        if (H4) *H4 = (gtsam::Matrix66() << Hpose * Hint4_P, Hvelv * Hint4_V).finished();

        if (H5)
          *H5 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint5_P, Hvelp * Hrot * Hint5_V +
                                                                        Hvelv).finished();    // Shape 6 x 6
        if (H6) *H6 = (gtsam::Matrix63() << Hpose * Hint6_P, Hvelv * Hint6_V.block<3, 3>(3, 0)).finished();
        if (H7) *H7 = (gtsam::Matrix63() << Hpose * Hint7_P, Hvelv * (Hint7_V.block<3, 3>(0, 0))).finished();
        if (H8) *H8 = (gtsam::Matrix66() << Hpose * Hint8_P, Hvelv * Hint8_V).finished();
      } else if (velocityFrame_ == MeasurementFrame::NED) {
        const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(posEva));
        const auto jac = matlab_utils::jacobianECEF2NED(posEva, vel_e);
        eVel = nedRe.rotate(vel_e) - vel_;
        if (H1)
          *H1 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint1_P, jac.block<3, 3>(0, 0) *
                                                                        (Hvelp * Hrot * Hint1_V +
                                                                         Hvelv)).finished();    // Shape 6 x 6
        if (H2)
          *H2 = (gtsam::Matrix63() << Hpose * Hint2_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint2_V.block<3, 3>(3, 0)).finished();
        if (H3)
          *H3 = (gtsam::Matrix63() << Hpose * Hint3_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (Hint3_V.block<3, 3>(0, 0))).finished();
        if (H4)
          *H4 = (gtsam::Matrix66() << Hpose * Hint4_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint4_V).finished();
        if (H5)
          *H5 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint5_P, jac.block<3, 3>(0, 0) *
                                                                        (Hvelp * Hrot * Hint5_V +
                                                                         Hvelv)).finished();    // Shape 6 x 6
        if (H6)
          *H6 = (gtsam::Matrix63() << Hpose * Hint6_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (Hint6_V.block<3, 3>(0, 0))).finished();
        if (H7)
          *H7 = (gtsam::Matrix63() << Hpose * Hint7_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (Hint7_V.block<3, 3>(0, 0))).finished();
        if (H8)
          *H8 = (gtsam::Matrix66() << Hpose * Hint8_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint8_V).finished();
      } else if (velocityFrame_ == MeasurementFrame::ENU) {
        const auto enuRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(posEva));
        eVel = enuRe.rotate(vel_e) - vel_;
        const auto jac = matlab_utils::jacobianECEF2ENU(posEva, vel_e);
        if (H1)
          *H1 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint1_P, jac.block<3, 3>(0, 0) * Hvelp * Hrot *
                                                                        Hint1_V).finished();    // Shape 6 x 6
        if (H2)
          *H2 = (gtsam::Matrix63() << Hpose * Hint2_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint2_V.block<3, 3>(3, 0)).finished();
        if (H3)
          *H3 = (gtsam::Matrix63() << Hpose * Hint3_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (lb_skew + Hint3_V.block<3, 3>(0, 0))).finished();
        if (H4)
          *H4 = (gtsam::Matrix63() << Hpose * Hint4_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint4_V).finished();
        if (H5)
          *H5 = (gtsam::Matrix66() << (Hpose + Hrot2 * Hrot) * Hint5_P, jac.block<3, 3>(0, 0) * Hvelp * Hrot *
                                                                        Hint5_V).finished();    // Shape 6 x 6
        if (H6)
          *H6 = (gtsam::Matrix63() << Hpose * Hint6_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint6_V.block<3, 3>(3, 0)).finished();
        if (H7)
          *H7 = (gtsam::Matrix63() << Hpose * Hint7_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       (lb_skew + Hint7_V.block<3, 3>(0, 0))).finished();
        if (H8)
          *H8 = (gtsam::Matrix63() << Hpose * Hint8_P, jac.block<3, 3>(3, 0) * Hvelv *
                                                       Hint8_V).finished();
      }
      return (gtsam::Vector6() << ePos, eVel).finished();
      //}
    }

    [[nodiscard]] gtsam::Vector
    evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                   const gtsam::Vector6 &accI,
                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ,
                   const gtsam::Vector6 &accJ) const {
      const auto pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
      const auto vel = GPBase_->interpolateVelocity(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
      const auto &eRb = pose.rotation();
      const auto ePos = pose.translation() + eRb.rotate(lb_) - pos_;
      const auto vel_e = eRb.rotate(vel.tail(3)); // + gtsam::skewSymmetric(-lb_) * vel.head(3));

      if (velocityFrame_ == MeasurementFrame::ECEF) {
        const auto eVel = vel_e - vel_;
        return (gtsam::Vector6() << ePos, eVel).finished();
      } else if (velocityFrame_ == MeasurementFrame::NED) {
        const auto nedRe = gtsam::Rot3(fgo::utils::nedRe_Matrix(pose.translation()));
        const auto eVel = nedRe.rotate(vel_e) - vel_;
        return (gtsam::Vector6() << ePos, eVel).finished();
      } else {
        const auto enuRe = gtsam::Rot3(fgo::utils::enuRe_Matrix(pose.translation()));
        const auto eVel = enuRe.rotate(vel_e) - vel_;
        return (gtsam::Vector6() << ePos, eVel).finished();
      }
    }

    /** lifting all related state values in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) override {
      const auto poseI = values.at<gtsam::Pose3>(key1());
      const auto velI = values.at<gtsam::Vector3>(key2());
      const auto omegaI = values.at<gtsam::Vector3>(key3());
      const auto accI = values.at<gtsam::Vector6>(key4());
      const auto poseJ = values.at<gtsam::Pose3>(key5());
      const auto velJ = values.at<gtsam::Vector3>(key6());
      const auto omegaJ = values.at<gtsam::Vector3>(key7());
      const auto accJ = values.at<gtsam::Vector6>(key8());

      const auto liftedStates = (gtsam::Vector(36) << poseI.rotation().rpy(),
        poseI.translation(),
        velI, omegaI, accI,
        poseJ.rotation().rpy(),
        poseJ.translation(),
        velJ, omegaJ, accJ).finished();
      return liftedStates;
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) override {
      assert(state.size() != 36);
      gtsam::Values values;
      try {
        values.insert(key1(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(0, 0)),
                                           gtsam::Point3(state.block<3, 1>(3, 0))));
        values.insert(key2(), gtsam::Vector3(state.block<3, 1>(6, 0)));
        values.insert(key3(), gtsam::Vector3(state.block<3, 1>(9, 0)));

        values.insert(key4(), gtsam::Vector6(state.block<6, 1>(12, 0)));

        values.insert(key5(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(18, 0)),
                                           gtsam::Point3(state.block<3, 1>(21, 0))));
        values.insert(key6(), gtsam::Vector3(state.block<3, 1>(24, 0)));
        values.insert(key7(), gtsam::Vector3(state.block<3, 1>(27, 0)));
        values.insert(key8(), gtsam::Vector6(state.block<6, 1>(30, 0)));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << getName() << " cannot generate values from state vector " << state << " due to "
                  << ex.what() << std::endl;
      }
      return values;
    }

    /** return the measured */
    [[nodiscard]] std::pair<gtsam::Point3, gtsam::Vector3> measured() const {
      return {pos_, vel_};
    }

    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Point3() << this->pos_).finished(),
                                          (gtsam::Point3() << e->pos_).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "PVTFactor" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterpolatedPVTFactorFull",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(pos_);
      ar & BOOST_SERIALIZATION_NVP(vel_);
    }

  };
}

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::GPInterpolatedPVTFactor> : public Testable<fgo::factor::GPInterpolatedPVTFactor> {
  };

  template<>
  struct traits<fgo::factor::GPInterpolatedPVTFactorFull> : public Testable<fgo::factor::GPInterpolatedPVTFactorFull> {
  };
}
#endif //ONLINE_FGO_GPINTERPOLATEDPVTFACTOR_H
