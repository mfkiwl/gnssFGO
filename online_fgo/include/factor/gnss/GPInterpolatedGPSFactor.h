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


#ifndef ONLINE_FGO_GPINTERPOLATEDGPSFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDGPSFACTOR_H

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
#include "factor/FactorTypeIDs.h"

namespace fgo::factor {
  class GPInterpolatedGPSFactor : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
  protected:
    gtsam::Point3 pos_;
    gtsam::Vector3 lb_;
    double tau_{};

    typedef GPInterpolatedGPSFactor This;
    typedef gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> Base;
    typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
    GPBase GPBase_;
    bool useAutoDiff_ = false;


  public:
    GPInterpolatedGPSFactor() = default;

    GPInterpolatedGPSFactor(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                            const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                            const gtsam::Point3 &positionMeasured,
                            const gtsam::Vector3 &lb,
                            const gtsam::SharedNoiseModel &model,
                            const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                            bool useAutoDiff = false) : Base(model, poseKeyI, velKeyI, omegaKeyI, poseKeyJ, velKeyJ,
                                                             omegaKeyJ),
                                                        pos_(positionMeasured), lb_(lb), tau_(interpolator->getTau()),
                                                        GPBase_(interpolator), useAutoDiff_(useAutoDiff) {
      factorTypeID_ = FactorTypeID::GPGPS;
      factorName_ = "GPInterpolatedGPSFactor";
    }

    ~GPInterpolatedGPSFactor() override = default;

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
      if (useAutoDiff_) {
        if (H1)
          *H1 = gtsam::numericalDerivative61<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
        if (H2)
          *H2 = gtsam::numericalDerivative62<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
        if (H3)
          *H3 = gtsam::numericalDerivative63<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
        if (H4)
          *H4 = gtsam::numericalDerivative64<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
        if (H5)
          *H5 = gtsam::numericalDerivative65<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
        if (H6)
          *H6 = gtsam::numericalDerivative66<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4, boost::placeholders::_5,
                        boost::placeholders::_6),
            poseI, velI, omegaI, poseJ, velJ, omegaJ, 1e-5);
        return evaluateError_(poseI, velI, omegaI, poseJ, velJ, omegaJ);
      } else {
        gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hpose, Hrot, Hrot2;

        gtsam::Pose3 pose;

        if (H1 || H2 || H3 || H4 || H5 || H6) {
          pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ,
                                          Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P);
        } else {
          pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
        }

        const auto ePos = pose.translation(Hpose) + pose.rotation(Hrot).rotate(lb_, Hrot2) - pos_;

        if (H1) *H1 = (Hpose + Hrot2 * Hrot) * Hint1_P;
        if (H2) *H2 = (Hpose + Hrot2 * Hrot) * Hint2_P;
        if (H3) *H3 = (Hpose + Hrot2 * Hrot) * Hint3_P;
        if (H4) *H4 = (Hpose + Hrot2 * Hrot) * Hint4_P;
        if (H5) *H5 = (Hpose + Hrot2 * Hrot) * Hint5_P;
        if (H6) *H6 = (Hpose + Hrot2 * Hrot) * Hint6_P;

        return ePos;
      }
    }

    [[nodiscard]] gtsam::Vector
    evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ) const {
      const auto pose = GPBase_->interpolatePose(poseI, velI, omegaI, poseJ, velJ, omegaJ);
      return pose.translation() + pose.rotation().rotate(lb_) - pos_;
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
    [[nodiscard]] gtsam::Point3 measured() const {
      return pos_;
    }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Point3() << this->pos_).finished(),
                                          (gtsam::Point3() << e->pos_).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterpolatedGPSFactor" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterpolatedGPSFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(pos_);
    }

  };

  class GPInterpolatedGPSFactorFull
    : public NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> {
  protected:
    gtsam::Point3 pos_;
    gtsam::Vector3 lb_;
    double tau_{};

    typedef GPInterpolatedGPSFactorFull This;
    typedef NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> Base;
    typedef std::shared_ptr<fgo::models::GPInterpolator> GPBase;
    GPBase GPBase_;
    bool useAutoDiff_ = false;


  public:
    GPInterpolatedGPSFactorFull() = default;

    GPInterpolatedGPSFactorFull(const gtsam::Key &poseKeyI, const gtsam::Key &velKeyI, const gtsam::Key &omegaKeyI,
                                const gtsam::Key &accI,
                                const gtsam::Key &poseKeyJ, const gtsam::Key &velKeyJ, const gtsam::Key &omegaKeyJ,
                                const gtsam::Key &accJ,
                                const gtsam::Point3 &positionMeasured,
                                const gtsam::Vector3 &lb,
                                const gtsam::SharedNoiseModel &model,
                                const std::shared_ptr<fgo::models::GPInterpolator> &interpolator,
                                bool useAutoDiff = false) : Base(model, poseKeyI, velKeyI, omegaKeyI, accI,
                                                                 poseKeyJ, velKeyJ, omegaKeyJ, accJ),
                                                            pos_(positionMeasured), lb_(lb),
                                                            tau_(interpolator->getTau()), GPBase_(interpolator),
                                                            useAutoDiff_(useAutoDiff) {
      factorTypeID_ = FactorTypeID::GPGPS;
      factorName_ = "GPInterpolatedGPSFactorFull";
    }

    ~GPInterpolatedGPSFactorFull() override = default;

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
      if (useAutoDiff_) {
        if (H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
            std::bind(&This::evaluateError_, this, std::placeholders::_1, velI, omegaI, accI, poseJ, velJ,
                      omegaJ, accJ),
            poseI, 1e-5);
        if (H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, poseI, std::placeholders::_1, omegaI, accI, poseJ,
                      velJ, omegaJ, accJ),
            velI, 1e-5);
        if (H3)
          *H3 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, poseI, velI, std::placeholders::_1, accI, poseJ, velJ,
                      omegaJ, accJ),
            omegaI, 1e-5);

        if (H4)
          *H4 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector6>(
            std::bind(&This::evaluateError_, this, poseI, velI, omegaI, std::placeholders::_1, poseJ, velJ,
                      omegaJ, accJ),
            accI, 1e-5);

        if (H5)
          *H5 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
            std::bind(&This::evaluateError_, this, poseJ, velI, omegaI, accI, std::placeholders::_1, velJ,
                      omegaJ, accJ),
            poseJ, 1e-5);

        if (H6)
          *H6 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ,
                      std::placeholders::_1, omegaJ, accJ),
            velJ, 1e-5);

        if (H7)
          *H7 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ,
                      std::placeholders::_1, accJ),
            omegaJ, 1e-5);

        if (H8)
          *H8 = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Vector6>(
            std::bind(&This::evaluateError_, this, poseI, velI, omegaI, accI, poseJ, velJ, omegaJ,
                      std::placeholders::_1),
            accJ, 1e-5);

        return evaluateError_(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
      } else {
        gtsam::Matrix Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hint7_P, Hint8_P, Hint9_P, Hint10_P, Hpose, Hrot, Hrot2;

        gtsam::Pose3 pose;

        if (H1 || H2 || H3 || H4 || H5 || H6) {
          pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ,
                                          Hint1_P, Hint2_P, Hint3_P, Hint4_P, Hint5_P, Hint6_P, Hint7_P, Hint8_P);
        } else {
          pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
        }

        const auto ePos = pose.translation(Hpose) + pose.rotation(Hrot).rotate(lb_, Hrot2) - pos_;

        if (H1) *H1 = (Hpose + Hrot2 * Hrot) * Hint1_P;
        if (H2) *H2 = (Hpose + Hrot2 * Hrot) * Hint2_P;
        if (H3) *H3 = (Hpose + Hrot2 * Hrot) * Hint3_P;
        if (H4) *H4 = (Hpose + Hrot2 * Hrot) * Hint4_P;
        if (H5) *H5 = (Hpose + Hrot2 * Hrot) * Hint5_P;
        if (H6) *H6 = (Hpose + Hrot2 * Hrot) * Hint6_P;
        if (H7) *H7 = (Hpose + Hrot2 * Hrot) * Hint7_P;
        if (H8) *H8 = (Hpose + Hrot2 * Hrot) * Hint8_P;

        return ePos;
      }
    }

    [[nodiscard]] gtsam::Vector
    evaluateError_(const gtsam::Pose3 &poseI, const gtsam::Vector3 &velI, const gtsam::Vector3 &omegaI,
                   const gtsam::Vector6 &accI,
                   const gtsam::Pose3 &poseJ, const gtsam::Vector3 &velJ, const gtsam::Vector3 &omegaJ,
                   const gtsam::Vector6 &accJ) const {
      const auto pose = GPBase_->interpolatePose(poseI, velI, omegaI, accI, poseJ, velJ, omegaJ, accJ);
      return pose.translation() + pose.rotation().rotate(lb_) - pos_;
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
    [[nodiscard]] gtsam::Point3 measured() const {
      return pos_;
    }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Point3() << this->pos_).finished(),
                                          (gtsam::Point3() << e->pos_).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "GPInterpolatedGPSFactorFull" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GPInterpolatedGPSFactorFull",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(pos_);
    }

  };
}

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::GPInterpolatedGPSFactor> : public Testable<fgo::factor::GPInterpolatedGPSFactor> {
  };

  template<>
  struct traits<fgo::factor::GPInterpolatedGPSFactorFull> : public Testable<fgo::factor::GPInterpolatedGPSFactorFull> {
  };
}

#endif //ONLINE_FGO_GPINTERPOLATEDGPSFACTOR_H
