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

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>
#include "utils/NavigationTools.h"
#include "utils/Pose3Utils.h"
#include "factor/FactorTypeID.h"

/*Inputs:
* Keys: pose of time i X(i), velocity of time i V(i), clock bias drift of time i C(i)
* Pseudorange measurement and doppler measurement: measRho and measdRho
* gyroscope measurement of time i: angularRate
* Position of the satellite and velocity of satellite: satXYZ, satVEL
* Position of sensor with respect to the Body: lb
* Covariance Matrix of the measurement/s: model*/
/* measurement equations used:
 * Pseudorange = Distance of Satellite and Receiver + Range through clock bias,
 * Doppler = Velocity between Satellite and Receiver (in direction) + Velocity through clock drift */
/*Jacobian: for X(i) = (e_RS * R_eb * skrew(lb_), e_RS * R_eb), V(i) = 0, e_RS, C(i) = 1,0,0,1
 * */
namespace fgo::factor {
  class PrDrFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias, gtsam::Vector2> {

  protected:
    gtsam::Vector3 satXYZ_;
    gtsam::Vector3 satVEL_;
    gtsam::Vector3 lb_;//lever arm between IMU and antenna in body frame
    gtsam::Vector3 omega_;
    double measRho_;
    double measdRho_;
    typedef PrDrFactor This;
    typedef gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias, gtsam::Vector2> Base;

    bool useAutoDiff_ = false;


  public:

    PrDrFactor() = default;  /* Default constructor */

    /**
     * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
     */
    PrDrFactor(gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key bias_i, gtsam::Key cbd_i,
               const double &measRho, const double &measdRho, const gtsam::Vector3 &satXYZ,
               const gtsam::Vector3 &satVEL, const gtsam::Vector3 &lb, const gtsam::Vector3 &omega,
               const gtsam::SharedNoiseModel &model, bool useAutoDiff = false) :
      Base(model, pose_i, vel_i, bias_i, cbd_i), satXYZ_(satXYZ), satVEL_(satVEL),
      lb_(lb), omega_(omega), measRho_(measRho), measdRho_(measdRho), useAutoDiff_(useAutoDiff) {
      factorTypeID_ = FactorTypeID::PRDR;
      factorName_ = "PrDrFactor";
    }

    ~PrDrFactor() override = default;

    /// @return a deep copy of this factor
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    gtsam::Vector
    evaluateError(const gtsam::Pose3 &pose, const gtsam::Vector3 &vel, const gtsam::imuBias::ConstantBias &bias1,
                  const gtsam::Vector2 &cbd,
                  boost::optional<gtsam::Matrix &> H1 = boost::none,
                  boost::optional<gtsam::Matrix &> H2 = boost::none,
                  boost::optional<gtsam::Matrix &> H3 = boost::none,
                  boost::optional<gtsam::Matrix &> H4 = boost::none) const override {

      if (useAutoDiff_) {
        if (H1) {
          *H1 = gtsam::numericalDerivative41<gtsam::Vector2, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias, gtsam::Vector2>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4), pose, vel, bias1, cbd, 1e-5);
        }
        if (H2)
          *H2 = gtsam::numericalDerivative42<gtsam::Vector2, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias, gtsam::Vector2>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4), pose, vel, bias1, cbd, 1e-5);
        if (H3)
          *H3 = gtsam::numericalDerivative43<gtsam::Vector2, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias, gtsam::Vector2>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4), pose, vel, bias1, cbd, 1e-5);
        if (H4)
          *H4 = gtsam::numericalDerivative44<gtsam::Vector2, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias, gtsam::Vector2>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, boost::placeholders::_2,
                        boost::placeholders::_3, boost::placeholders::_4), pose, vel, bias1, cbd, 1e-5);

        return evaluateError_(pose, vel, bias1, cbd);
      } else {

        //gtsam::Matrix36 Ha;
        gtsam::Matrix13 Hd;
        //gtsam::Point3 P_eb_e = pose.translation(H1 ? &Ha : nullptr);
        gtsam::Matrix Hpose_P, Hpose_Pr, Hpose_r, Hbias_gyro;
        gtsam::Matrix3 Hrho_p, Hdrho_p, Hdrho_t;

        gtsam::Point3 P_eA_e = pose.translation(&Hpose_P) + pose.rotation(&Hpose_Pr).rotate(lb_,
                                                                                            &Hrho_p); //pose.compose(body_P_sensor,H0_P).translation(Hpose_P);
        double real_range = gtsam::distance3(P_eA_e, satXYZ_, &Hd);

        const gtsam::Rot3 &eRb = pose.rotation(&Hpose_r);
        gtsam::Point3 lbv = gtsam::skewSymmetric((-lb_)) * bias1.correctGyroscope(omega_, &Hbias_gyro);
        gtsam::Point3 vel_eA_e = vel + eRb.rotate(lbv, &Hdrho_p, &Hdrho_t);

        double vel_delta = (Hd * (vel_eA_e - satVEL_))(0);

        gtsam::Matrix H1_rho;
        gtsam::Matrix H1_drho;

        if (H1) {

          gtsam::Matrix tmp_rho = Hrho_p * Hpose_Pr;
          H1_rho = Hd * (Hpose_P + tmp_rho);

          gtsam::Matrix tmp_drho = Hdrho_p * Hpose_r;
          H1_drho = Hd * (tmp_drho + Hdrho_t);
        }

        if (H1) *H1 = (gtsam::Matrix26() << H1_rho, H1_drho).finished();
        if (H2) *H2 = (gtsam::Matrix23() << 0, 0, 0, Hd).finished();
        if (H3)
          *H3 = (gtsam::Matrix26() << 0, 0, 0, 0, 0, 0, Hd * Hdrho_t * gtsam::skewSymmetric(lb_) *
                                                        Hbias_gyro).finished();
        if (H4) *H4 = (gtsam::Matrix22() << 1, 0, 0, 1).finished();

        return (gtsam::Vector2() << real_range + cbd(0) - measRho_,
          vel_delta + cbd(1) - measdRho_).finished();
      }
    }

    [[nodiscard]] gtsam::Vector
    evaluateError_(const gtsam::Pose3 &pose, const gtsam::Vector3 &vel, const gtsam::imuBias::ConstantBias &bias1,
                   const gtsam::Vector2 &cbd) const {

      gtsam::Point3 lbv = gtsam::skewSymmetric((-lb_)) * bias1.correctGyroscope(omega_);
      const gtsam::Point3 &P_eb_e = pose.translation();
      const gtsam::Rot3 &eRb = pose.rotation();
      gtsam::Point3 vel_eA_e = vel + eRb.rotate(lbv);
      gtsam::Point3 P_eA_e = P_eb_e + eRb.rotate(lb_);
      gtsam::Matrix13 Hd;
      double real_range = gtsam::distance3(P_eA_e, satXYZ_, Hd);
      double vel_delta = (Hd * (vel_eA_e - satVEL_))(0);

      return (gtsam::Vector2() << real_range + cbd(0) - measRho_,
        vel_delta + cbd(1) - measdRho_).finished();

    }

    /** lifting all related state values in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) override {
      const auto pose = values.at<gtsam::Pose3>(key1());
      const auto vel = values.at<gtsam::Vector3>(key2());
      const auto bias = values.at<gtsam::imuBias::ConstantBias>(key3());
      const auto cbd = values.at<gtsam::Vector2>(key4());
      const auto liftedStates = (gtsam::Vector(17) << pose.rotation().rpy(),
        pose.translation(),
        vel, bias.vector(), cbd).finished();
      return liftedStates;
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) override {
      assert(state.size() != 17);
      gtsam::Values values;
      try {
        values.insert(key1(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(0, 0)),
                                           gtsam::Point3(state.block<3, 1>(3, 0))));
        values.insert(key2(), gtsam::Vector3(state.block<3, 1>(6, 0)));
        values.insert(key3(), gtsam::imuBias::ConstantBias(state.block<6, 1>(9, 0)));
        values.insert(key4(), gtsam::Vector2(state.block<2, 1>(15, 0)));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << getName() << " cannot generate values from state vector " << state << " due to "
                  << ex.what() << std::endl;
      }
      return values;
    }

    /** return the measured */
    [[nodiscard]] gtsam::Vector2 measured() const {
      return {measRho_, measdRho_};
    }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != NULL && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol((gtsam::Vector2() << this->measRho_, this->measdRho_).finished(),
                                          (gtsam::Vector2() << e->measRho_, e->measdRho_).finished(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "Gnssfactor specifically designed for septentrio" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("PrDrFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measRho_);
    }
  }; // PrDrFactor
} // namespace fgo_online

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::PrDrFactor> : public Testable<fgo::factor::PrDrFactor> {
  };
}

