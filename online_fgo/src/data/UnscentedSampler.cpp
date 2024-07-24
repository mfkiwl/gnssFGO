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

#include "data/sampling/UncentedSampler.h"

namespace fgo::data::sampler {
  MerweScaledSigmaPoints::MerweScaledSigmaPoints(size_t n, double alpha, double beta, double kappa)
    : alpha_(alpha), beta_(beta) {
    kappa_ = kappa;
    n_state_ = n;
    n_sigma_points_ = 2 * n_state_ + 1;
    lambda_ = alpha_ * alpha_ * (n_state_ + kappa_) - n_state_;
    const auto c = 0.5 / (n_state_ + lambda_);

    Wm_ = gtsam::Vector::Ones(n_sigma_points_) * c;
    Wc_ = Wm_;
    Wm_[0] = lambda_ / (n_state_ + lambda_);
    Wc_[0] = lambda_ / (n_state_ + lambda_) + (1 - alpha_ * alpha_ + beta_);
  }

  gtsam::Matrix MerweScaledSigmaPoints::sigma_points(const gtsam::Vector &mean, const gtsam::Matrix &cov,
                                                     bool isTrangularized) {
    gtsam::Matrix A;
    if (!isTrangularized) {
      A = cov.llt().matrixL();
    } else
      A = cov;
    // std::cout << A << std::endl;
    const auto s = sqrt(lambda_ + n_state_);
    gtsam::Matrix sigmas = gtsam::Matrix::Zero(n_sigma_points_, n_state_);
    sigmas.row(0) = mean;
    for (size_t i = 0; i < n_state_; i++) {
      sigmas.block(i + 1, 0, 1, n_state_) = (s * A.row(i)) + mean.transpose();
      sigmas.block(n_state_ + i + 1, 0, 1, n_state_) = (-1 * s * A.row(i)) + mean.transpose();
    }

    /*
    std::cout << "rows: " << sigmas.rows() << " cols: " << sigmas.cols() << std::endl;

    for(size_t i = 0; i < sigmas.rows(); i++)
    {
        std::cout << sigmas.row(i) << std::endl;
    }

    std::cout << std::endl;
    */
    return sigmas;
  }

  JulierSigmaPoints::JulierSigmaPoints(size_t n, double kappa) {
    kappa_ = kappa;
    n_state_ = n;
    n_sigma_points_ = 2 * n_state_ + 1;
    Wm_ = gtsam::Vector::Ones(n_sigma_points_) * 0.5 / (n_state_ + kappa_);
    Wm_[0] = kappa_ / (n_state_ + kappa_);
    Wc_ = Wm_;
  }

  gtsam::Matrix JulierSigmaPoints::sigma_points(const gtsam::Vector &mean, const gtsam::Matrix &cov,
                                                bool isTrangularized) {
    gtsam::Matrix A;
    if (!isTrangularized) {
      A = cov.llt().matrixL();
    } else
      A = cov;
    const auto s = sqrt(kappa_ + n_state_);
    gtsam::Matrix sigmas = gtsam::Matrix::Zero(n_sigma_points_, n_state_);
    sigmas.row(0) = mean;

    for (size_t i = 0; i < n_state_; i++) {
      sigmas.block(i + 1, 0, 1, n_state_) = (s * A.row(i)) + mean;
      sigmas.block(n_state_ + i + 1, 0, 1, n_state_) = (-1 * s * A.row(i)) + mean;
    }
    std::cout << "rows: " << sigmas.rows() << " cols: " << sigmas.cols() << std::endl;
    return sigmas;
  }

  SimplexSigmaPoints::SimplexSigmaPoints(size_t n, double alpha) : alpha_(alpha) {
    n_state_ = n;
    n_sigma_points_ = n_state_ + 1;
    lambda_ = n_state_ / n_sigma_points_;
    const auto c = 1. / n_sigma_points_;
    Wm_ = gtsam::Vector::Ones(n_sigma_points_) * c;
    Wc_ = Wm_;
  }

  gtsam::Matrix SimplexSigmaPoints::sigma_points(const gtsam::Vector &mean, const gtsam::Matrix &cov,
                                                 bool isTrangularized) {
    gtsam::Matrix A;
    if (!isTrangularized) {
      A = cov.llt().matrixL();
    } else
      A = cov;
    const auto Istar = nc::NdArray<double>();

    for (size_t i = 1; i < n_state_ + 1; i++) {
      const auto vf = 1. / sqrt(lambda_ * i * (i + 1));
      const auto vb = -n_state_ * vf;
      auto row = nc::ones<double>(1, n_state_) * vf;
      row = nc::append(row, nc::asarray({vb}));
      for (size_t k = 0; k < n_state_ + 1 - row.shape().cols; k++) {
        row = nc::append(row, nc::zeros<double>(1));
      }
      nc::append(Istar, row, nc::Axis::ROW);
    }

    const auto I = nc::sqrt(n_state_) * Istar;
    const auto I_ = gtsam::Matrix::Map(I.data(), I.numRows(), I.numCols());
    const auto scaled_unitary = A.transpose() * I_;
    return scaled_unitary.colwise() + mean;
  }


  UnscentedSampler::UnscentedSampler(const SamplerConfig::SharedPtr &param,
                                     const gtsam::Vector &mean,
                                     const gtsam::noiseModel::Base::shared_ptr &model)
    : gtsam::Sampler(mean, model), param_(param) {
    if (param_->sigmaType == "Merwe")
      sigma_generator_ = std::make_shared<MerweScaledSigmaPoints>(mean.size(),
                                                                  param_->alpha,
                                                                  param_->beta,
                                                                  param_->kappa);
    else if (param_->sigmaType == "Julier")
      sigma_generator_ = std::make_shared<JulierSigmaPoints>(mean.size(), param_->kappa);

    else if (param_->sigmaType == "Simplex")
      sigma_generator_ = std::make_shared<SimplexSigmaPoints>(mean.size(), param_->alpha);

    else
      throw std::runtime_error("UnscentedSampler::sigma type " + param_->sigmaType + " not supported!");
  }

  UnscentedSampler::UnscentedSampler(const SamplerConfig::SharedPtr &param, const gtsam::Vector &sigmas)
    : gtsam::Sampler(sigmas), param_(param) {
    if (param_->sigmaType == "Merwe")
      sigma_generator_ = std::make_shared<MerweScaledSigmaPoints>(sigmas.size(),
                                                                  param_->alpha,
                                                                  param_->beta,
                                                                  param_->kappa);
    else if (param_->sigmaType == "Julier")
      sigma_generator_ = std::make_shared<JulierSigmaPoints>(sigmas.size(), param_->kappa);

    else if (param_->sigmaType == "Simplex")
      sigma_generator_ = std::make_shared<SimplexSigmaPoints>(sigmas.size(), param_->alpha);

    else
      throw std::runtime_error("UnscentedSampler::sigma type " + param_->sigmaType + " not supported!");
  }

  UnscentedSampler::UnscentedSampler(const SamplerConfig::SharedPtr &param, const gtsam::Vector &mean,
                                     const gtsam::Matrix &cov)
    : gtsam::Sampler(mean, gtsam::noiseModel::Gaussian::Covariance(cov, false)), param_(param), cov_(cov) {
    if (param_->sigmaType == "Merwe")
      sigma_generator_ = std::make_shared<MerweScaledSigmaPoints>(mean.size(),
                                                                  param_->alpha,
                                                                  param_->beta,
                                                                  param_->kappa);
    else if (param_->sigmaType == "Julier")
      sigma_generator_ = std::make_shared<JulierSigmaPoints>(mean.size(), param_->kappa);

    else if (param_->sigmaType == "Simplex")
      sigma_generator_ = std::make_shared<SimplexSigmaPoints>(mean.size(), param_->alpha);

    else
      throw std::runtime_error("UnscentedSampler::sigma type " + param_->sigmaType + " not supported!");
  }

  /*
  void UnscentedSampler::unscented_transform(const std::function<gtsam::Vector(const gtsam::Values &,
                                                                              boost::optional<std::vector<gtsam::Matrix> &>)> &func)
  {

  }

  void UnscentedSampler::unscented_transform(const std::function<gtsam::Vector(const gtsam::Values &)> &func) {

  }
  */


  gtsam::Matrix UnscentedSampler::samples() const {
    const auto sigma_points = sigma_generator_->sigma_points(this->mean(), cov_, false);
    return sigma_points;
  }


}