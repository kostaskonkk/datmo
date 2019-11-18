/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "filter_base.h"
#include "filter_common.h"

#include <vector>

namespace RobotLocalization
{
  FilterBase::FilterBase() :
    initialized_(false),
    lastMeasurementTime_(0.0),
    predictedState_(STATE_SIZE),
    state_(STATE_SIZE),
    covarianceEpsilon_(STATE_SIZE, STATE_SIZE),
    estimateErrorCovariance_(STATE_SIZE, STATE_SIZE),
    identity_(STATE_SIZE, STATE_SIZE),
    processNoiseCovariance_(STATE_SIZE, STATE_SIZE),
    transferFunction_(STATE_SIZE, STATE_SIZE),
    debugStream_(NULL),
    debug_(false)
  {
    reset();
  }

  FilterBase::~FilterBase()
  {
  }

  void FilterBase::reset()
  {
    initialized_ = false;

    // Clear the state and predicted state
    state_.setZero();
    predictedState_.setZero();

    // Prepare the invariant parts of the transfer
    // function
    transferFunction_.setIdentity();

    // Set the estimate error covariance. We want our measurements
    // to be accepted rapidly when the filter starts, so we should
    // initialize the state's covariance with large values.
    estimateErrorCovariance_.setIdentity();
    estimateErrorCovariance_ *= 1e-9;

    // We need the identity for the update equations
    identity_.setIdentity();

    // Set the epsilon matrix to be a matrix with small values on the diagonal
    // It is used to maintain the positive-definite property of the covariance
    covarianceEpsilon_.setIdentity();
    covarianceEpsilon_ *= 0.001;

    // Initialize our measurement time
    lastMeasurementTime_ = 0;

    // These can be overridden via the launch parameters,
    // but we need default values.
    processNoiseCovariance_.setZero();
    processNoiseCovariance_(StateMemberX, StateMemberX) = 0.05;
    processNoiseCovariance_(StateMemberY, StateMemberY) = 0.05;
    processNoiseCovariance_(StateMemberYaw, StateMemberYaw) = 0.06;
    processNoiseCovariance_(StateMemberVx, StateMemberVx) = 0.025;
    processNoiseCovariance_(StateMemberVy, StateMemberVy) = 0.025;
    processNoiseCovariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;

  }

  bool FilterBase::getDebug()
  {
    return debug_;
  }

  const Eigen::MatrixXd& FilterBase::getEstimateErrorCovariance()
  {
    return estimateErrorCovariance_;
  }

  bool FilterBase::getInitializedStatus()
  {
    return initialized_;
  }

  double FilterBase::getLastMeasurementTime()
  {
    return lastMeasurementTime_;
  }

  const Eigen::VectorXd& FilterBase::getPredictedState()
  {
    return predictedState_;
  }

  const Eigen::MatrixXd& FilterBase::getProcessNoiseCovariance()
  {
    return processNoiseCovariance_;
  }

  const Eigen::VectorXd& FilterBase::getState()
  {
    return state_;
  }

  void FilterBase::setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance)
  {
    estimateErrorCovariance_ = estimateErrorCovariance;
  }

  void FilterBase::setLastMeasurementTime(const double lastMeasurementTime)
  {
    lastMeasurementTime_ = lastMeasurementTime;
  }

  void FilterBase::setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance)
  {
    processNoiseCovariance_ = processNoiseCovariance;
  }

  void FilterBase::setState(const Eigen::VectorXd &state)
  {
    state_ = state;
  }

  void FilterBase::validateDelta(double &delta)
  {
    // This handles issues with ROS time when use_sim_time is on and we're playing from bags.
    if (delta > 100000.0)
    {
      FB_DEBUG("Delta was very large. Suspect playing from bag file. Setting to 0.01\n");

      delta = 0.01;
    }
  }

  void FilterBase::wrapStateAngles()
  {
    state_(StateMemberYaw)   = FilterUtilities::clampRotation(state_(StateMemberYaw));
  }

  bool FilterBase::checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                                             const Eigen::MatrixXd &invCovariance,
                                             const double nsigmas)
  {
    double sqMahalanobis = innovation.dot(invCovariance * innovation);
    double threshold = nsigmas * nsigmas;

    if (sqMahalanobis >= threshold)
    {
      FB_DEBUG("Innovation mahalanobis distance test failed. Squared Mahalanobis is: " << sqMahalanobis << "\n" <<
               "Threshold is: " << threshold << "\n" <<
               "Innovation is: " << innovation << "\n" <<
               "Innovation covariance is:\n" << invCovariance << "\n");

      return false;
    }

    return true;
  }
}  // namespace RobotLocalization
