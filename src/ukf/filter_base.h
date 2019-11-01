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

#ifndef ROBOT_LOCALIZATION_FILTER_BASE_H
#define ROBOT_LOCALIZATION_FILTER_BASE_H

#include "filter_utilities.h"
#include "filter_common.h"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <map>
#include <ostream>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace RobotLocalization
{

//! @brief Structure used for storing and comparing measurements
//! (for priority queues)
//!
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in seconds.
//!
struct Measurement
{
  // The time stamp of the most recent control term (needed for lagged data)
  double latestControlTime_;

  // The Mahalanobis distance threshold in number of sigmas
  double mahalanobisThresh_;

  // The real-valued time, in seconds, since some epoch
  // (presumably the start of execution, but any will do)
  double time_;

  // The topic name for this measurement. Needed
  // for capturing previous state values for new
  // measurements.
  std::string topicName_;

  // This defines which variables within this measurement
  // actually get passed into the filter. std::vector<bool>
  // is generally frowned upon, so we use ints.
  std::vector<int> updateVector_;

  // The most recent control vector (needed for lagged data)
  Eigen::VectorXd latestControl_;

  // The measurement and its associated covariance
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd covariance_;

  // We want earlier times to have greater priority
  bool operator()(const boost::shared_ptr<Measurement> &a, const boost::shared_ptr<Measurement> &b)
  {
    return (*this)(*(a.get()), *(b.get()));
  }

  bool operator()(const Measurement &a, const Measurement &b)
  {
    return a.time_ > b.time_;
  }

  Measurement() :
    latestControlTime_(0.0),
    mahalanobisThresh_(std::numeric_limits<double>::max()),
    time_(0.0),
    topicName_("")
  {
  }
};
typedef boost::shared_ptr<Measurement> MeasurementPtr;

class FilterBase
{
  public:
    //! @brief Constructor for the FilterBase class
    //!
    FilterBase();

    //! @brief Destructor for the FilterBase class
    //!
    virtual ~FilterBase();

    //! @brief Resets filter to its unintialized state
    //!
    void reset();

    //! @brief Carries out the correct step in the predict/update cycle. This method
    //! must be implemented by subclasses.
    //!
    //! @param[in] measurement - The measurement to fuse with the state estimate
    //!
    //virtual void correct(const Measurement &measurement) = 0;

    //! @brief Gets the value of the debug_ variable.
    //!
    //! @return True if in debug mode, false otherwise
    //!
    bool getDebug();

    //! @brief Gets the estimate error covariance
    //!
    //! @return A copy of the estimate error covariance matrix
    //!
    const Eigen::MatrixXd& getEstimateErrorCovariance();

    //! @brief Gets the filter's initialized status
    //!
    //! @return True if we've received our first measurement, false otherwise
    //!
    bool getInitializedStatus();

    //! @brief Gets the most recent measurement time
    //!
    //! @return The time at which we last received a measurement
    //!
    double getLastMeasurementTime();

    //! @brief Gets the filter's predicted state, i.e., the
    //! state estimate before correct() is called.
    //!
    //! @return A constant reference to the predicted state
    //!
    const Eigen::VectorXd& getPredictedState();

    //! @brief Gets the filter's process noise covariance
    //!
    //! @return A constant reference to the process noise covariance
    //!
    const Eigen::MatrixXd& getProcessNoiseCovariance();

    //! @brief Gets the sensor timeout value (in seconds)
    //!
    //! @return The sensor timeout value
    //!
    double getSensorTimeout();

    //! @brief Gets the filter state
    //!
    //! @return A constant reference to the current state
    //!
    const Eigen::VectorXd& getState();

    //! @brief Carries out the predict step in the predict/update cycle.
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion. This method must be implemented by subclasses.
    //!
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //!
    //virtual void predict(const double referenceTime, const double delta) = 0;

    //! @brief Does some final preprocessing, carries out the predict/update cycle
    //!
    //! @param[in] measurement - The measurement object to fuse into the filter
    //!
    //virtual void processMeasurement(const Measurement &measurement);

    //! @brief Sets the filter into debug mode
    //!
    //! NOTE: this will generates a lot of debug output to the provided stream.
    //! The value must be a pointer to a valid ostream object.
    //!
    //! @param[in] debug - Whether or not to place the filter in debug mode
    //! @param[in] outStream - If debug is true, then this must have a valid pointer.
    //! If the pointer is invalid, the filter will not enter debug mode. If debug is
    //! false, outStream is ignored.
    //!
    void setDebug(const bool debug, std::ostream *outStream = NULL);

    //! @brief Manually sets the filter's estimate error covariance
    //!
    //! @param[in] estimateErrorCovariance - The state to set as the filter's current state
    //!
    void setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance);

    //! @brief Sets the filter's last measurement time.
    //!
    //! @param[in] lastMeasurementTime - The last measurement time of the filter
    //!
    void setLastMeasurementTime(const double lastMeasurementTime);

    //! @brief Sets the process noise covariance for the filter.
    //!
    //! This enables external initialization, which is important, as this
    //! matrix can be difficult to tune for a given implementation.
    //!
    //! @param[in] processNoiseCovariance - The STATE_SIZExSTATE_SIZE process noise covariance matrix
    //! to use for the filter
    //!
    void setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance);

    //! @brief Manually sets the filter's state
    //!
    //! @param[in] state - The state to set as the filter's current state
    //!
    void setState(const Eigen::VectorXd &state);

    //! @brief Ensures a given time delta is valid (helps with bag file playback issues)
    //!
    //! @param[in] delta - The time delta, in seconds, to validate
    //!
    void validateDelta(double &delta);

  protected:

    //! @brief Keeps the state Euler angles in the range [-pi, pi]
    //!
    virtual void wrapStateAngles();

    //! @brief Tests if innovation is within N-sigmas of covariance. Returns true if passed the test.
    //! @param[in] innovation - The difference between the measurement and the state
    //! @param[in] invCovariance - The innovation error
    //! @param[in] nsigmas - Number of standard deviations that are considered acceptable
    //!
    virtual bool checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                                           const Eigen::MatrixXd &invCovariance,
                                           const double nsigmas);

    //! @brief Whether or not we've received any measurements
    //!
    bool initialized_;

    //! This value is used to monitor sensor readings with respect to the sensorTimeout_.
    //! We also use it to compute the time delta values for our prediction step.
    //!
    double lastMeasurementTime_;

    //! @brief Holds the last predicted state of the filter
    //!
    Eigen::VectorXd predictedState_;

    //! @brief This is the robot's state vector, which is what we are trying to
    //! filter. The values in this vector are what get reported by the node.
    //!
    Eigen::VectorXd state_;

    //! @brief Covariance matrices can be incredibly unstable. We can
    //! add a small value to it at each iteration to help maintain its
    //! positive-definite property.
    //!
    Eigen::MatrixXd covarianceEpsilon_;

    //! @brief This matrix stores the total error in our position
    //! estimate (the state_ variable).
    //!
    Eigen::MatrixXd estimateErrorCovariance_;

    //! @brief We need the identity for a few operations. Better to store it.
    //!
    Eigen::MatrixXd identity_;

    //! @brief As we move through the world, we follow a predict/update
    //! cycle. If one were to imagine a scenario where all we did was make
    //! predictions without correcting, the error in our position estimate
    //! would grow without bound. This error is stored in the
    //! stateEstimateCovariance_ matrix. However, this matrix doesn't answer
    //! the question of *how much* our error should grow for each time step.
    //! That's where the processNoiseCovariance matrix comes in. When we
    //! make a prediction using the transfer function, we add this matrix
    //! (times deltaT) to the state estimate covariance matrix.
    //!
    Eigen::MatrixXd processNoiseCovariance_;

    //! @brief The Kalman filter transfer function
    //!
    //! Kalman filters and extended Kalman filters project the current
    //! state forward in time. This is the "predict" part of the predict/correct
    //! cycle. A Kalman filter has a (typically constant) matrix A that defines
    //! how to turn the current state, x, into the predicted next state. For an
    //! EKF, this matrix becomes a function f(x). However, this function can still
    //! be expressed as a matrix to make the math a little cleaner, which is what
    //! we do here. Technically, each row in the matrix is actually a function.
    //! Some rows will contain many trigonometric functions, which are of course
    //! non-linear. In any case, you can think of this as the 'A' matrix in the
    //! Kalman filter formulation.
    //!
    Eigen::MatrixXd transferFunction_;

    //! @brief Used for outputting debug messages
    //!
    std::ostream *debugStream_;

  private:
    //! @brief Whether or not the filter is in debug mode
    //!
    bool debug_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_FILTER_BASE_H
