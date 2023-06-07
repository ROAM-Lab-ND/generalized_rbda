/*! @file realSenseSimulator.h
 *  @brief Simulated RealSense with noise
 */

#ifndef PROJECT_REALSENSESIMULATOR_H
#define PROJECT_REALSENSESIMULATOR_H

#include <random>
#include "ControlParameters/SimulatorParameters.h"
#include <common/dynamics/FloatingBaseModel.h>
#include <common/sensors/LocalizationTypes.h>
#include "cppTypes.h"

/*!
 * Simulation of Real Sense
 */
template <typename T>
class realSenseSimulator {
 public:
  explicit realSenseSimulator(SimulatorParameters& simSettings, u64 seed = 0)
      : _simSettings(simSettings),
        _mt(seed),
        _localizationPositionDistribution(-simSettings.rs_localization_position_noise,
                                   simSettings.rs_localization_position_noise),
        _localizationOrientationDistribution(-simSettings.rs_localization_position_noise,
                                   simSettings.rs_localization_position_noise)
  {
    if (simSettings.rs_localization_position_noise + simSettings.rs_localization_orientation_noise != 0) {
      _localizationNoise = true;
    }
  }

  void updateLocalization(const FBModelState<T>& robotState,
                       const FBModelStateDerivative<T>& robotStateD,
                       LocalizationData* data);

 private:
  SimulatorParameters& _simSettings;
  std::mt19937 _mt;
  std::uniform_real_distribution<float> _localizationPositionDistribution;
  std::uniform_real_distribution<float> _localizationOrientationDistribution;
  bool _localizationNoise = false;

};
#endif  // PROJECT_REALSENSESIMULATOR_H
