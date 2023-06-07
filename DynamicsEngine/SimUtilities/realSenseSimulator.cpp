/*! @file realSenseSimulator.cpp
 *  @brief Simulated Real Sense
 */
#include "realSenseSimulator.h"
#include <math/spatial.h>
#include <math/orientation_tools.h>
#include <utilities/utilities.h>


/*!
 * Compute position (other data will be added later) from RealSense localization sensor
 * At this point, we assume we only use sensor 1 (located on the left side of the robot)
 * State estimator only uses position and rotation matrix, so at this point we do not
 * bother to simulate the other outputs that the localization sensor offers (vBody, omegaBody,etc)
 */
template <typename T>
void realSenseSimulator<T>::updateLocalization(
    const FBModelState<T> &robotState,
    const FBModelStateDerivative<T> &robotStateD, LocalizationData *data) {

  for(int sidx(0); sidx<NUM_TRACKING_SENSOR; ++sidx){
    // Global position of the robot in the world frame
    for(int i = 0; i < 3; i++)
      data->positionRobot[sidx][i] = robotState.bodyPosition[i];

    // Orientation of the robot (describes rotation from world frame to body frame)
    for(int i = 0; i < 4; i++)
      data->orientation[sidx][i] = robotState.bodyOrientation[i];

    if(_localizationNoise){
      Vec3<float> noise;
      fillEigenWithRandom(noise, _mt, _localizationPositionDistribution);
      data->positionRobot[sidx][0] += noise[0];
      data->positionRobot[sidx][1] += noise[1];
      data->positionRobot[sidx][2] += noise[2];

      Vec4<float> noiseQuat;
      fillEigenWithRandom(noiseQuat, _mt, _localizationOrientationDistribution);
      data->orientation[sidx][0] += noiseQuat[0];
      data->orientation[sidx][1] += noiseQuat[1];
      data->orientation[sidx][2] += noiseQuat[2];
      data->orientation[sidx][3] += noiseQuat[3];
    }

    data->rpy_world_to_body[sidx] = ori::quatToRPY(data->orientation[sidx]);

    data->tracker_conf[sidx] = 3.0;
  }
}

template class realSenseSimulator<double>;
template class realSenseSimulator<float>;
