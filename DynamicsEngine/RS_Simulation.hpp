/*!
 * @file Simulation.h
 * @brief Main simulation class
 */

#ifndef PROJECT_SIMULATION_H
#define PROJECT_SIMULATION_H

#include "ControlParameters/SimulatorParameters.h"

#include <SimUtilities/ImuSimulator.h>
#include <SimUtilities/realSenseSimulator.h>
#include <utilities/Timer.h>

#include <utility>
#include <vector>

#include "DynamicsEngine.h"

#include <lcm/lcm-cpp.hpp>
#include <common/dynamics/ActuatorModel.h>
#include <simulator_lcmt.hpp>
#include <sim_box_info_lcmt.hpp>

#define SIM_LCM_NAME "simulator_state"

/*!
 * Top-level control of a simulation.
 * A simulation includes 1 robot and 1 controller
 * It does not include the graphics window: this must be set with the setWindow
 * method
 */
class RS_Simulation {
  //friend class SimControlPanel;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      explicit RS_Simulation(
          FloatingBaseModel<double> * model,
          SimulatorParameters& params,
          const std::string & terrain_file);

    /*!
     * Explicitly set the state of the robot
     */
    void setRobotState(FBModelState<double>& state) {
      _dyn_engine->setState(state);
    }

    const FBModelState<double>& getRobotState() {
      return _dyn_engine->getState();
    }

    void step(
        const std::vector<ActuatorModel<double>*> & actuator_models,
        FBModelState<double> & state, FBModelStateDerivative<double> & dstate);

    void addCollisionPlane(double mu, double resti, double height);
    size_t addCollisionBox(double mu, double resti, double depth, double width,
        double height, const Vec3<double>& pos,
        const Mat3<double>& ori, bool addToWindow = true,
        bool transparent = true);
    void addCollisionMesh(double mu, double resti, double grid_size,
        const Vec3<double>& left_corner_loc,
        const DMat<double>& _height_map, bool addToWindow = true,
        bool transparent = true);

    void removeCollisionBox(size_t collision_idx);

    void highLevelControl();

    void applyExternalForceToBase(SVec<double> force);

    void resetSimTime() {
      _currentSimTime = 0.;
      _timeOfNextLowLevelControl = 0.;
      _timeOfNextHighLevelControl = 0.;
    }

    ~RS_Simulation() {
      delete _dyn_engine;
      delete _lcm;
    }
    void buildLcmMessage();
    void loadTerrainFile(const std::string& terrainFileName, bool addGraphics = true);

    DynamicsEngine<double>* getDynamicsEngine(){return _dyn_engine;}


  private:
    size_t _numActJoint;
    FloatingBaseModel<double>* _model;
    DVec<double> _tau;
    DynamicsEngine<double>* _dyn_engine = nullptr;
    SimulatorParameters& _simParams;
    lcm::LCM* _lcm = nullptr;

    double _currentSimTime = 0.;
    double _timeOfNextLowLevelControl = 0.;
    double _timeOfNextHighLevelControl = 0.;
    s64 _highLevelIterations = 0;

    // LCM types
    simulator_lcmt _simLCM;
    sim_box_info_lcmt _simObjectLCM;

    // terrain file and heightmap data
    DMat<double> terrainHeightmap;
    Vec3<double> terrainMeshLeftCorner; // xyz location of the (bottom) left corner of the grid
    double terrainResolution_; // units are [m/cell]
    int terrainGridSize_[2]; // number of cells: (rows, cols), aligned with world frame, (x, y)
    bool terrainFile = false; // whether a terrain file has been loaded
};

#endif // PROJECT_SIMULATION_H
