#include "RS_Simulation.hpp"
#include <ParamHandler/ParamHandler.hpp>

#include <Configuration.h>
#include <unistd.h>
#include <fstream>
#include <utilities/utilities.h>

/*
 * TODO list
 * sensor update
 * low-level control dt, high level control dt
 * sensor and cheater mode output
 * simulation_visualization_info: heightmap, collision box, ...
 * speed change, pause, rewind, ... GUI
 */

// if DISABLE_HIGH_LEVEL_CONTROL is defined, the simulator will run freely,
// without trying to connect to a robot
//#define DISABLE_HIGH_LEVEL_CONTROL

/*!
 * Initialize the simulator here.  It is _not_ okay to block here waiting for
 * the robot to connect. Use firstRun() instead!
 */
RS_Simulation::RS_Simulation(FloatingBaseModel<double>* model,
      SimulatorParameters& params, const std::string & terrain_file):
  _model(model),
  _simParams(params)
{

  // init parameters
  printf("[RS_Simulation] Load parameters...\n");
  if (!_simParams.isFullyInitialized()) {
    printf("[ERROR] Simulator parameters are not fully initialized.  You forgot: "
        "\n%s\n",
        _simParams.generateUnitializedList().c_str());
    throw std::runtime_error("simulator not initialized");
  }

  // init LCM
  if (_simParams.sim_state_lcm) {
    printf("[RS_Simulation] Setup LCM...\n");
    _lcm = new lcm::LCM(getLcmUrl(_simParams.sim_lcm_ttl));
    if (!_lcm->good()) {
      printf("[ERROR] Failed to set up LCM\n");
      throw std::runtime_error("lcm bad");
    }
  }

  printf("[RS_Simulation] Build RS_Simulation Environment...\n");

  // init rigid body dynamics
  printf("[RS_Simulation] Build rigid body model...\n");
  _dyn_engine = new DynamicsEngine<double>(_model, (bool)_simParams.use_spring_damper);
  _numActJoint = _model->_nDof - 6;
  _tau = DVec<double>::Zero(_numActJoint);


  loadTerrainFile(THIS_COM + terrain_file);
  printf("[RS_Simulation] Ready!\n");
}

/*!
 * Take a single timestep of dt seconds
 */
void RS_Simulation::step(
    const std::vector<ActuatorModel<double>*> & actuator_models,
    FBModelState<double> & state, FBModelStateDerivative<double> & dstate) {

  double dt = _simParams.dynamics_dt;
  double dtLowLevelControl = _simParams.low_level_dt;
  double dtHighLevelControl = _simParams.high_level_dt;

  double actuator_cmd (0.);
  // Low level control (if needed)
  if (_currentSimTime >= _timeOfNextLowLevelControl) {
    for (size_t idx(0); idx<_numActJoint; ++idx){
      actuator_cmd = actuator_models[idx]->computeLowLevelCMD(_dyn_engine->getState().q[idx], _dyn_engine->getState().qd[idx]);
      _tau[idx] = actuator_models[idx]->getTorque(actuator_cmd, _dyn_engine->getState().qd[idx]);
    }
    _timeOfNextLowLevelControl = _timeOfNextLowLevelControl + dtLowLevelControl;
  }
  //pretty_print(_tau, std::cout, "tau");

  // High level control
  if (_currentSimTime >= _timeOfNextHighLevelControl) {
    highLevelControl();
    _timeOfNextHighLevelControl = _timeOfNextHighLevelControl + dtHighLevelControl;
  }


  // dynamics
  _currentSimTime += dt;

  // Set Homing Information
  RobotHomingInfo<double> homing;
  homing.active_flag = _simParams.go_home;
  homing.position = _simParams.home_pos;
  homing.rpy = _simParams.home_rpy;
  homing.kp_lin = _simParams.home_kp_lin;
  homing.kd_lin = _simParams.home_kd_lin;
  homing.kp_ang = _simParams.home_kp_ang;
  homing.kd_ang = _simParams.home_kd_ang;
  _dyn_engine->setHoming(homing);
  _dyn_engine->step(dt, _tau, _simParams.floor_kp, _simParams.floor_kd);

  state = _dyn_engine->getState();
  dstate = _dyn_engine->getDState();

  // wait for robot code to finish (and send LCM while waiting)
  if (_lcm) {
    buildLcmMessage();
    _lcm->publish("simulator_state", &_simLCM);
  }
}

void RS_Simulation::highLevelControl() {
  // send IMU/localization data to robot:
  //_imuSimulator->updateCheaterState(_dyn_engine->getState(),
      //_dyn_engine->getDState(),
      //_sharedMemory().simToRobot.cheaterState);

  //_imuSimulator->updateVectornav(_dyn_engine->getState(),
      //_dyn_engine->getDState(),
      //&_sharedMemory().simToRobot.vectorNav);

  //_rsSimulator->updateLocalization(_dyn_engine->getState(),
      //_dyn_engine->getDState(),
      //&_sharedMemory().simToRobot.localization);

  _highLevelIterations++;
}



void RS_Simulation::buildLcmMessage() {
  auto& state = _dyn_engine->getState();
  auto& dstate = _dyn_engine->getDState();

  Vec3<double> rpy = ori::quatToRPY(state.bodyOrientation);
  RotMat<double> Rbody = ori::quaternionToRotationMatrix(state.bodyOrientation);
  Vec3<double> omega = Rbody.transpose() * state.bodyVelocity.head<3>();
  Vec3<double> v = Rbody.transpose() * state.bodyVelocity.tail<3>();

  _simLCM.time = _currentSimTime;
  _simLCM.timesteps = _highLevelIterations;

  for (size_t i = 0; i < 4; i++) {
    _simLCM.quat[i] = state.bodyOrientation[i];
  }

  for (size_t i = 0; i < 3; i++) {
    _simLCM.vb[i] = state.bodyVelocity[i + 3];  // linear velocity in body frame
    _simLCM.rpy[i] = rpy[i];
    for (size_t j = 0; j < 3; j++) {
      _simLCM.R[i][j] = Rbody(i, j);
    }
    _simLCM.omegab[i] = state.bodyVelocity[i];
    _simLCM.omega[i] = omega[i];
    _simLCM.p[i] = state.bodyPosition[i];
    _simLCM.v[i] = v[i];
    _simLCM.vbd[i] = dstate.dBodyVelocity[i + 3];
  }
  for (int leg = 0; leg < 2; ++leg){
    for (int jt = 0; jt < 5; ++jt){
      _simLCM.qdHleg[leg][jt] = state.qd[5*leg+jt];
    }
  }
}

/*!
 * Add an infinite collision plane to the simulator
 * @param mu          : friction of the plane
 * @param resti       : restitution coefficient
 * @param height      : height of plane
 * @param sizeX
 * @param sizeY
 * @param checkerX
 * @param checkerY
 * @param addToWindow : if true, also adds graphics for the plane
 */
void RS_Simulation::addCollisionPlane(double mu, double resti, double height) {
  _dyn_engine->addCollisionPlane(mu, resti, height);
}

/*!
 * Add an box collision to the simulator
 * @param mu          : location of the box
 * @param resti       : restitution coefficient
 * @param depth       : depth (x) of box
 * @param width       : width (y) of box
 * @param height      : height (z) of box
 * @param pos         : position of box
 * @param ori         : orientation of box
 * @param addToWindow : if true, also adds graphics for the plane
 */
size_t RS_Simulation::addCollisionBox(double mu, double resti, double depth,
    double width, double height,
    const Vec3<double>& pos,
    const Mat3<double>& ori, bool addToWindow,
    bool transparent) {
  size_t collision_idx = _dyn_engine->addCollisionBox(mu, resti, depth, width, height, pos, ori);
  return collision_idx;
}

void RS_Simulation::removeCollisionBox(size_t collision_idx) {
  _dyn_engine->removeCollisionBox(collision_idx);
}

void RS_Simulation::addCollisionMesh(double mu, double resti, double grid_size,
    const Vec3<double>& left_corner_loc,
    const DMat<double>& height_map,
    bool addToWindow, bool transparent) {
  _dyn_engine->addCollisionMesh(mu, resti, grid_size, left_corner_loc, height_map);
}

void RS_Simulation::applyExternalForceToBase(SVec<double> force) {
  vectorAligned<SVec<double>> _externalForces;
  for(size_t i(0); i<_dyn_engine->getNumBodies(); ++i){
    _externalForces.push_back(SVec<double>::Zero());
  }
  _externalForces[5] = force;
  _dyn_engine->setAllExternalForces(_externalForces);
}

void RS_Simulation::loadTerrainFile(const std::string& terrainFileName,
    bool addGraphics) {
  printf("load terrain %s\n", terrainFileName.c_str());
  ParamHandler paramHandler(terrainFileName);

  if (!paramHandler.fileOpenedSuccessfully()) {
    printf("[ERROR] could not open yaml file for terrain\n");
    throw std::runtime_error("yaml bad");
  }

  std::vector<std::string> keys = paramHandler.getKeys();

  for (auto& key : keys) {
    // define some lambda functions
    auto load = [&](double& val, const std::string& name) {
      if (!paramHandler.getValue<double>(key, name, val))
        throw std::runtime_error("terrain read bad: " + key + " " + name);
    };

    auto loadVec = [&](double& val, const std::string& name, size_t idx) {
      std::vector<double> v;
      if (!paramHandler.getVector<double>(key, name, v))
        throw std::runtime_error("terrain read bad: " + key + " " + name);
      val = v.at(idx);
    };

    auto loadArray = [&](double* val, const std::string& name, size_t idx) {
      std::vector<double> v;
      if (!paramHandler.getVector<double>(key, name, v))
        throw std::runtime_error("terrain read bad: " + key + " " + name);
      assert(v.size() == idx);
      for (size_t i = 0; i < idx; i++) val[i] = v[i];
    };

    printf("terrain element %s\n", key.c_str());
    std::string typeName;
    paramHandler.getString(key, "type", typeName);
    if (typeName == "infinite-plane") {
      double mu, resti, height, gfxX, gfxY, checkerX, checkerY;
      load(mu, "mu");
      load(resti, "restitution");
      load(height, "height");
      addCollisionPlane(mu, resti, height);
    } else if (typeName == "box") {
      double mu, resti, depth, width, height, transparent;
      double pos[3];
      double ori[3];
      load(mu, "mu");
      load(resti, "restitution");
      load(depth, "depth");
      load(width, "width");
      load(height, "height");
      loadArray(pos, "position", 3);
      loadArray(ori, "orientation", 3);
      load(transparent, "transparent");

      for(size_t i = 0; i < 3; i++){
        _simObjectLCM.position[i] = pos[i];
      }
      _simObjectLCM.scale[0] = depth;
      _simObjectLCM.scale[1] = width;
      _simObjectLCM.scale[2] = height;
      _lcm->publish("simulator_objects", &_simObjectLCM);

      Mat3<double> R_box = ori::rpyToRotMat(Vec3<double>(ori));
      R_box.transposeInPlace();  // collisionBox uses "rotation" matrix instead
      // of "transformation"
      addCollisionBox(mu, resti, depth, width, height, Vec3<double>(pos), R_box,
          addGraphics, transparent != 0.);
    } else if (typeName == "stairs") {
      double mu, resti, rise, run, stepsDouble, width, transparent;
      double pos[3];
      double ori[3];
      load(mu, "mu");
      load(resti, "restitution");
      load(rise, "rise");
      load(width, "width");
      load(run, "run");
      load(stepsDouble, "steps");
      loadArray(pos, "position", 3);
      loadArray(ori, "orientation", 3);
      load(transparent, "transparent");

      Mat3<double> R = ori::rpyToRotMat(Vec3<double>(ori));
      Vec3<double> pOff(pos);
      R.transposeInPlace();  // "graphics" rotation matrix

      size_t steps = (size_t)stepsDouble;

      double heightOffset = rise / 2;
      double runOffset = run / 2;
      for (size_t step = 0; step < steps; step++) {
        Vec3<double> p(runOffset, 0, heightOffset);
        p = R * p + pOff;

        addCollisionBox(mu, resti, run, width, heightOffset * 2, p, R,
            addGraphics, transparent != 0.);

        heightOffset += rise / 2;
        runOffset += run;
      }
    } else if (typeName == "mesh") {
      // TODO: issue with current framework, having two meshes in default-terrain.yaml
      // would likely be problematic
      double mu, resti, transparent, resolution;
      Vec3<double> left_corner;
      std::vector<std::vector<double> > height_map_2d;
      load(mu, "mu");
      load(resti, "restitution");
      load(transparent, "transparent");
      load(resolution, "resolution");
      loadVec(left_corner[0], "left_corner_loc", 0);
      loadVec(left_corner[1], "left_corner_loc", 1);
      loadVec(left_corner[2], "left_corner_loc", 2);

      int x_len = 0;
      int y_len = 0;
      bool file_input(false);
      paramHandler.getBoolean(key, "heightmap_file", file_input);
      if (file_input) {
        // Read from text file
        std::string file_name;
        paramHandler.getString(key, "heightmap_file_name", file_name);
        std::ifstream f_height;
        f_height.open(THIS_COM "/config/" + file_name);
        if (!f_height.good()) {
          std::cout << "file reading error: "
            << THIS_COM "../config/" + file_name << std::endl;
        }
        int i = 0, j = 0;
        double tmp;

        std::string line;
        std::vector<double> height_map_vec;
        while (getline(f_height, line)) {
          std::istringstream iss(line);
          j = 0;
          while (iss >> tmp) {
            height_map_vec.push_back(tmp);
            ++j;
          }
          y_len = j;
          height_map_2d.push_back(height_map_vec);
          height_map_vec.clear();
          // printf("y len: %d\n", y_len);
          ++i;
        }
        x_len = i;

      } else {
        paramHandler.get2DArray(key, "height_map", height_map_2d);
        x_len = height_map_2d.size();
        y_len = height_map_2d[0].size();
        // printf("x, y len: %d, %d\n", x_len, y_len);
      }

      terrainHeightmap = DMat<double>::Zero(x_len, y_len);
      for (int i = 0; i < x_len; ++i) {
        for (int j = 0; j < y_len; ++j) {
          terrainHeightmap(i, j) = height_map_2d[i][j];
          // printf("height (%d, %d) : %f\n", i, j, height_map(i,j) );
        }
      }
      addCollisionMesh(mu, resti, resolution, left_corner, terrainHeightmap, addGraphics,
          transparent != 0.);

      terrainResolution_ = resolution;
      terrainMeshLeftCorner = left_corner;
      terrainFile = true;
      terrainGridSize_[0] = x_len;
      terrainGridSize_[1] = y_len;
    } else {
      throw std::runtime_error("unknown terrain " + typeName);
    }
  }
}
