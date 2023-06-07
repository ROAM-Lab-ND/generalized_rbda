/*! @file VisualizationData.h
 *  @brief Data sent from robot code to simulator GUI for debugging
 *
 */

#ifndef VISUALIZATION_DATA_H
#define VISUALIZATION_DATA_H

#define VISUALIZATION_MAX_PATH_POINTS 2000
#define VISUALIZATION_MAX_PATHS 20
#define VISUALIZATION_MAX_ITEMS 10000
#define VISUALIZATION_MAX_CHEETAHS 0

#define VISUALIZATION_MAX_MESHES 5
#define VISUALIZATION_MAX_MESH_GRID 150

//Default information
#define DEFAULT_PATH_WIDTH 0.01  //[m] radius

#include "cppTypes.h"
#include <debug_visualization_lcmt.hpp>
#include <string>

/*!
 * Debugging sphere
 */
struct SphereVisualization {
  Vec3<float> position;
  Vec4<float> color;
  double radius;
};

/*!
 * Debugging box
 */
struct BlockVisualization {
  Vec3<float> dimension;
  Vec3<float> corner_position;
  Vec3<float> rpy;
  Vec4<float> color;
};

/*!
 * Debugging box obstacle
 */
struct ObstacleVisualization {
  Vec3<float> dimension;
  Vec3<float> center_position;
  Vec3<float> rpy;
  Vec4<float> color;
  float collision_idx;
};

/*!
 * Debugging arrow
 */
struct ArrowVisualization {
  Vec3<float> base_position;
  Vec3<float> direction;
  Vec4<float> color;
  std::string id;
  float nominal_magnitude; //Increasing it will decrease the size of the arrow
};

/*!
 * Debugging robot (draws the same type of robot as currently simulating)
 */
struct CheetahVisualization {
  //Vec12<float> q;
  Vec100<float> q; // Just big vector to address robot with many joints 
  //Vec16<float> q; // Humanoid DoF (16)
  Quat<float> quat;
  Vec3<float> p;
  Vec4<float> color;
};

/*!
 * Debugging "path"
 */
struct PathVisualization {
  size_t num_points = 0;
  Vec4<float> color;
  float width= DEFAULT_PATH_WIDTH;
  Vec3<float> position[VISUALIZATION_MAX_PATH_POINTS];
  void clear() {
    num_points = 0;
  }
};

/*!
 * Debugging Cone
 */
struct ConeVisualization {
  Vec3<float> point_position;
  Vec3<float> direction;
  Vec4<float> color;
  double radius;
};

/*!
 * Mesh Visualization
 */
struct MeshVisualization {
  Vec3<float> left_corner;
  Eigen::Matrix<float, VISUALIZATION_MAX_MESH_GRID, VISUALIZATION_MAX_MESH_GRID> height_map;

  int rows, cols;

  float grid_size;
  float height_max;
  float height_min;
};


/*!
 * Collection of all debugging data
 */
struct VisualizationData {
  size_t num_paths = 0, num_arrows = 0, num_cones = 0, num_spheres = 0,
         num_blocks = 0, num_meshes = 0, num_obstacles = 0;
  SphereVisualization spheres[VISUALIZATION_MAX_ITEMS];
  BlockVisualization blocks[VISUALIZATION_MAX_ITEMS];
  ObstacleVisualization obstacles[VISUALIZATION_MAX_ITEMS];
  ArrowVisualization arrows[VISUALIZATION_MAX_ITEMS];
  ConeVisualization cones[VISUALIZATION_MAX_ITEMS];
  PathVisualization paths[VISUALIZATION_MAX_PATHS];
  MeshVisualization meshes[VISUALIZATION_MAX_MESHES];

  //Default parameters
  Vec4<float> sphere_default_color= {1., 0.0, 0.00, 0.7};
  float sphere_default_radius =.05;

  Vec4<float> arrow_default_color= {0.95, 0.0, 0.00, 0.7};
  float arrow_default_magnitude =1.;

  Vec4<float> path_default_color= {0.0, 1.0, 0.00, 0.5};
  float path_default_width =1.;



  /*!
   * Remove all debug data
   */
  void clear() {
    num_paths = 0, num_arrows = 0, num_cones = 0, num_spheres = 0,
    num_blocks = 0, num_meshes = 0;
  }

  /*!
   * Add a new sphere
   * @return A sphere, or nullptr if there isn't enough room
   */
  SphereVisualization* addSphere() {
    if(num_spheres < VISUALIZATION_MAX_ITEMS) {
      return &spheres[num_spheres++];
    }
    return nullptr;
  }

    /*!
   * Add a new sphere with default parameters
   * @return A sphere, or nullptr if there isn't enough room
   */
    SphereVisualization* addSphere(Vec3<float> &pos) {
        if(num_spheres < VISUALIZATION_MAX_ITEMS) {
            spheres[num_spheres].color=sphere_default_color;
            spheres[num_spheres].radius = sphere_default_radius;
            spheres[num_spheres].position = pos;
            return &spheres[num_spheres++];
        }
        return nullptr;
    }

  /*!
   * Add a new box
   * @return A box, or nullptr if there isn't enough room
   */
  BlockVisualization* addBlock() {
    if(num_blocks < VISUALIZATION_MAX_ITEMS) {
      return &blocks[num_blocks++];
    }
    return nullptr;
  }

  /*!
   * Add a new box obstacle
   * @return A box, or nullptr if there isn't enough room
   */
  ObstacleVisualization* addObstacle() {
    if(num_obstacles < VISUALIZATION_MAX_ITEMS) {
      return &obstacles[num_obstacles++];
    }
    return nullptr;
  }

  /*!
   * Add a new arrow
   * @return An arrow, or nullptr if there isn't enough room
   */
  ArrowVisualization* addArrow() {
    if(num_arrows < VISUALIZATION_MAX_ITEMS) {
      return &arrows[num_arrows++];
    }
    return nullptr;
  }

    /*!
   * Add a new arrow with default parameters
   * @return An arrow, or nullptr if there isn't enough room
   */
    ArrowVisualization* addArrow(Vec3<float> &base_position, Vec3<float> &direction) {
        if(num_arrows < VISUALIZATION_MAX_ITEMS) {
            arrows[num_arrows].color=arrow_default_color;
            arrows[num_arrows].nominal_magnitude=arrow_default_magnitude;
            arrows[num_arrows].id = "Default";
            arrows[num_arrows].base_position=base_position;
            arrows[num_arrows].direction=direction;
            return &arrows[num_arrows++];
        }
        return nullptr;
    }

    /*!
    * Adds 3 arrows as the xyz directions of the reference frame
     * Takes reference frame position and orientation (3x3 rotation matrix
    * @returns nullptr
    */
    ArrowVisualization* addFrame(Vec3<float> &base_position, Mat3<float> &orientation) {
        //Reference frame colors
        Vec4<float> reference_frame_colors[3];
        reference_frame_colors[0]={1., 0.0, 0.00, 0.5};
        reference_frame_colors[1]={0., 1.0, 0.00, 0.5};
        reference_frame_colors[2]={0., 0.0, 1.00, 0.5};
        for(size_t i(0); i< 3; ++i) {
            if (num_arrows < VISUALIZATION_MAX_ITEMS) {
                arrows[num_arrows].color = reference_frame_colors[i];
                arrows[num_arrows].nominal_magnitude = 1.25;
                arrows[num_arrows].id = "Frame";
                arrows[num_arrows].base_position = base_position;
                arrows[num_arrows].direction = orientation.col(i);
//                &arrows[num_arrows++];
                num_arrows++;
            }
        }
        return nullptr;
    }

  /*!
   * Add a new cone
   * @return A cone, or nullptr if there isn't enough room
   */
  ConeVisualization* addCone() {
    if(num_cones < VISUALIZATION_MAX_ITEMS) {
      return &cones[num_cones++];
    }
    return nullptr;
  }

  /*!
   * Add a new path
   * @return A path, or nullptr if there isn't enough room
   */
  PathVisualization* addPath() {
    if(num_paths < VISUALIZATION_MAX_PATHS) {
      auto* path = &paths[num_paths++];
      path->clear();
      return path;
    }
    return nullptr;
  }

    /*!
   * Add a new path with default parameters
   * @return A path, or nullptr if there isn't enough room
   */
    PathVisualization* addPath(size_t num_points, Vec3<float> *positions) {
        if(num_paths < VISUALIZATION_MAX_PATHS) {
            paths[num_paths].clear();
            paths[num_paths].num_points=num_points;
            paths[num_paths].color =path_default_color;
            for (size_t i(0); i< paths[num_paths].num_points; i++){
                paths[num_paths].position[i]=positions[i];
            }
            return &paths[num_paths++];
        }
        return nullptr;
    }

  /*!
   * Add a new Mesh
   * @return A mesh, or nullptr if there isn't enough room
   */
   MeshVisualization* addMesh() { 
    if(num_paths < VISUALIZATION_MAX_MESHES) {
      return &meshes[num_meshes++];
    }
    return nullptr;
  }
  
  void buildMessageLCM(debug_visualization_lcmt & _debug_lcmt){
    // Encode spheres
    if (num_spheres){
      _debug_lcmt.sphere_count=num_spheres;
      _debug_lcmt.sphere_elements=num_spheres;
      _debug_lcmt.sphere_position_elements=num_spheres*3;
      _debug_lcmt.sphere_color_elements=num_spheres*4;
      _debug_lcmt.sphere_positions.resize(_debug_lcmt.sphere_position_elements);
      _debug_lcmt.sphere_radii.resize(_debug_lcmt.sphere_elements);
      _debug_lcmt.sphere_colors.resize(_debug_lcmt.sphere_color_elements);
      for(size_t i(0); i< num_spheres; ++i){
        _debug_lcmt.sphere_positions[3*i]=spheres[i].position[0];
        _debug_lcmt.sphere_positions[3*i+1]=spheres[i].position[1];
        _debug_lcmt.sphere_positions[3*i+2]=spheres[i].position[2];
        _debug_lcmt.sphere_radii[i]=(float)spheres[i].radius;
        _debug_lcmt.sphere_colors[4*i]=spheres[i].color[0];
        _debug_lcmt.sphere_colors[4*i+1]=spheres[i].color[1];
        _debug_lcmt.sphere_colors[4*i+2]=spheres[i].color[2];
        _debug_lcmt.sphere_colors[4*i+3]=spheres[i].color[3];
      }
    }
    else{
      _debug_lcmt.sphere_count=0;
      _debug_lcmt.sphere_elements=1;
      _debug_lcmt.sphere_position_elements=1;
      _debug_lcmt.sphere_color_elements=1;
      _debug_lcmt.sphere_positions.resize(_debug_lcmt.sphere_position_elements);
      _debug_lcmt.sphere_radii.resize(_debug_lcmt.sphere_elements);
      _debug_lcmt.sphere_colors.resize(_debug_lcmt.sphere_color_elements);
      _debug_lcmt.sphere_radii[0]=0.0f;
      _debug_lcmt.sphere_positions[0]=0.0f;
      _debug_lcmt.sphere_colors[0]=0.0f;
    }

    // Encode Obstacles
    if (num_obstacles){
      _debug_lcmt.obst_count=num_obstacles;
      _debug_lcmt.obst_elements=num_obstacles;
      _debug_lcmt.obst_position_elements=num_obstacles*3;
      _debug_lcmt.obst_rpy_elements=num_obstacles*3;
      _debug_lcmt.obst_dim_elements=num_obstacles*3;
      _debug_lcmt.obst_color_elements=num_obstacles*4;
      _debug_lcmt.obst_positions.resize(_debug_lcmt.obst_position_elements);
      _debug_lcmt.obst_oris.resize(_debug_lcmt.obst_rpy_elements);
      _debug_lcmt.obst_dims.resize(_debug_lcmt.obst_dim_elements);
      _debug_lcmt.obst_colors.resize(_debug_lcmt.obst_color_elements);
      for(size_t i(0); i< num_obstacles; ++i){
        _debug_lcmt.obst_positions[3*i]=obstacles[i].center_position[0];
        _debug_lcmt.obst_positions[3*i+1]=obstacles[i].center_position[1];
        _debug_lcmt.obst_positions[3*i+2]=obstacles[i].center_position[2];

        _debug_lcmt.obst_oris[3*i]=obstacles[i].rpy[0];
        _debug_lcmt.obst_oris[3*i+1]=obstacles[i].rpy[1];
        _debug_lcmt.obst_oris[3*i+2]=obstacles[i].rpy[2];

        _debug_lcmt.obst_dims[3*i] =obstacles[i].dimension[0];
        _debug_lcmt.obst_dims[3*i+1] =obstacles[i].dimension[1];
        _debug_lcmt.obst_dims[3*i+2] =obstacles[i].dimension[2];

        _debug_lcmt.obst_colors[4*i]=obstacles[i].color[0];
        _debug_lcmt.obst_colors[4*i+1]=obstacles[i].color[1];
        _debug_lcmt.obst_colors[4*i+2]=obstacles[i].color[2];
        _debug_lcmt.obst_colors[4*i+3]=obstacles[i].color[3];
      }
    }
    else{
      _debug_lcmt.obst_count=0;
      _debug_lcmt.obst_elements=1;
      _debug_lcmt.obst_position_elements=1;
      _debug_lcmt.obst_rpy_elements=1;
      _debug_lcmt.obst_dim_elements=1;
      _debug_lcmt.obst_color_elements=1;
      _debug_lcmt.obst_positions.resize(_debug_lcmt.obst_position_elements);
      _debug_lcmt.obst_oris.resize(_debug_lcmt.obst_rpy_elements);
      _debug_lcmt.obst_dims.resize(_debug_lcmt.obst_dim_elements);
      _debug_lcmt.obst_colors.resize(_debug_lcmt.obst_color_elements);
      _debug_lcmt.obst_positions[0]=0.0f;
      _debug_lcmt.obst_dims[0]=0.0f;
      _debug_lcmt.obst_colors[0]=0.0f;
    }

    //Encode paths
    if(num_paths){
      int num_path_points=0;
      for(size_t i(0); i< num_paths; ++i){ //get the number of path points
        num_path_points+=paths[i].num_points;
      }
      _debug_lcmt.path_count=num_paths;
      _debug_lcmt.path_elements=num_paths;
      _debug_lcmt.path_position_elements=num_path_points*3;
      _debug_lcmt.path_color_elements=num_paths*4;
      _debug_lcmt.path_lengths.resize(_debug_lcmt.path_elements);
      _debug_lcmt.path_start_idxs.resize(_debug_lcmt.path_elements);
      _debug_lcmt.path_widths.resize(_debug_lcmt.path_elements);
      _debug_lcmt.path_positions.resize(_debug_lcmt.path_position_elements);
      _debug_lcmt.path_colors.resize(_debug_lcmt.path_color_elements);
      int path_point_ctr=0;
      for(size_t i(0); i< num_paths; ++i){ //get the number of path points
        _debug_lcmt.path_lengths[i]=paths[i].num_points;
        _debug_lcmt.path_start_idxs[i]=path_point_ctr;
        _debug_lcmt.path_widths[i]=paths[i].width; //need to create a width member in visualizationData

        _debug_lcmt.path_colors[4*i]=paths[i].color[0];
        _debug_lcmt.path_colors[4*i+1]=paths[i].color[1];
        _debug_lcmt.path_colors[4*i+2]=paths[i].color[2];
        _debug_lcmt.path_colors[4*i+3]=paths[i].color[3];

        for(size_t j(0); j< paths[i].num_points; ++j){ //get the number of path points
          _debug_lcmt.path_positions[path_point_ctr]=paths[i].position[j][0];
          path_point_ctr++;
          _debug_lcmt.path_positions[path_point_ctr]=paths[i].position[j][1];
          path_point_ctr++;
          _debug_lcmt.path_positions[path_point_ctr]=paths[i].position[j][2];
          path_point_ctr++;
        }
      }
    }
    else{
      _debug_lcmt.path_count=0;
      _debug_lcmt.path_elements=1;
      _debug_lcmt.path_position_elements=1;
      _debug_lcmt.path_color_elements=1;
      _debug_lcmt.path_lengths.resize(_debug_lcmt.path_elements);
      _debug_lcmt.path_start_idxs.resize(_debug_lcmt.path_elements);
      _debug_lcmt.path_widths.resize(_debug_lcmt.path_elements);
      _debug_lcmt.path_positions.resize(_debug_lcmt.path_position_elements);
      _debug_lcmt.path_colors.resize(_debug_lcmt.path_color_elements);
      _debug_lcmt.path_lengths[0]=0;
      _debug_lcmt.path_start_idxs[0]=0;
      _debug_lcmt.path_widths[0]=0;
      _debug_lcmt.path_positions[0]=0;
      _debug_lcmt.path_colors[0]=0;
    }


    //Encode Arrows
    if (num_arrows){
      _debug_lcmt.arrow_count=num_arrows;
      _debug_lcmt.arrow_elements=num_arrows;
      _debug_lcmt.arrow_position_elements=num_arrows*3;
      _debug_lcmt.arrow_color_elements=num_arrows*4;
      _debug_lcmt.arrow_base_positions.resize(_debug_lcmt.arrow_position_elements);
      _debug_lcmt.arrow_directions.resize(_debug_lcmt.arrow_position_elements);
      _debug_lcmt.arrow_colors.resize(_debug_lcmt.arrow_color_elements);
      _debug_lcmt.arrow_ids.resize(_debug_lcmt.arrow_elements);
      _debug_lcmt.arrow_nom.resize(_debug_lcmt.arrow_elements);
      for(size_t i(0); i< num_arrows; ++i){
        _debug_lcmt.arrow_base_positions[3*i]=arrows[i].base_position[0];
        _debug_lcmt.arrow_base_positions[3*i+1]=arrows[i].base_position[1];
        _debug_lcmt.arrow_base_positions[3*i+2]=arrows[i].base_position[2];
        _debug_lcmt.arrow_directions[3*i]=arrows[i].direction[0];
        _debug_lcmt.arrow_directions[3*i+1]=arrows[i].direction[1];
        _debug_lcmt.arrow_directions[3*i+2]=arrows[i].direction[2];
        _debug_lcmt.arrow_colors[4*i]=arrows[i].color[0];
        _debug_lcmt.arrow_colors[4*i+1]=arrows[i].color[1];
        _debug_lcmt.arrow_colors[4*i+2]=arrows[i].color[2];
        _debug_lcmt.arrow_colors[4*i+3]=arrows[i].color[3];
        _debug_lcmt.arrow_ids[i]=arrows[i].id;
        _debug_lcmt.arrow_nom[i]=arrows[i].nominal_magnitude;
      }
    }
    else{ //not needed technically since always sending the ground reaction force arrows
      _debug_lcmt.arrow_count=0;
      _debug_lcmt.arrow_elements=1;
      _debug_lcmt.arrow_position_elements=1;
      _debug_lcmt.arrow_color_elements=1;
      _debug_lcmt.arrow_base_positions.resize(_debug_lcmt.arrow_position_elements);
      _debug_lcmt.arrow_directions.resize(_debug_lcmt.arrow_position_elements);
      _debug_lcmt.arrow_colors.resize(_debug_lcmt.arrow_color_elements);
      _debug_lcmt.arrow_ids.resize(_debug_lcmt.arrow_elements);
      _debug_lcmt.arrow_nom.resize(_debug_lcmt.arrow_elements);
      _debug_lcmt.arrow_base_positions[0]=0;
      _debug_lcmt.arrow_directions[0]=0;
      _debug_lcmt.arrow_colors[0]=0;
      _debug_lcmt.arrow_ids[0] = "";
      _debug_lcmt.arrow_nom[0] = 1.0;
    }
  }
};

#endif
