# Generalized Rigid Body Dynamics Algorithms

## Summary
This repository contains a C++ library of rigid-body dynamics algorithms that are **compatible with kinematic loops** and especially efficient for those induced by common **actuation sub-mechanisms** like geared transmissions, differential drives, and closed-chain linkages.  
The algorithms are based on the principle of **constraint embedding**, originally developed by Jain [1](https://watermark.silverchair.com/139_1.pdf?token=AQECAHi208BE49Ooan9kkhW_Ercy7Dm3ZL_9Cf3qfKAc485ysgAABPYwggTyBgkqhkiG9w0BBwagggTjMIIE3wIBADCCBNgGCSqGSIb3DQEHATAeBglghkgBZQMEAS4wEQQMVpAz0QBG_TLnW6dtAgEQgIIEqcYTCuhh4cgP4Ki2supwv4C5NUDfGljTL8R22IlekhuRRYjOg8TTnUUPM1Uz_g6SC-PbIus0bXqaSjuwsYiTbNTyGAF8BKWh2M2E8XrB5ElCYsRLVLoX4cM9BZN0faog65pCiPr_7kkRyaq8oxFu_T0l0t-3FzlinLwk9GGttXg1Nw6zxAc5t3AYxRw3yl86XDt83foeEWCtcO7-LOBqIiX6EpVZcPm--Ajbys3TZSTC5fA81-H9VBn5Sp2ZZgRZdXvfAF9R0zRcuZ5L-037imoFQVLNMEyner8r5KKJjuEQzinT63seEdvyK9lKaX2BHz7QeXqJm4QB7tapDXgy-jdrWgWBkH8JlRJOzWWzm1fGRFCi3jvAzFRePYNTMCJqxB9rJJhMchJK2vIqeDNhZxQmPmtb0Wq1wY7exRene4Lbmnde9E645BTsntrmjEoolg4idKBReOR3UgVR_bOeJ-7AJ8VnltYNW8UkXrl7HSOxN5Hd-_VESQLnVXfI5H2nTC0SBb17mmCvaoZjmz29VXdfArnSdwI9KQZRPuXBjiubqbPI8p5CxmvMQxIbYzusIHf6E34LFKWs-WSkO61KOfuFwByR4DJJLBTJrXYJz4u5G4Deccbokup4XMKumrA0QVnteOwC1sUuHNbbGqew6uf_xL-XlgctHjS3r0-5moSy_hYcsyeIBE7pT7tSuIYvuovjHIPeIlHPBNcvA4odVfEYWUWUdqsPFWjdIykEb439w6A4vIhDrVWJqhTucGp70PT0sXXw3d3A9JmjqQXd8UDBpM6nDCNHkDvmQyWXSpoiki-nopKXJVrhcuPuGW0JNZZm8w39N57pOBpPBgyOC2NjBBOwZBDXY1FmlSxnDWBRxkPt3sG_Q3qSAjUwiico8lPn0QJtUulVGceXteP6rjkqZHjNaPM5jNdSWNQ6VDHV_8MrxlCiEOKho5H4mPN6BRINP66eCQlvXRhEKDPhhMI8R1PzhGH8DotSg065JECCAn41WsJwLquJ1jQLToDnLJhx3MIoY8_26CZ36oMgSyzgX9OYrl9qBsmuKFimvp9oiTLIJ19wh6T_DwH6ipavqrSB1fOQ7TaYlWfPffe_9z2Ur9IdD5XIM5mVRh9GAdx6wtWD-iGFrloa1RpnLa_cpnVQLpr7OoH2pr7Kaip_5RfRanx3zVALJAwJDTQCMh4zGmrEotdo8X2xUhHIxUSKyKOOPVwe1s_3kS2hTjenm4WOoZtqCX7AKQNRlsWOdoc3uAiOiiBs_ZrcQIHDik_Vd80QApphnQl7NZ97oaMJ31zmo_ztiJBZYL9BpwT1wIBqs5auKMscIJDdHHr4qihegBujtrsWCMRB7UwF2DZkPW10jnHmaudboN-LynhlM1mGKjaCG6-m9Uk4mOdo0lStOGloVm9zfXnZDtdm6J38Qdf0HuQ0_ghK_liZOMNsz4TnejxOAUJlzN3Q9CjE7Pz7BrM5j-XOwJkzeDw13OiCTjSIqodSc55Q12CyFvmOGWv3gxpMBVEB-qcHVXY_k4gONd5MD0F4mnA0uIhElvaWpC3LEPuwAwL0HFmkOfxHKMuB9cD6WINcNbbP), and revisited through our propagation-based perspective.
Details of this propagation perspective are presented in our [accompanying paper](https://arxiv.org/abs/2311.13732).
Our software reflects this **complementary formulation** that generalizes the traditional concept of joint models and motion/force subspaces—extending them from single rigid bodies to **groups of bodies** linked through local loops.

__Note__: For notational continuity with the literature, our paper refers to these groups of bodies as "aggregate links" and the constraints between them as "aggregate joints". However, in this repository, we refer to them as "clusters" and "cluster joints" respectively.

Actuation sub-mechanisms are increasingly common in robotic systems, yet no open-source library existed to compute their dynamics efficiently.  
The loops these mechanisms induce violate assumptions of conventional recursive dynamics algorithms.  
**Constraint embedding** resolves this by **grouping bodies into clusters**, allowing the loop constraints to be resolved **locally** during forward/backward passes.  
This repository provides the first open-source implementation of these ideas in a modern, templated C++ library.

<div style="text-align:center;">
   <img src="images/ActuationSubMechanisms.png" alt="drawing" width="300"/>
</div>

## Table of Contents

1. [Installation](#installation)
2. [Usage](#usage)
3. [Configuration](#configuration)
6. [License](#license)
7. [Contact Information](#contact-information)
8. [Citation](#citation)

## Installation
### Dependencies
- C++17 or higher
- CMake 3.15 or higher
- Eigen 3.3.7 or higher
- Casadi 3.6.3 or higher

They can be installed using the provided [install script](scripts/install_dependencies.sh).

### Supported Operating Systems
- Ubuntu 18.04 or higher
- MacOS 10.15 or higher (Note: When compiling on a machine with Apple M1 chip, you can optimize the performance by using the following command: `cmake -DM1_BUILD=ON ..`)

### Building
To run the unit tests and benchmarks
- Create a build directory: `mkdir build && cd build`
- Run CMake: `cmake -DBUILD_BENCHMARKS=ON ..`
- Build: `make`
- Unit tests can be run all at once with `ctest` or individually with `./bin/<name-of-unit-test-binary>`
- The benchmark can be run with `./bin/pinocchio_benchmark` and the output files are at `Benchmarking/data/`

Optionally, a dockerfile is provided to run the benchmarks:
- Build the container using [this script](grbda-docker/build_container)
- Start the container using [this script](grbda-docker/start_container)
- (within the container) `mkdir build && cd build`
- (within the container) Run CMake: `cmake -DBUILD_BENCHMARKS=ON ..`
- (within the container) Build: `make`
- (within the container, optional) Unit tests: `ctest`
- (within the container) Run benchmark: `./bin/pinocchio_benchmark` and see the output files at `Benchmarking/data/`

To incorporate the library into your own project, see [Configuration](#configuration).

## Usage

As a general purpose rigid-body dynamics library, the project can be used in a variety of ways.
The most common uses are as follows:
- **Dynamic Simulation**: Using the constrained forward dynamics algorithms and the contact dynamics algorithms, the repository can be used as part of a simulator for contact-rich robotic systems.
- **Model-Based Control**: Using the inverse dynamics algorithms, the repository can be used to as part of a whole-body controller to compute the control torques required to achieve a desired state.

We offer multiple ways to construct a model, including:
- **Manual Model Construction**: The user can manually construct a model by following these steps:
   - Register all of the bodies contained in a cluster by providing their connectivity and inertia information.
   - Append those bodies to the model by providing information about the cluster joint that connects them.
   - Repeat the above steps for each cluster in the system.

   User can leverage the provided specialized classes for common cluster joint types. Specific examples of how to build a model can be found in the [README](include/grbda/Dynamics/README.md) detailing the process.
- **URDF Parsing**: For robots without kinematics loops, the user can parse a standard URDF file to automatically construct the model. For systems with loops, we support the [URDF+](https://github.com/mit-biomimetics/urdfdom) format—a minimal extension of URDF to specify kinematic loops. URDF+ preserves compatibility with existing tooling while enabling loop specification directly in XML. We refer readers to the [URDF+ format paper](https://ieeexplore.ieee.org/document/10769903) for full details.

## Configuration

You can use this repository in your project using CMake by:
1. Adding the `include` directory in your project's include path.
2. Linking against the `generalized_rbda` library.

The following example shows how to do this in CMake:
```cmake
set(GRBDA_INCLUDE_DIR /path/to/generalized_rbda/include)
find_library(GRBDA_LIBRARY
    NAMES generalized_rbda
    HINTS ${GRBDA_INCLUDE_DIR}/../build)
if(GRBDA_LIBRARY)
    set(GRBDA_LIBRARIES ${GRBDA_LIBRARIES} ${GRBDA_LIBRARY})
endif()

include_directories(`${GRBDA_INCLUDE_DIR}`)
target_link_libraries(${PROJECT_NAME} ${GRBDA_LIBRARIES})
```

## License
The code is licensed under the MIT license. See the [LICENSE](LICENSE) file for more details.

## Contact Information:
For further questions or collaboration inquiries, please contact the developers at:
- [Matthew Chignoli](mailto:chignoli@mit.edu)

## Citation
To cite this library in your research, please use the following citation for the accompanying paper:
```
@article{chignoli2023recursive,
  title={A Propagation Perspective on Recursive Forward Dynamics for Systems with Kinematic Loops},
  author={Chignoli, Matthew and Adrian, Nicholas and Kim, Sangbae and Wensing, Patrick M.},
  journal={arXiv preprint arXiv:2311.13732},
  year={2024}
}
```
and the following citation for the codebase:
```
@misc{grbdaweb,
   author = {Matthew Chignoli, Nicholas Adrian, and Patrick M. Wensing},
   title = {GRBDA: Generalized Rigid-Body Dynamics Algorithms},
   howpublished = {https://github.com/ROAM-Lab-ND/generalized_rbda},
   year = {2023}
}
```
