## Cluster Joints

Cluster joints are kinematic constraints between clusters of rigid-bodies.
Like conventional joints, they describe the motion that is permitted between these groups of bodies.
However, unlike conventional joints, they can include additional [loop constraints](#loop-constraints) that must be considered when computing the dynamics of the system.
This project implements a `ClusterJoint::Base` class that describes the common functionality of all cluster joints.
The main feature of this class is the `ClusterJoint::Base::updateKinematics` function that updates the `S`, `vJ`, and `cJ` quantities for the cluster joint.
These quantities are defined as follows:
- `S`: The motion subspace matrix for the cluster joint. Note that we include the effect of the explicit loop constraint on the motion subspace matrix, so it has dimension $6N \times n_i$, where $N$ is the number of bodies in the cluster joint and $n_i$ is the number of independent degrees of freedom of the cluster joint.
- `vJ`: The joint velocity vector, $\mathsf{v}_J = \mathsf{S}\dot{\mathsf{y}}$. It is the concatenation of the relative velocities of each body in the cluster joint with respect to their output bodies in the parent cluster. It is a vector of length $6N$, where $N$ is the number of bodies in the cluster joint.
- `cJ`: The joint velocity bias vector, $\mathsf{c}_J = \mathring{\mathsf{S}}\dot{\mathsf{y}}$, where $\mathring{\mathsf{S}}$ is the apparent derivative of the motion subspace matrix. It is the concatenation of the relative velocity biases of each body in the cluster joint with respect to their output bodies in the parent cluster ($c_J = \dot{S}\dot{q}$). It is a vector of length $6N$, where $N$ is the number of bodies in the cluster joint.

The list of currently supported cluster joints is given below:
- `ClusterJoint::Free`: Free joint that can be used to create a floating-base robot. Technically there is no loop constraint for this cluster joint, but algorithmically we treat it as a trivial "identity" loop constraint.
- `ClusterJoint::Revolute`: Conventional revolute joint between two bodies. Technically there is no loop constraint for this cluster joint, but algorithmically we treat it as a trivial "identity" loop constraint.
- `ClusterJoint::RevolutePair`: Exists for testing purposes only. It clusters two bodies together in serial, both connected to their parent with a revolute joint. Technically there is no loop constraint for this cluster joint, but algorithmically we treat it as a trivial "identity" loop constraint.
- `ClusterJoint::RevolutePairWithRotor`: Connects four rigid-bodies (two links, two rotors) to a single output body. The two links are connected in serial with revolute joints, and the two rotors are connected directly to the output body with revolute joints. The loop constraint encodes the kinematic constraint of the parallel belt transmission that relates the links and rotors. This is the joint used for the knee and ankle of the MIT Humanoid (TODO: provide link to humanoids 2023 paper).
- `ClusterJoint::RevoluteTripleWithRotor`: Connects six rigid-bodies (three links, three rotors) to a single output body. The three links are connected in serial with revolute joints, and the three rotors are connected directly to the output body with revolute joints. The loop constraint encodes the kinematic constraint of the parallel belt transmission that relates the links and rotors. This is the joint used for the hip of the [MIT Biomimetic Robotics Lab Manipulation Platform](https://ieeexplore.ieee.org/document/10160930).
- `ClusterJoint::RevoluteWithRotor`: Connects two rigid-bodies (one link and one rotor) to a single output body. The loop constraint encodes the kinematics constraint of the geared transmission that relates the link and rotor. This is a common actuation sub-mechanism used in many robots.
- `ClusterJoint::TelloDifferential`: Connects four rigid-bodies (two links and two rotors) to a single output body. The two links are connected in serial with revolute joints, and the two rotors are connected directly to the output body with revolute joints. The loop constraint encodes the kinematic constraint of the differential drive involving the links and rotors. This is the joint used for the hip and the knee and ankle of the [Tello Humanoid](https://ieeexplore.ieee.org/document/9813569) robot.
- `ClusterJoint::Generic`: A generic cluster joint that can be used to create any cluster. It is constructed from (i) the collection of rigid-bodies in the cluster, (ii) the collection of tree joints for each of those rigid-bodies, (iii) a `LoopConstraint` object that describes the loop constraint for the cluster. It is the most general, but also the slowest because, unlike the specialized classes, it does not exploit any sparsity to reduce computation time.

## Loop Constraints

Loop constraints are additional kinematic constraints between bodies in a cluster beyond those imposed by conventional joints. 
They can be expressed in two ways:

|          | Position | Velocity | Acceleration |
| -------- | ------- | -------- | ------------ |
| **implicit:** | $\phi(\mathbf{q}) = \mathbf{0}$ | $\mathbf{K}\dot{\mathbf{q}} = \mathbf{0}$ | $\mathbf{K}\ddot{\mathbf{q}} = \mathbf{k}$ |
| **explicit:** | $\mathbf{q} = \gamma(\mathbf{y}) $ | $\dot{\mathbf{q}} = \mathbf{G}\dot{\mathbf{y}} $ | $\ddot{\mathbf{q}} = \mathbf{G}\ddot{\mathbf{y}} + \mathbf{g}$ |

where,
$$\mathbf{K} = \frac{\partial \phi}{\partial \mathbf{q}}, \quad \mathbf{k} = -\dot{\mathbf{K}}\dot{\mathbf{q}}, \quad \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}}, \quad \mathbf{g} = \dot{\mathbf{G}}\dot{\mathbf{y}}.$$
For a more thorough treatment of loop constraints, see Chapters 3.2 and 4.1 of [Rigid Body Dynamics Algorithms](https://link.springer.com/book/10.1007/978-1-4899-7560-7).

Every `ClusterJoint` has an associated `LoopConstraint` object that describes the loop constraint associated with that joint.
The `LoopConstraint` class is an abstract base class that defines the common functionality of all loop constraints.
The list of currently supported loop constraints is given below:
- `LoopConstraint::Static`: The constraint Jacobians $\mathbf{G}$ and $\mathbf{K}$ are constast, and the constraint biases $\mathbf{g}$ and $\mathbf{k}$ are zero. The are constructed from $\mathbf{G}$ and $\mathbf{K}$.
- `LoopConstraint::Free`: A trivial constraint for free joints. It is different from the `Static` constraint because it accounts for the fact that the number of independent positions (7) is not equal to the number of independent degrees of freedom (6).
- `LoopConstraint::TelloDifferential`: A nonlinear, implicit constraint. [CasADi](/include/grbda/Codegen/CasadiGen.h) generated functions are used to compute the constraint Jacobians and biases. This class is specific to the [Tello Humanoid](https://ieeexplore.ieee.org/document/9813569) robot, but serves as a good example of how to implement a custom loop constraint using CasADi.
