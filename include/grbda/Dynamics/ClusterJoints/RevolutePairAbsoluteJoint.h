#ifndef GRBDA_CLUSTER_JOINTS_REVOLUTE_PAIR_ABSOLUTE_H
#define GRBDA_CLUSTER_JOINTS_REVOLUTE_PAIR_ABSOLUTE_H

#include "grbda/Dynamics/ClusterJoints/ClusterJoint.h"
#include <casadi/casadi.hpp>

namespace grbda
{
namespace ClusterJoints
{

template <typename Scalar = double>
class RevolutePairAbsolute : public Base<Scalar>
{
public:
    RevolutePairAbsolute(ori::CoordinateAxis axis1, 
                         ori::CoordinateAxis axis2,
                         const spatial::Transform<Scalar>& X_tree_internal);
    ~RevolutePairAbsolute() {}

    ClusterJointTypes type() const override { return ClusterJointTypes::RevolutePair; }

    void updateKinematics(const JointState<Scalar>& joint_state) override;

    void computeSpatialTransformFromParentToCurrentCluster(
        spatial::GeneralizedTransform<Scalar>& Xup) const override;

    std::vector<DMat<Scalar>> getSq() const override;
    DMat<Scalar> getSdotqd_qd() const override;

private:
    const ori::CoordinateAxis axis1_;
    const ori::CoordinateAxis axis2_;
    const spatial::Transform<Scalar> X_tree_internal_;

    mutable DVec<Scalar> q_cache_;
    mutable DVec<Scalar> qd_cache_;

    mutable spatial::Transform<Scalar> XJ1_cache_;
    mutable spatial::Transform<Scalar> X21_cache_;

    mutable casadi::Function f_dS_dq1_;
    mutable casadi::Function f_dS_dq2_;
    mutable casadi::Function f_Sdotqd_qd_;
    mutable bool casadi_functions_initialized_;

    void initializeCasadiFunctions() const;
    char axisToChar(ori::CoordinateAxis axis) const;
};

} // namespace ClusterJoints
} // namespace grbda

#endif
