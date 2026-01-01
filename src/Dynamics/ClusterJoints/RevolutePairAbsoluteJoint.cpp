#include "grbda/Dynamics/ClusterJoints/RevolutePairAbsoluteJoint.h"
#include "grbda/Utils/CasadiDerivatives.h"

namespace grbda
{
namespace ClusterJoints
{

template <typename Scalar>
RevolutePairAbsolute<Scalar>::RevolutePairAbsolute(
    ori::CoordinateAxis axis1,
    ori::CoordinateAxis axis2,
    const spatial::Transform<Scalar>& X_tree_internal)
    : Base<Scalar>(2, 2, 2),
      axis1_(axis1),
      axis2_(axis2),
      X_tree_internal_(X_tree_internal),
      casadi_functions_initialized_(false)
{
    q_cache_ = DVec<Scalar>::Zero(2);
    qd_cache_ = DVec<Scalar>::Zero(2);
    
    this->S_ = DMat<Scalar>::Zero(12, 2);
    this->Psi_ = DMat<Scalar>::Zero(12, 2);
    this->vJ_ = DVec<Scalar>::Zero(12);
    this->cJ_ = DVec<Scalar>::Zero(12);
    this->S_ring_ = DMat<Scalar>::Zero(12, 2);
}

template <typename Scalar>
char RevolutePairAbsolute<Scalar>::axisToChar(ori::CoordinateAxis axis) const
{
    switch (axis)
    {
        case ori::CoordinateAxis::X: return 'X';
        case ori::CoordinateAxis::Y: return 'Y';
        case ori::CoordinateAxis::Z: return 'Z';
        default: return 'Z';
    }
}

template <typename Scalar>
void RevolutePairAbsolute<Scalar>::updateKinematics(const JointState<Scalar>& joint_state)
{
    const DVec<Scalar>& q = joint_state.position;
    const DVec<Scalar>& qd = joint_state.velocity;

    q_cache_ = q;
    qd_cache_ = qd;

    const Scalar q1_abs = q(0);
    const Scalar q2_abs = q(1);
    const Scalar q_rel = q2_abs - q1_abs;

    const auto XJ1 = spatial::rotation<Scalar>(axis1_, q1_abs);
    const auto XJ2 = spatial::rotation<Scalar>(axis2_, q_rel);

    const DVec<Scalar> S1 = spatial::jointMotionSubspace<Scalar>(spatial::JointType::Revolute, axis1_);
    const DVec<Scalar> S2 = spatial::jointMotionSubspace<Scalar>(spatial::JointType::Revolute, axis2_);

    const auto X21 = XJ2 * X_tree_internal_;

    // Cache transforms for computeSpatialTransformFromParentToCurrentCluster
    XJ1_cache_ = XJ1;
    X21_cache_ = X21;

    this->S_.setZero();
    this->S_.template block<6, 1>(0, 0) = S1;
    this->S_.template block<6, 1>(6, 0) = X21.transformMotionSubspace(S1) - S2;
    this->S_.template block<6, 1>(6, 1) = S2;

    this->Psi_ = this->S_;
    this->vJ_ = this->S_ * qd;
    this->cJ_.setZero();
    this->S_ring_.setZero();
}

template <typename Scalar>
void RevolutePairAbsolute<Scalar>::computeSpatialTransformFromParentToCurrentCluster(
    spatial::GeneralizedTransform<Scalar>& Xup) const
{
#ifdef DEBUG_MODE
    if (Xup.getNumOutputBodies() != 2)
        throw std::runtime_error("[RevolutePairAbsolute] Xup must have 12 rows");
#endif

    // Body 1 transform: XJ1 (rotation by q1_abs)
    Xup[0] = XJ1_cache_;

    // Body 2 transform: X21 * XJ1 (compound rotation)
    Xup[1] = X21_cache_ * Xup[0];
}

template <typename Scalar>
void RevolutePairAbsolute<Scalar>::initializeCasadiFunctions() const
{
    if (casadi_functions_initialized_) return;
    
    using namespace casadi;
    using namespace casadi_derivatives;
    
    SX q1 = SX::sym("q1");
    SX q2 = SX::sym("q2");
    SX q_rel = q2 - q1;
    
    char ax1 = axisToChar(axis1_);
    char ax2 = axisToChar(axis2_);
    
    SX S1_sym = revoluteMotionSubspace(ax1);
    SX S2_sym = revoluteMotionSubspace(ax2);

    // Convert X_tree_internal to CasADi DM (numeric constant), then to SX
    DMat<double> X_internal_eigen = X_tree_internal_.toMatrix().template cast<double>();
    std::vector<double> X_internal_vec(36);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            X_internal_vec[i * 6 + j] = X_internal_eigen(i, j);
        }
    }
    DM X_internal_dm = DM(reshape(DM(X_internal_vec), 6, 6));
    SX X_internal_sx = SX(X_internal_dm);  // Convert DM to SX

    SX XJ2_sym = spatialRotation(ax2, q_rel);
    SX X21_sym = mtimes(XJ2_sym, X_internal_sx);
    
    SX S_col1_body2 = mtimes(X21_sym, S1_sym) - S2_sym;

    // Compute ∂S/∂q directly
    SX dS_dq1 = jacobian(S_col1_body2, q1);
    SX dS_dq2 = jacobian(S_col1_body2, q2);

    // For Sdot*qd derivatives, we need ∂(Ṡ·q̇)/∂q̇
    // Ṡ = ∂S/∂q · q̇, so Ṡ·q̇ = (∂S/∂q · q̇)·q̇
    // For the body 2 component: Ṡ(6:11,0)·q̇ = ∂(X21*S1 - S2)/∂q·q̇·q̇
    //                                        = ∂X21/∂q_rel·∂q_rel/∂q·q̇·S1·q̇
    SX qd1 = SX::sym("qd1");
    SX qd2 = SX::sym("qd2");

    // ∂X21/∂q_rel = ∂(XJ2*X_internal)/∂q_rel
    // For a rotation XJ2 = Rot(q_rel), we have ∂XJ2/∂q_rel = crm(S2)*XJ2
    // So ∂X21/∂q_rel = crm(S2)*X21
    SX S2_crm = spatialCrossMatrix(S2_sym);
    SX dX21_dq_rel = mtimes(S2_crm, X21_sym);

    // Ṡ(6:11,0)·q̇(0) = dX21_dq_rel * S1 * ∂q_rel/∂q * q̇
    // where ∂q_rel/∂q1 = -1, ∂q_rel/∂q2 = +1
    SX Sdot_qd_term = mtimes(dX21_dq_rel, S1_sym) * (qd2 - qd1);

    // ∂(Ṡ·q̇)/∂q̇ for the body 2 component
    SX dSdotqd_dqd1 = jacobian(Sdot_qd_term, qd1);
    SX dSdotqd_dqd2 = jacobian(Sdot_qd_term, qd2);

    f_dS_dq1_ = Function("dS_dq1", {q1, q2}, {dS_dq1});
    f_dS_dq2_ = Function("dS_dq2", {q1, q2}, {dS_dq2});
    f_Sdotqd_qd_ = Function("Sdotqd_qd", {q1, q2, qd1, qd2}, {horzcat(dSdotqd_dqd1, dSdotqd_dqd2)});
    
    casadi_functions_initialized_ = true;
}

template <typename Scalar>
std::vector<DMat<Scalar>> RevolutePairAbsolute<Scalar>::getSq() const
{
    initializeCasadiFunctions();
    
    const int nv = 2;
    const int spatial_dim = 12;
    
    std::vector<DMat<Scalar>> S_q(nv);
    
    std::vector<casadi::DM> input = {
        casadi::DM(static_cast<double>(q_cache_(0))),
        casadi::DM(static_cast<double>(q_cache_(1)))
    };
    
    std::vector<casadi::DM> res_dq1 = f_dS_dq1_(input);
    std::vector<casadi::DM> res_dq2 = f_dS_dq2_(input);
    
    S_q[0] = DMat<Scalar>::Zero(spatial_dim, nv);
    for (int i = 0; i < 6; ++i) {
        S_q[0](6 + i, 0) = static_cast<Scalar>(static_cast<double>(res_dq1[0](i)));
    }
    
    S_q[1] = DMat<Scalar>::Zero(spatial_dim, nv);
    for (int i = 0; i < 6; ++i) {
        S_q[1](6 + i, 0) = static_cast<Scalar>(static_cast<double>(res_dq2[0](i)));
    }
    
    return S_q;
}

template <typename Scalar>
DMat<Scalar> RevolutePairAbsolute<Scalar>::getSdotqd_qd() const
{
    initializeCasadiFunctions();
    
    const int nv = 2;
    const int spatial_dim = 12;
    
    std::vector<casadi::DM> input = {
        casadi::DM(static_cast<double>(q_cache_(0))),
        casadi::DM(static_cast<double>(q_cache_(1))),
        casadi::DM(static_cast<double>(qd_cache_(0))),
        casadi::DM(static_cast<double>(qd_cache_(1)))
    };
    
    std::vector<casadi::DM> res = f_Sdotqd_qd_(input);
    
    DMat<Scalar> result = DMat<Scalar>::Zero(spatial_dim, nv);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < nv; ++j) {
            result(6 + i, j) = static_cast<Scalar>(static_cast<double>(res[0](i, j)));
        }
    }
    
    return result;
}

template class RevolutePairAbsolute<double>;

// For complex-step, we only need to instantiate the methods that are actually used.
// The derivative methods (getSq, getSdotqd_qd) return zero since complex-step
// doesn't use analytical derivatives.

// Explicitly instantiate only the needed methods for complex<double>
template RevolutePairAbsolute<std::complex<double>>::RevolutePairAbsolute(
    ori::CoordinateAxis, ori::CoordinateAxis,
    const spatial::Transform<std::complex<double>>&);

template void RevolutePairAbsolute<std::complex<double>>::updateKinematics(
    const JointState<std::complex<double>>&);

template void RevolutePairAbsolute<std::complex<double>>::computeSpatialTransformFromParentToCurrentCluster(
    spatial::GeneralizedTransform<std::complex<double>>&) const;

template <>
std::vector<DMat<std::complex<double>>>
RevolutePairAbsolute<std::complex<double>>::getSq() const
{
    const int nv = 2;
    const int spatial_dim = 12;
    return std::vector<DMat<std::complex<double>>>(
        nv, DMat<std::complex<double>>::Zero(spatial_dim, nv));
}

template <>
DMat<std::complex<double>>
RevolutePairAbsolute<std::complex<double>>::getSdotqd_qd() const
{
    const int nv = 2;
    const int spatial_dim = 12;
    return DMat<std::complex<double>>::Zero(spatial_dim, nv);
}

} // namespace ClusterJoints
} // namespace grbda
