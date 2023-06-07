#include "SpatialTransforms.h"

namespace spatial {

////////////////////////////////////////////////////////////////////////////////////////////////////
// SpatialTransform
////////////////////////////////////////////////////////////////////////////////////////////////////

SpatialTransform::SpatialTransform(const Mat3<double> &E, const Vec3<double> &r) : E_(E), r_(r) {}

void SpatialTransform::setIdentity()
{
    E_.setIdentity();
    r_.setZero();
}

Mat6<double> SpatialTransform::toMatrix() const {
    Mat6<double> X = Mat6<double>::Zero();
    X.template topLeftCorner<3, 3>() = E_;
    X.template bottomRightCorner<3, 3>() = E_;
    X.template bottomLeftCorner<3, 3>() = -E_ * vectorToSkewMat(r_);
    return X;
 }

SVec<double> SpatialTransform::transformMotionVector(const SVec<double> &m_in) const
{
    SVec<double> m_out;
    m_out.head<3>() = E_ * m_in.head<3>();
    m_out.tail<3>() = -E_ * vectorToSkewMat(r_) * m_in.head<3>() + E_ * m_in.tail<3>();
    return m_out;
}

SVec<double> SpatialTransform::inverseTransformMotionVector(const SVec<double> &m_in) const
{
    SVec<double> m_out;
    m_out.head<3>() = E_.transpose() * m_in.head<3>();
    m_out.tail<3>() = vectorToSkewMat(r_) * E_.transpose() * m_in.head<3>() +
                      E_.transpose() * m_in.tail<3>();
    return m_out;
}

SVec<double> SpatialTransform::transformForceVector(const SVec<double> &f_in) const
{
    SVec<double> f_out;
    f_out.head<3>() = E_ * f_in.head<3>() - E_ * vectorToSkewMat(r_) * f_in.tail<3>();
    f_out.tail<3>() = E_ * f_in.tail<3>();
    return f_out;
}

SVec<double> SpatialTransform::inverseTransformForceVector(const SVec<double> &f_in) const
{
    SVec<double> f_out;
    f_out.head<3>() = E_.transpose() * f_in.head<3>() +
                      vectorToSkewMat(r_) * E_.transpose() * f_in.tail<3>();
    f_out.tail<3>() = E_.transpose() * f_in.tail<3>();
    return f_out;
}

D6Mat<double> SpatialTransform::transformMotionSubspace(const D6Mat<double> &S_in) const
{
    D6Mat<double> S_out = D6Mat<double>::Zero(6, S_in.cols());
    for (int i = 0; i < S_in.cols(); i++)
        S_out.col(i) = transformMotionVector(S_in.col(i));
    return S_out;
}

D6Mat<double> SpatialTransform::inverseTransformMotionSubspace(const D6Mat<double> &S_in) const
{
    D6Mat<double> S_out = D6Mat<double>::Zero(6, S_in.cols());
    for (int i = 0; i < S_in.cols(); i++)
        S_out.col(i) = inverseTransformMotionVector(S_in.col(i));
    return S_out;
}

D6Mat<double> SpatialTransform::inverseTransformForceSubspace(const D6Mat<double> &F_in) const
{
    D6Mat<double> F_out = D6Mat<double>::Zero(6, F_in.cols());
    for (int i = 0; i < F_in.cols(); i++)
        F_out.col(i) = inverseTransformForceVector(F_in.col(i));
    return F_out;
}

Mat6<double> SpatialTransform::inverseTransformSpatialInertia(const Mat6<double> &I_in) const
{
    Mat6<double> I_out;
    Mat3<double> E_trans = E_.transpose();
    Mat3<double> r_hat = vectorToSkewMat(r_);

    I_out.topLeftCorner<3, 3>() = E_trans * I_in.topLeftCorner<3, 3>() * E_ +
                                  r_hat * E_trans * I_in.bottomLeftCorner<3, 3>() * E_ -
                                  E_trans * I_in.topRightCorner<3, 3>() * E_ * r_hat -
                                  r_hat * E_trans * I_in.bottomRightCorner<3, 3>() * E_ * r_hat;
    I_out.topRightCorner<3, 3>() = E_trans * I_in.topRightCorner<3, 3>() * E_ +
                                   r_hat * E_trans * I_in.bottomRightCorner<3, 3>() * E_;
    I_out.bottomLeftCorner<3, 3>() = E_trans * I_in.bottomLeftCorner<3, 3>() * E_ -
                                     E_trans * I_in.bottomRightCorner<3, 3>() * E_ * r_hat;
    I_out.bottomRightCorner<3, 3>() = E_trans * I_in.bottomRightCorner<3, 3>() * E_;

    return I_out;
}

Vec3<double> SpatialTransform::transformPoint(const Vec3<double> &local_offset) const
{
    return E_ * (local_offset - r_);
}

Vec3<double> SpatialTransform::inverseTransformPoint(const Vec3<double> &local_offset) const
{
    return E_.transpose() * local_offset + r_;
}

SpatialTransform SpatialTransform::operator*(const SpatialTransform &X_in) const
{
    Mat3<double> E_out = E_ * X_in.getRotation();
    Mat3<double> r_out = E_out.transpose() * E_ * vectorToSkewMat(r_) * X_in.getRotation() +
                         X_in.getSkewTranslationMatrix();
    return SpatialTransform(E_out, matToSkewVec(r_out));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// GeneralizedAbsoluteSpatialTransform
////////////////////////////////////////////////////////////////////////////////////////////////////

void GeneralizedAbsoluteSpatialTransform::appendSpatialTransform(const SpatialTransform &X)
{
    transforms_.push_back(X);
    num_output_bodies_++;
}

const SpatialTransform &
GeneralizedAbsoluteSpatialTransform::getTransformForOutputBody(int output_body_index) const
{
    return transforms_[output_body_index];
}

DMat<double> GeneralizedAbsoluteSpatialTransform::toMatrix() const
{
    DMat<double> X_mat = DMat<double>::Zero(6 * num_output_bodies_, 6);
    int output_body = 0;
    for (const auto &transform : transforms_)
    {
        X_mat.block<6, 6>(6 * output_body, 0) = transform.toMatrix();
        output_body++;
    }
    return X_mat;
}

DVec<double>
GeneralizedAbsoluteSpatialTransform::transformExternalForceVector(const DVec<double> &f_in) const
{
    if (f_in.rows() != 6 * num_output_bodies_)
        throw std::runtime_error("Invalid dimension for external force vector being transformed");

    DVec<double> f_out = DVec<double>::Zero(6 * num_output_bodies_);
    int output_body = 0;
    for (const auto &transform : transforms_)
    {
        f_out.segment<6>(6 * output_body) =
            transform.transformForceVector(f_in.segment<6>(6 * output_body));
        output_body++;
    }
    return f_out;
}

SpatialTransform &GeneralizedAbsoluteSpatialTransform::operator[](int output_body_index)
{
    return transforms_[output_body_index];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// GeneralizedSpatialTransform
////////////////////////////////////////////////////////////////////////////////////////////////////

GeneralizedSpatialTransform::GeneralizedSpatialTransform(int num_parent_bodies)
    : num_parent_bodies_(num_parent_bodies) {}

void GeneralizedSpatialTransform::appendSpatialTransformWithClusterAncestorSubIndex(
    const SpatialTransform &X, const int subindex)
{
    if (subindex >= num_parent_bodies_)
        throw std::runtime_error("Parent subindex greater than the number of parent bodies");

    transforms_and_parent_subindices_.push_back(std::pair(X, subindex));
    num_output_bodies_++;
}

DMat<double> GeneralizedSpatialTransform::toMatrix() const
{
    DMat<double> X_mat = DMat<double>::Zero(6 * num_output_bodies_, 6 * num_parent_bodies_);
    int output_body = 0;
    for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
    {
        const SpatialTransform &X = transform_and_parent_subindex.first;
        const int parent_subindex = transform_and_parent_subindex.second;
        X_mat.block<6, 6>(6 * output_body, 6 * parent_subindex) = X.toMatrix();
        output_body++;
    }
    return X_mat;
}

GeneralizedAbsoluteSpatialTransform GeneralizedSpatialTransform::toAbsolute() const
{
    GeneralizedAbsoluteSpatialTransform Xa_out{};
    for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
        Xa_out.appendSpatialTransform(transform_and_parent_subindex.first);
    return Xa_out;
}

const std::pair<SpatialTransform, int> &
GeneralizedSpatialTransform::transform_and_parent_subindex(int output_body_index) const
{
    return transforms_and_parent_subindices_[output_body_index];
}

DVec<double> GeneralizedSpatialTransform::transformMotionVector(const DVec<double> &m_in) const
{
    DVec<double> m_out = DVec<double>::Zero(6 * num_output_bodies_);
    int output_body = 0;
    for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
    {
        const SpatialTransform &X = transform_and_parent_subindex.first;
        const int parent_subindex = transform_and_parent_subindex.second;
        m_out.segment<6>(6 * output_body) =
            X.transformMotionVector(m_in.segment<6>(6 * parent_subindex));
        output_body++;
    }
    return m_out;
}

DVec<double>
GeneralizedSpatialTransform::inverseTransformForceVector(const DVec<double> &f_in) const
{
    DVec<double> f_out = DVec<double>::Zero(6 * num_parent_bodies_);
    int output_body = 0;
    for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
    {
        const SpatialTransform &X = transform_and_parent_subindex.first;
        const int parent_subindex = transform_and_parent_subindex.second;
        f_out.segment<6>(6 * parent_subindex) +=
            X.inverseTransformForceVector(f_in.segment<6>(6 * output_body));
        output_body++;
    }
    return f_out;
}

DMat<double>
GeneralizedSpatialTransform::inverseTransformForceSubspace(const DMat<double> &F_in) const
{
    const int num_cols = F_in.cols();
    DMat<double> F_out = DMat<double>::Zero(6 * num_parent_bodies_, num_cols);
    int output_body = 0;
    for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
    {
        const SpatialTransform &X = transform_and_parent_subindex.first;
        const int parent_subindex = transform_and_parent_subindex.second;
        F_out.block(6 * parent_subindex, 0, 6, num_cols) +=
            X.inverseTransformForceSubspace(F_in.block(6 * output_body, 0, 6, num_cols));
        output_body++;
    }
    return F_out;
}

DMat<double> GeneralizedSpatialTransform::inverseTransformSpatialInertia(const DMat<double> &I_in) const
{
    return toMatrix().transpose() * I_in * toMatrix();
}

SpatialTransform &GeneralizedSpatialTransform::operator[](int output_body_index)
{
    return transforms_and_parent_subindices_[output_body_index].first;
}

GeneralizedSpatialTransform
GeneralizedSpatialTransform::operator*(const GeneralizedSpatialTransform &X_in) const
{
    GeneralizedSpatialTransform X_out = GeneralizedSpatialTransform(X_in.getNumParentBodies());

    for (const auto &transform_and_parent_subindex1 : transforms_and_parent_subindices_)
    {
        const SpatialTransform &X1 = transform_and_parent_subindex1.first;
        const int parent_subindex1 = transform_and_parent_subindex1.second;

        const auto &transform_and_parent_subindex2 =
            X_in.transform_and_parent_subindex(parent_subindex1);
        const SpatialTransform &X2 = transform_and_parent_subindex2.first;
        const int parent_subindex2 = transform_and_parent_subindex2.second;

        X_out.appendSpatialTransformWithClusterAncestorSubIndex(X1 * X2, parent_subindex2);
    }

    return X_out;
}

GeneralizedAbsoluteSpatialTransform
GeneralizedSpatialTransform::operator*(const GeneralizedAbsoluteSpatialTransform &X_in) const
{
    GeneralizedAbsoluteSpatialTransform Xa_out = GeneralizedAbsoluteSpatialTransform();

    for (const auto &transform_and_parent_subindex1 : transforms_and_parent_subindices_)
    {
        const SpatialTransform &X1 = transform_and_parent_subindex1.first;
        const int parent_subindex1 = transform_and_parent_subindex1.second;
        Xa_out.appendSpatialTransform(X1 * X_in.getTransformForOutputBody(parent_subindex1));
    }

    return Xa_out;
}

} // namespace spatial
