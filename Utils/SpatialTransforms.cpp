#include "SpatialTransforms.h"

namespace grbda
{

    namespace spatial
    {

        ////////////////////////////////////////////////////////////////////////////////////////////
        // SpatialTransform
        ////////////////////////////////////////////////////////////////////////////////////////////

        template <typename Scalar>
        Transform<Scalar>::Transform(const Mat3<Scalar> &E, const Vec3<Scalar> &r) : E_(E), r_(r) {}

        template <typename Scalar>
        void Transform<Scalar>::setIdentity()
        {
            E_.setIdentity();
            r_.setZero();
        }

        template <typename Scalar>
        Mat6<Scalar> Transform<Scalar>::toMatrix() const
        {
            Mat6<Scalar> X = Mat6<Scalar>::Zero();
            X.template topLeftCorner<3, 3>() = E_;
            X.template bottomRightCorner<3, 3>() = E_;
            X.template bottomLeftCorner<3, 3>() = -E_ * ori::vectorToSkewMat(r_);
            return X;
        }

        template <typename Scalar>
        SVec<Scalar> Transform<Scalar>::transformMotionVector(const SVec<Scalar> &m_in) const
        {
            SVec<Scalar> m_out;
            m_out.template head<3>() = E_ * m_in.template head<3>();
            m_out.template tail<3>() = -E_ * ori::vectorToSkewMat(r_) * m_in.template head<3>() +
                                       E_ * m_in.template tail<3>();
            return m_out;
        }

        template <typename Scalar>
        SVec<Scalar> Transform<Scalar>::inverseTransformMotionVector(const SVec<Scalar> &m_in) const
        {
            SVec<Scalar> m_out;
            const Mat3<Scalar> ET = E_.transpose();
            m_out.template head<3>() = ET * m_in.template head<3>();
            m_out.template tail<3>() = ori::vectorToSkewMat(r_) * ET * m_in.template head<3>() +
                                       ET * m_in.template tail<3>();
            return m_out;
        }

        template <typename Scalar>
        SVec<Scalar> Transform<Scalar>::transformForceVector(const SVec<Scalar> &f_in) const
        {
            SVec<Scalar> f_out;
            f_out.template head<3>() = E_ * f_in.template head<3>() -
                                       E_ * ori::vectorToSkewMat(r_) * f_in.template tail<3>();
            f_out.template tail<3>() = E_ * f_in.template tail<3>();
            return f_out;
        }

        template <typename Scalar>
        SVec<Scalar> Transform<Scalar>::inverseTransformForceVector(const SVec<Scalar> &f_in) const
        {
            SVec<Scalar> f_out;
            const Mat3<Scalar> ET = E_.transpose();
            f_out.template head<3>() = ET * f_in.template head<3>() +
                                       ori::vectorToSkewMat(r_) * ET * f_in.template tail<3>();
            f_out.template tail<3>() = ET * f_in.template tail<3>();
            return f_out;
        }

        template <typename Scalar>
        D6Mat<Scalar> Transform<Scalar>::transformMotionSubspace(const D6Mat<Scalar> &S_in) const
        {
            D6Mat<Scalar> S_out = D6Mat<Scalar>::Zero(6, S_in.cols());
            for (int i = 0; i < S_in.cols(); i++)
                S_out.col(i) = transformMotionVector(S_in.col(i));
            return S_out;
        }

        template <typename Scalar>
        D6Mat<Scalar> Transform<Scalar>::inverseTransformMotionSubspace(const D6Mat<Scalar> &S_in) const
        {
            D6Mat<Scalar> S_out = D6Mat<Scalar>::Zero(6, S_in.cols());
            for (int i = 0; i < S_in.cols(); i++)
                S_out.col(i) = inverseTransformMotionVector(S_in.col(i));
            return S_out;
        }

        template <typename Scalar>
        D6Mat<Scalar> Transform<Scalar>::inverseTransformForceSubspace(const D6Mat<Scalar> &F_in) const
        {
            D6Mat<Scalar> F_out = D6Mat<Scalar>::Zero(6, F_in.cols());
            for (int i = 0; i < F_in.cols(); i++)
                F_out.col(i) = inverseTransformForceVector(F_in.col(i));
            return F_out;
        }

        template <typename Scalar>
        Mat6<Scalar>
        Transform<Scalar>::inverseTransformSpatialInertia(const Mat6<Scalar> &I_in) const
        {
            Mat6<Scalar> I_out;
            Mat3<Scalar> E_trans = E_.transpose();
            Mat3<Scalar> r_hat = ori::vectorToSkewMat(r_);

            const Mat3<Scalar> &I_TL = I_in.template topLeftCorner<3, 3>();
            const Mat3<Scalar> &I_TR = I_in.template topRightCorner<3, 3>();
            const Mat3<Scalar> &I_BL = I_in.template bottomLeftCorner<3, 3>();
            const Mat3<Scalar> &I_BR = I_in.template bottomRightCorner<3, 3>();

            I_out.template topLeftCorner<3, 3>() = E_trans * I_TL * E_ +
                                                   r_hat * E_trans * I_BL * E_ -
                                                   E_trans * I_TR * E_ * r_hat -
                                                   r_hat * E_trans * I_BR * E_ * r_hat;
            I_out.template topRightCorner<3, 3>() = E_trans * I_TR * E_ +
                                                    r_hat * E_trans * I_BR * E_;
            I_out.template bottomLeftCorner<3, 3>() = E_trans * I_BL * E_ -
                                                      E_trans * I_BR * E_ * r_hat;
            I_out.template bottomRightCorner<3, 3>() = E_trans * I_BR * E_;

            return I_out;
        }

        template <typename Scalar>
        Vec3<Scalar> Transform<Scalar>::transformPoint(const Vec3<Scalar> &local_offset) const
        {
            return E_ * (local_offset - r_);
        }

        template <typename Scalar>
        Vec3<Scalar> Transform<Scalar>::inverseTransformPoint(const Vec3<Scalar> &local_offset) const
        {
            return E_.transpose() * local_offset + r_;
        }

        template <typename Scalar>
        Transform<Scalar> Transform<Scalar>::operator*(const Transform<Scalar> &X_in) const
        {
            const Mat3<Scalar>& R_in = X_in.getRotation();
            Mat3<Scalar> E_out = E_ * R_in;
            Mat3<Scalar> r_out = E_out.transpose() * E_ * ori::vectorToSkewMat(r_) * R_in +
                                 X_in.getSkewTranslationMatrix();
            return Transform(E_out, ori::matToSkewVec(r_out));
        }

        template <typename Scalar>
        Mat6<Scalar> Transform<Scalar>::rightMultiplyMotionTransform(const Mat6<Scalar> &M_in) const
        {
            Mat6<Scalar> M_out;

            const Mat3<Scalar> &M_TL = M_in.template topLeftCorner<3, 3>();
            const Mat3<Scalar> &M_TR = M_in.template topRightCorner<3, 3>();
            const Mat3<Scalar> &M_BL = M_in.template bottomLeftCorner<3, 3>();
            const Mat3<Scalar> &M_BR = M_in.template bottomRightCorner<3, 3>();

            const Mat3<Scalar> r_hat = ori::vectorToSkewMat(r_);

            M_out.template topLeftCorner<3, 3>() = M_TL * E_ - M_TR * E_ * r_hat;
            M_out.template topRightCorner<3, 3>() = M_TR * E_;
            M_out.template bottomLeftCorner<3, 3>() = M_BL * E_ - M_BR * E_ * r_hat;
            M_out.template bottomRightCorner<3, 3>() = M_BR * E_;

            return M_out;
        }

        template <typename Scalar>
        Mat6<Scalar> Transform<Scalar>::leftMultiplyForceTransform(const Mat6<Scalar> &M_in) const
        {
            Mat6<Scalar> M_out;
            const Mat3<Scalar> &M_TL = M_in.template topLeftCorner<3, 3>();
            const Mat3<Scalar> &M_TR = M_in.template topRightCorner<3, 3>();
            const Mat3<Scalar> &M_BL = M_in.template bottomLeftCorner<3, 3>();
            const Mat3<Scalar> &M_BR = M_in.template bottomRightCorner<3, 3>();

            const Mat3<Scalar> E_trans = E_.transpose();
            const Mat3<Scalar> r_hat = ori::vectorToSkewMat(r_);

            M_out.template topLeftCorner<3, 3>() = E_trans * M_TL + r_hat * E_trans * M_BL;
            M_out.template topRightCorner<3, 3>() = E_trans * M_TR + r_hat * E_trans * M_BR;
            M_out.template bottomLeftCorner<3, 3>() = E_trans * M_BL;
            M_out.template bottomRightCorner<3, 3>() = E_trans * M_BR;

            return M_out;
        }

        template class Transform<double>;
        template class Transform<float>;
        template class Transform<casadi::SX>;

        ////////////////////////////////////////////////////////////////////////////////////////////
        // GeneralizedAbsoluteTransform
        ////////////////////////////////////////////////////////////////////////////////////////////

        template <typename Scalar>
        void GeneralizedAbsoluteTransform<Scalar>::appendTransform(const Transform<Scalar> &X)
        {
            transforms_.push_back(X);
            num_output_bodies_++;
        }

        template <typename Scalar>
        const Transform<Scalar> &
        GeneralizedAbsoluteTransform<Scalar>::getTransformForOutputBody(int output_body_index) const
        {
            return transforms_[output_body_index];
        }

        template <typename Scalar>
        DMat<Scalar> GeneralizedAbsoluteTransform<Scalar>::toMatrix() const
        {
            DMat<Scalar> X_mat = DMat<Scalar>::Zero(6 * num_output_bodies_, 6);
            int output_body = 0;
            for (const auto &transform : transforms_)
            {
                X_mat.template block<6, 6>(6 * output_body, 0) = transform.toMatrix();
                output_body++;
            }
            return X_mat;
        }

        template <typename Scalar>
        DVec<Scalar> GeneralizedAbsoluteTransform<Scalar>::transformExternalForceVector(
            const DVec<Scalar> &f_in) const
        {
            if (f_in.rows() != 6 * num_output_bodies_)
                throw std::runtime_error("Invalid dimension for external force vector being transformed");

            DVec<Scalar> f_out = DVec<Scalar>::Zero(6 * num_output_bodies_);
            int output_body = 0;
            for (const auto &transform : transforms_)
            {
                f_out.template segment<6>(6 * output_body) =
                    transform.transformForceVector(f_in.template segment<6>(6 * output_body));
                output_body++;
            }
            return f_out;
        }

        template <typename Scalar>
        Transform<Scalar> &GeneralizedAbsoluteTransform<Scalar>::operator[](int output_body_index)
        {
            return transforms_[output_body_index];
        }

        template class GeneralizedAbsoluteTransform<double>;
        template class GeneralizedAbsoluteTransform<float>;
        template class GeneralizedAbsoluteTransform<casadi::SX>;

        ///////////////////////////////////////////////////////////////////////////////////////////
        // GeneralizedTransform
        ///////////////////////////////////////////////////////////////////////////////////////////

        template <typename Scalar>
        GeneralizedTransform<Scalar>::GeneralizedTransform(int num_parent_bodies)
            : num_parent_bodies_(num_parent_bodies) {}

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::appendTransformWithClusterAncestorSubIndex(
            const Transform<Scalar> &X, const int subindex)
        {
            if (subindex >= num_parent_bodies_)
                throw std::runtime_error("Parent subindex greater than the number of parent bodies");

            transforms_and_parent_subindices_.push_back(std::pair(X, subindex));
            num_output_bodies_++;
        }

        template <typename Scalar>
        DMat<Scalar> GeneralizedTransform<Scalar>::toMatrix() const
        {
            DMat<Scalar> X_mat = DMat<Scalar>::Zero(6 * num_output_bodies_, 6 * num_parent_bodies_);
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int parent_subindex = transform_and_parent_subindex.second;
                X_mat.template block<6, 6>(6 * output_body, 6 * parent_subindex) = X.toMatrix();
                output_body++;
            }
            return X_mat;
        }

        template <typename Scalar>
        GeneralizedAbsoluteTransform<Scalar> GeneralizedTransform<Scalar>::toAbsolute() const
        {
            GeneralizedAbsoluteTransform<Scalar> Xa_out{};
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
                Xa_out.appendTransform(transform_and_parent_subindex.first);
            return Xa_out;
        }

        template <typename Scalar>
        const std::pair<Transform<Scalar>, int> &
        GeneralizedTransform<Scalar>::transform_and_parent_subindex(int output_body_index) const
        {
            return transforms_and_parent_subindices_[output_body_index];
        }

        template <typename Scalar>
        DVec<Scalar>
        GeneralizedTransform<Scalar>::transformMotionVector(const DVec<Scalar> &m_in) const
        {
            DVec<Scalar> m_out = DVec<Scalar>::Zero(6 * num_output_bodies_);
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int parent_subindex = transform_and_parent_subindex.second;
                m_out.template segment<6>(6 * output_body) =
                    X.transformMotionVector(m_in.template segment<6>(6 * parent_subindex));
                output_body++;
            }
            return m_out;
        }

        template <typename Scalar>
        DVec<Scalar>
        GeneralizedTransform<Scalar>::inverseTransformForceVector(const DVec<Scalar> &f_in) const
        {
            DVec<Scalar> f_out = DVec<Scalar>::Zero(6 * num_parent_bodies_);
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int parent_subindex = transform_and_parent_subindex.second;
                f_out.template segment<6>(6 * parent_subindex) +=
                    X.inverseTransformForceVector(f_in.template segment<6>(6 * output_body));
                output_body++;
            }
            return f_out;
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::inverseTransformForceSubspace(const DMat<Scalar> &F_in) const
        {
            const int num_cols = F_in.cols();
            DMat<Scalar> F_out = DMat<Scalar>::Zero(6 * num_parent_bodies_, num_cols);
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int parent_subindex = transform_and_parent_subindex.second;
                F_out.template block(6 * parent_subindex, 0, 6, num_cols) +=
                    X.inverseTransformForceSubspace(F_in.template block(6 * output_body, 0, 6, num_cols));
                output_body++;
            }
            return F_out;
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::inverseTransformSpatialInertia(const DMat<Scalar> &I_in) const
        {
            return leftMultiplyForceTransform(rightMultiplyMotionTransform(I_in));
        }

        template <typename Scalar>
        Transform<Scalar> &GeneralizedTransform<Scalar>::operator[](int output_body_index)
        {
            return transforms_and_parent_subindices_[output_body_index].first;
        }

        template <typename Scalar>
        GeneralizedTransform<Scalar>
        GeneralizedTransform<Scalar>::operator*(const GeneralizedTransform &X_in) const
        {
            GeneralizedTransform X_out = GeneralizedTransform(X_in.getNumParentBodies());

            for (const auto &transform_and_parent_subindex1 : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X1 = transform_and_parent_subindex1.first;
                const int parent_subindex1 = transform_and_parent_subindex1.second;

                const auto &transform_and_parent_subindex2 =
                    X_in.transform_and_parent_subindex(parent_subindex1);
                const Transform<Scalar> &X2 = transform_and_parent_subindex2.first;
                const int parent_subindex2 = transform_and_parent_subindex2.second;

                X_out.appendTransformWithClusterAncestorSubIndex(X1 * X2, parent_subindex2);
            }

            return X_out;
        }

        template <typename Scalar>
        GeneralizedAbsoluteTransform<Scalar>
        GeneralizedTransform<Scalar>::operator*(const GeneralizedAbsoluteTransform<Scalar> &X_in) const
        {
            GeneralizedAbsoluteTransform<Scalar> Xa_out = GeneralizedAbsoluteTransform<Scalar>();

            for (const auto &transform_and_parent_subindex1 : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X1 = transform_and_parent_subindex1.first;
                const int parent_subindex1 = transform_and_parent_subindex1.second;
                Xa_out.appendTransform(X1 * X_in.getTransformForOutputBody(parent_subindex1));
            }

            return Xa_out;
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::rightMultiplyMotionTransform(const DMat<Scalar> &M_in) const
        {
#ifdef DEBU_MODE
            if (M_in.rows() != 6 * num_output_bodies_ || M_in.cols() != 6 * num_output_bodies_)
            {
                throw std::runtime_error("ERROR: M_in must be 6num_output_bodies_ * 6num_output_bodies_");
            }
#endif

            DMat<Scalar> M_out = DMat<Scalar>::Zero(6 * num_output_bodies_,
                                                    6 * num_parent_bodies_);

            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int &parent_subindex = transform_and_parent_subindex.second;

                for (int i = 0; i < M_in.rows(); i += 6)
                {
                    M_out.template block<6, 6>(i, 6 * parent_subindex) +=
                        X.rightMultiplyMotionTransform(M_in.template block<6, 6>(i, 6 * output_body));
                }

                output_body++;
            }

            return M_out;
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::leftMultiplyForceTransform(const DMat<Scalar> &M_in) const
        {
#ifdef DEBU_MODE
            if (M_in.rows() != 6 * num_output_bodies_ || M_in.cols() != 6 * num_parent_bodies_)
            {
                throw std::runtime_error("ERROR: M_in must be 6num_output_bodies_ * 6num_parent_bodies_");
            }
#endif

            DMat<Scalar> M_out = DMat<Scalar>::Zero(6 * num_parent_bodies_,
                                                    6 * num_parent_bodies_);

            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int &parent_subindex = transform_and_parent_subindex.second;

                for (int i = 0; i < M_in.cols(); i += 6)
                {
                    M_out.template block<6, 6>(6 * parent_subindex, i) +=
                        X.leftMultiplyForceTransform(M_in.template block<6, 6>(6 * output_body, i));
                }

                output_body++;
            }

            return M_out;
        }

        template class GeneralizedTransform<double>;
        template class GeneralizedTransform<float>;
        template class GeneralizedTransform<casadi::SX>;

    } // namespace spatial

} // namespace grbda
