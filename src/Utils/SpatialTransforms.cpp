#include "grbda/Utils/SpatialTransforms.h"
#include "grbda/Utils/Spatial.h"

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
        Transform<Scalar>::Transform(const urdf::Pose &pose)
        {
            urdf::Rotation rotation = pose.rotation;
            Quat<Scalar> quat = Quat<Scalar>(rotation.w, rotation.x, rotation.y, rotation.z);
            E_ = ori::quaternionToRotationMatrix(quat);
            r_ = Vec3<Scalar>(pose.position.x, pose.position.y, pose.position.z);
        }

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
        void Transform<Scalar>::transformMotionSubspace(const D6Mat<Scalar> &S_in,
                                                        D6Mat<Scalar> &S_out) const
        {
            S_out.resize(6, S_in.cols());
            for (int i = 0; i < S_in.cols(); i++)
                S_out.col(i) = transformMotionVector(S_in.col(i));
        }

        template <typename Scalar>
        D6Mat<Scalar> Transform<Scalar>::transformMotionSubspace(const D6Mat<Scalar> &S_in) const
        {
            D6Mat<Scalar> S_out;
            transformMotionSubspace(S_in, S_out);
            return S_out;
        }

        template <typename Scalar>
        void Transform<Scalar>::inverseTransformMotionSubspace(const D6Mat<Scalar> &S_in,
                                                               D6Mat<Scalar> &S_out) const
        {
            S_out.resize(6, S_in.cols());
            for (int i = 0; i < S_in.cols(); i++)
                S_out.col(i) = inverseTransformMotionVector(S_in.col(i));
        }

        template <typename Scalar>
        D6Mat<Scalar> Transform<Scalar>::inverseTransformMotionSubspace(const D6Mat<Scalar> &S_in) const
        {
            D6Mat<Scalar> S_out;
            inverseTransformMotionSubspace(S_in, S_out);
            return S_out;
        }

        template <typename Scalar>
        void Transform<Scalar>::inverseTransformForceSubspace(const D6Mat<Scalar> &F_in,
                                                              D6Mat<Scalar> &F_out) const
        {
            // X^{-T} * F where X^{-T} = [E^T, r_hat * E^T; 0, E^T]
            const Mat3<Scalar> ET = E_.transpose();
            const Mat3<Scalar> r_hat_ET = ori::vectorToSkewMat(r_) * ET;

            F_out.resize(6, F_in.cols());
            F_out.template topRows<3>().noalias() = ET * F_in.template topRows<3>();
            F_out.template topRows<3>().noalias() += r_hat_ET * F_in.template bottomRows<3>();
            F_out.template bottomRows<3>().noalias() = ET * F_in.template bottomRows<3>();
        }

        template <typename Scalar>
        D6Mat<Scalar> Transform<Scalar>::inverseTransformForceSubspace(const D6Mat<Scalar> &F_in) const
        {
            D6Mat<Scalar> F_out;
            inverseTransformForceSubspace(F_in, F_out);
            return F_out;
        }

        template <typename Scalar>
        void Transform<Scalar>::inverseTransformForceSubspace(
            DMat<Scalar> &F1, DMat<Scalar> &F2, DMat<Scalar> &F3, DMat<Scalar> &F4) const
        {
            // Batched version: transforms 4 force subspaces with shared E^T and r_hat*E^T computation
            // This is used in the ID derivatives walk-to-root loop where t1, t2, t3, t4 are all transformed
            const Mat3<Scalar> ET = E_.transpose();
            const Mat3<Scalar> r_hat_ET = ori::vectorToSkewMat(r_) * ET;

            // Scratch buffer shared across all 4 transforms (F1-F4 are all the same size)
            DMat<Scalar> F_out(6, F1.cols());

            auto transformInPlace = [&ET, &r_hat_ET, &F_out](DMat<Scalar> &F) {
                F_out.template topRows<3>().noalias() = ET * F.template topRows<3>();
                F_out.template topRows<3>().noalias() += r_hat_ET * F.template bottomRows<3>();
                F_out.template bottomRows<3>().noalias() = ET * F.template bottomRows<3>();
                std::swap(F, F_out);
            };

            transformInPlace(F1);
            transformInPlace(F2);
            transformInPlace(F3);
            transformInPlace(F4);
        }

        template <typename Scalar>
        Mat6<Scalar>
        Transform<Scalar>::inverseTransformSpatialInertia(const Mat6<Scalar> &I_in) const
        {
            Mat6<Scalar> I_out;
            const Mat3<Scalar> E_trans = E_.transpose();
            const Mat3<Scalar> r_hat = ori::vectorToSkewMat(r_);

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
        Mat6<Scalar>
        Transform<Scalar>::transformSpatialInertiaToWorld(const Mat6<Scalar> &I_in) const
        {
            // Transform inertia from body frame to world frame
            // If X transforms world -> body (X * v_world = v_body), then
            // I_world = X^{-T} * I_body * X^{-1}
            //
            // X = [E, 0; -E*r_hat, E]  (transforms world -> body)
            // X^{-1} = [E^T, 0; r_hat*E^T, E^T]  (transforms body -> world)
            // X^{-T} = [E, -E*r_hat^T; 0, E]
            //
            // Note: r_hat^T = -r_hat, so X^{-T} = [E, E*r_hat; 0, E]

            Mat6<Scalar> I_out;
            const Mat3<Scalar> E_trans = E_.transpose();
            const Mat3<Scalar> r_hat = ori::vectorToSkewMat(r_);

            const Mat3<Scalar> &I_TL = I_in.template topLeftCorner<3, 3>();
            const Mat3<Scalar> &I_TR = I_in.template topRightCorner<3, 3>();
            const Mat3<Scalar> &I_BL = I_in.template bottomLeftCorner<3, 3>();
            const Mat3<Scalar> &I_BR = I_in.template bottomRightCorner<3, 3>();

            // Compute X^{-T} * I * X^{-1}
            // X^{-T} = [E, E*r_hat; 0, E]
            // X^{-1} = [E^T, 0; r_hat*E^T, E^T]

            // First compute I * X^{-1}:
            // [I_TL, I_TR]   [E^T,       0  ]   [I_TL*E^T + I_TR*r_hat*E^T,  I_TR*E^T]
            // [I_BL, I_BR] * [r_hat*E^T, E^T] = [I_BL*E^T + I_BR*r_hat*E^T,  I_BR*E^T]

            const Mat3<Scalar> r_hat_ET = r_hat * E_trans;
            const Mat3<Scalar> temp_TL = I_TL * E_trans + I_TR * r_hat_ET;
            const Mat3<Scalar> temp_TR = I_TR * E_trans;
            const Mat3<Scalar> temp_BL = I_BL * E_trans + I_BR * r_hat_ET;
            const Mat3<Scalar> temp_BR = I_BR * E_trans;

            // Now compute X^{-T} * (I * X^{-1}):
            // [E,  E*r_hat]   [temp_TL, temp_TR]
            // [0,  E      ] * [temp_BL, temp_BR]

            const Mat3<Scalar> E_r_hat = E_ * r_hat;
            I_out.template topLeftCorner<3, 3>() = E_ * temp_TL + E_r_hat * temp_BL;
            I_out.template topRightCorner<3, 3>() = E_ * temp_TR + E_r_hat * temp_BR;
            I_out.template bottomLeftCorner<3, 3>() = E_ * temp_BL;
            I_out.template bottomRightCorner<3, 3>() = E_ * temp_BR;

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
        template class Transform<std::complex<double>>;
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

        template <typename Scalar>
        DMat<Scalar> GeneralizedAbsoluteTransform<Scalar>::transformMotionSubspaceToWorld(
            const DMat<Scalar> &S_local) const
        {
            // Transform motion subspace from local body frames to world frame
            // Each 6-row block is transformed by the inverse of the corresponding Xa
            const int num_cols = S_local.cols();
            DMat<Scalar> S_world = DMat<Scalar>::Zero(6 * num_output_bodies_, num_cols);

            for (int body = 0; body < num_output_bodies_; body++)
            {
                const Transform<Scalar> &Xa = transforms_[body];
                // S_world = X^{-1} * S_local (body -> world)
                // inverseTransformMotionSubspace does X^{-1} * S
                S_world.template middleRows<6>(6 * body) =
                    Xa.inverseTransformMotionSubspace(S_local.template middleRows<6>(6 * body));
            }

            return S_world;
        }

        template class GeneralizedAbsoluteTransform<double>;
        template class GeneralizedAbsoluteTransform<std::complex<double>>;
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
        void GeneralizedTransform<Scalar>::inverseTransformForceSubspace(const DMat<Scalar> &F_in,
                                                                          DMat<Scalar> &F_out) const
        {
            const int num_cols = F_in.cols();
            F_out.setZero(6 * num_parent_bodies_, num_cols);
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int parent_subindex = transform_and_parent_subindex.second;
                F_out.template block(6 * parent_subindex, 0, 6, num_cols) +=
                    X.inverseTransformForceSubspace(F_in.template block(6 * output_body, 0, 6, num_cols));
                output_body++;
            }
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::inverseTransformForceSubspace(const DMat<Scalar> &F_in) const
        {
            DMat<Scalar> F_out;
            inverseTransformForceSubspace(F_in, F_out);
            return F_out;
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::inverseTransformSpatialInertia(const DMat<Scalar> &I_in,
                                                                           DMat<Scalar> &I_out) const
        {
            DMat<Scalar> tmp;
            rightMultiplyMotionTransform(I_in, tmp);
            leftMultiplyForceTransform(tmp, I_out);
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::inverseTransformSpatialInertia(const DMat<Scalar> &I_in) const
        {
            DMat<Scalar> I_out;
            inverseTransformSpatialInertia(I_in, I_out);
            return I_out;
        }

        template <typename Scalar>
        Transform<Scalar> &GeneralizedTransform<Scalar>::operator[](int output_body_index)
        {
            return transforms_and_parent_subindices_[output_body_index].first;
        }

        template <typename Scalar>
        const Transform<Scalar> &GeneralizedTransform<Scalar>::operator[](int output_body_index) const
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
        void GeneralizedTransform<Scalar>::rightMultiplyMotionTransform(const DMat<Scalar> &M_in,
                                                                         DMat<Scalar> &M_out) const
        {
#ifdef DEBU_MODE
            if (M_in.rows() != 6 * num_output_bodies_ || M_in.cols() != 6 * num_output_bodies_)
                throw std::runtime_error("ERROR: M_in must be 6num_output_bodies_ * 6num_output_bodies_");
#endif
            M_out.setZero(6 * num_output_bodies_, 6 * num_parent_bodies_);
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int &parent_subindex = transform_and_parent_subindex.second;
                for (int i = 0; i < M_in.rows(); i += 6)
                    M_out.template block<6, 6>(i, 6 * parent_subindex) +=
                        X.rightMultiplyMotionTransform(M_in.template block<6, 6>(i, 6 * output_body));
                output_body++;
            }
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::rightMultiplyMotionTransform(const DMat<Scalar> &M_in) const
        {
            DMat<Scalar> M_out;
            rightMultiplyMotionTransform(M_in, M_out);
            return M_out;
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::leftMultiplyForceTransform(const DMat<Scalar> &M_in,
                                                                       DMat<Scalar> &M_out) const
        {
#ifdef DEBU_MODE
            if (M_in.rows() != 6 * num_output_bodies_ || M_in.cols() != 6 * num_parent_bodies_)
                throw std::runtime_error("ERROR: M_in must be 6num_output_bodies_ * 6num_parent_bodies_");
#endif
            M_out.setZero(6 * num_parent_bodies_, 6 * num_parent_bodies_);
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int &parent_subindex = transform_and_parent_subindex.second;
                for (int i = 0; i < M_in.cols(); i += 6)
                    M_out.template block<6, 6>(6 * parent_subindex, i) +=
                        X.leftMultiplyForceTransform(M_in.template block<6, 6>(6 * output_body, i));
                output_body++;
            }
        }

        template <typename Scalar>
        DMat<Scalar>
        GeneralizedTransform<Scalar>::leftMultiplyForceTransform(const DMat<Scalar> &M_in) const
        {
            DMat<Scalar> M_out;
            leftMultiplyForceTransform(M_in, M_out);
            return M_out;
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::accumulateBlockDiagonalInertia(
            const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &I_child,
            std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &I_parent) const
        {
            for (int i = 0; i < num_output_bodies_; i++)
            {
                const Transform<Scalar> &X = transforms_and_parent_subindices_[i].first;
                const int parent_subindex = transforms_and_parent_subindices_[i].second;
                I_parent[parent_subindex] += X.inverseTransformSpatialInertia(I_child[i]);
            }
        }

        template <typename Scalar>
        DMat<Scalar> GeneralizedTransform<Scalar>::blockDiagonalInertiaTimesMotionSubspace(
            const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &Ic,
            const DMat<Scalar> &S) const
        {
            DMat<Scalar> out;
            blockDiagonalInertiaTimesMotionSubspace(Ic, S, out);
            return out;
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::blockDiagonalInertiaTimesMotionSubspace(
            const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &Ic,
            const DMat<Scalar> &S, DMat<Scalar> &out) const
        {
            const int num_cols = S.cols();
            out.resize(6 * num_output_bodies_, num_cols);
            for (int body = 0; body < num_output_bodies_; body++)
                out.template middleRows<6>(6 * body).noalias() =
                    Ic[body] * S.template middleRows<6>(6 * body);
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::accumulateBlockDiagonalPair(
            const DMat<Scalar> &M1_child, DMat<Scalar> &M1_parent,
            const DMat<Scalar> &M2_child, DMat<Scalar> &M2_parent) const
        {
            for (int i = 0; i < num_output_bodies_; i++)
            {
                const Transform<Scalar> &X = transforms_and_parent_subindices_[i].first;
                const int parent_subindex = transforms_and_parent_subindices_[i].second;

                const Mat3<Scalar> &E = X.getRotation();
                const Mat3<Scalar> E_trans = E.transpose();
                const Mat3<Scalar> r_hat = ori::vectorToSkewMat(X.getTranslation());

                auto transformBlock = [&](const Mat6<Scalar> &M_in) -> Mat6<Scalar> {
                    Mat6<Scalar> M_out;
                    const Mat3<Scalar> &TL = M_in.template topLeftCorner<3, 3>();
                    const Mat3<Scalar> &TR = M_in.template topRightCorner<3, 3>();
                    const Mat3<Scalar> &BL = M_in.template bottomLeftCorner<3, 3>();
                    const Mat3<Scalar> &BR = M_in.template bottomRightCorner<3, 3>();
                    M_out.template topLeftCorner<3, 3>() = E_trans * TL * E +
                                                           r_hat * E_trans * BL * E -
                                                           E_trans * TR * E * r_hat -
                                                           r_hat * E_trans * BR * E * r_hat;
                    M_out.template topRightCorner<3, 3>() = E_trans * TR * E +
                                                            r_hat * E_trans * BR * E;
                    M_out.template bottomLeftCorner<3, 3>() = E_trans * BL * E -
                                                              E_trans * BR * E * r_hat;
                    M_out.template bottomRightCorner<3, 3>() = E_trans * BR * E;
                    return M_out;
                };

                const int ps = 6 * parent_subindex;
                M1_parent.template block<6, 6>(ps, ps) +=
                    transformBlock(M1_child.template block<6, 6>(6 * i, 6 * i));
                M2_parent.template block<6, 6>(ps, ps) +=
                    transformBlock(M2_child.template block<6, 6>(6 * i, 6 * i));
            }
        }

        template <typename Scalar>
        DMat<Scalar> GeneralizedTransform<Scalar>::blockDiagonalInertiaTimesMotionSubspace(
            const DMat<Scalar> &Ic_block_diag, const DMat<Scalar> &S) const
        {
            DMat<Scalar> out;
            blockDiagonalInertiaTimesMotionSubspace(Ic_block_diag, S, out);
            return out;
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::blockDiagonalInertiaTimesMotionSubspace(
            const DMat<Scalar> &Ic_block_diag, const DMat<Scalar> &S, DMat<Scalar> &out) const
        {
            const int num_cols = S.cols();
            out.resize(6 * num_output_bodies_, num_cols);
            for (int body = 0; body < num_output_bodies_; body++)
                out.template middleRows<6>(6 * body).noalias() =
                    Ic_block_diag.template block<6, 6>(6 * body, 6 * body) *
                    S.template middleRows<6>(6 * body);
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::transformForceSubspaceToParent(
            const DMat<Scalar> &F_in, DMat<Scalar> &F_out) const
        {
            if (num_output_bodies_ == 1 && num_parent_bodies_ == 1)
            {
                const Transform<Scalar> &X = transforms_and_parent_subindices_[0].first;
                F_out = X.inverseTransformForceSubspace(F_in);
                return;
            }

            F_out.setZero(6 * num_parent_bodies_, F_in.cols());
            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int parent_subindex = transform_and_parent_subindex.second;
                F_out.template middleRows<6>(6 * parent_subindex).noalias() +=
                    X.inverseTransformForceSubspace(F_in.template middleRows<6>(6 * output_body));
                output_body++;
            }
        }

        template <typename Scalar>
        DMat<Scalar> GeneralizedTransform<Scalar>::transformForceSubspaceToParent(
            const DMat<Scalar> &F_in) const
        {
            DMat<Scalar> F_out;
            transformForceSubspaceToParent(F_in, F_out);
            return F_out;
        }

        template <typename Scalar>
        void GeneralizedTransform<Scalar>::inverseTransformForceSubspace(
            DMat<Scalar> &F1, DMat<Scalar> &F2, DMat<Scalar> &F3, DMat<Scalar> &F4) const
        {
            // Fast path for single-body to single-body (most common case)
            // Delegates to Transform::inverseTransformForceSubspace (4-arg) which batches the computation
            if (num_output_bodies_ == 1 && num_parent_bodies_ == 1)
            {
                const Transform<Scalar> &X = transforms_and_parent_subindices_[0].first;
                X.inverseTransformForceSubspace(F1, F2, F3, F4);
                return;
            }

            // Multi-body case: transform each body's portion and accumulate to parent bodies
            // We batch the rotation computation across all 4 matrices for each body
            const int num_cols_1 = F1.cols();
            const int num_cols_2 = F2.cols();
            const int num_cols_3 = F3.cols();
            const int num_cols_4 = F4.cols();

            DMat<Scalar> F1_out = DMat<Scalar>::Zero(6 * num_parent_bodies_, num_cols_1);
            DMat<Scalar> F2_out = DMat<Scalar>::Zero(6 * num_parent_bodies_, num_cols_2);
            DMat<Scalar> F3_out = DMat<Scalar>::Zero(6 * num_parent_bodies_, num_cols_3);
            DMat<Scalar> F4_out = DMat<Scalar>::Zero(6 * num_parent_bodies_, num_cols_4);

            int output_body = 0;
            for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
            {
                const Transform<Scalar> &X = transform_and_parent_subindex.first;
                const int parent_subindex = transform_and_parent_subindex.second;

                // Compute E^T and r_hat*E^T once for this body
                const Mat3<Scalar> ET = X.getRotation().transpose();
                const Mat3<Scalar> r_hat_ET = ori::vectorToSkewMat(X.getTranslation()) * ET;

                // Helper to transform a single body's portion of F and accumulate
                auto transformAndAccumulate = [&](const DMat<Scalar> &F_in, DMat<Scalar> &F_out, int num_cols) {
                    // Extract this body's 6 rows from input
                    const auto F_top = F_in.template block<3, Eigen::Dynamic>(6 * output_body, 0, 3, num_cols);
                    const auto F_bot = F_in.template block<3, Eigen::Dynamic>(6 * output_body + 3, 0, 3, num_cols);

                    // Transform and accumulate to parent body's rows
                    // Top: E^T * F_top + r_hat * E^T * F_bot
                    F_out.template block<3, Eigen::Dynamic>(6 * parent_subindex, 0, 3, num_cols).noalias() +=
                        ET * F_top + r_hat_ET * F_bot;
                    // Bottom: E^T * F_bot
                    F_out.template block<3, Eigen::Dynamic>(6 * parent_subindex + 3, 0, 3, num_cols).noalias() +=
                        ET * F_bot;
                };

                transformAndAccumulate(F1, F1_out, num_cols_1);
                transformAndAccumulate(F2, F2_out, num_cols_2);
                transformAndAccumulate(F3, F3_out, num_cols_3);
                transformAndAccumulate(F4, F4_out, num_cols_4);

                output_body++;
            }

            F1 = std::move(F1_out);
            F2 = std::move(F2_out);
            F3 = std::move(F3_out);
            F4 = std::move(F4_out);
        }

        template class GeneralizedTransform<double>;
        template class GeneralizedTransform<std::complex<double>>;
        template class GeneralizedTransform<float>;
        template class GeneralizedTransform<casadi::SX>;

    } // namespace spatial

} // namespace grbda
