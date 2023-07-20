#pragma once

#include "orientation_tools.h"

namespace grbda
{

    namespace spatial
    {
        using namespace ori;

        class SpatialTransform
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            SpatialTransform(const Mat3<double> &E = Mat3<double>::Identity(),
                             const Vec3<double> &r = Vec3<double>::Zero());

            void setIdentity();
            Mat6<double> toMatrix() const;

            SVec<double> transformMotionVector(const SVec<double> &m_in) const;
            SVec<double> inverseTransformMotionVector(const SVec<double> &m_in) const;

            SVec<double> transformForceVector(const SVec<double> &f_in) const;
            SVec<double> inverseTransformForceVector(const SVec<double> &f_in) const;

            Vec3<double> transformPoint(const Vec3<double> &local_offset) const;
            Vec3<double> inverseTransformPoint(const Vec3<double> &local_offset) const;

            Mat6<double> inverseTransformSpatialInertia(const Mat6<double> &I_in) const;

            D6Mat<double> transformMotionSubspace(const D6Mat<double> &S_in) const;
            D6Mat<double> inverseTransformMotionSubspace(const D6Mat<double> &S_in) const;
            D6Mat<double> inverseTransformForceSubspace(const D6Mat<double> &F_in) const;

            SpatialTransform operator*(const SpatialTransform &X_in) const;

            // TODO(@MatthewChignoli): move to the cpp file
            Mat6<double> rightMultiplyMotionTransform(const Mat6<double> &M_in) const
            {
                Mat6<double> M_out;

                const Mat3<double> &M_TL = M_in.topLeftCorner<3, 3>();
                const Mat3<double> &M_TR = M_in.topRightCorner<3, 3>();
                const Mat3<double> &M_BL = M_in.bottomLeftCorner<3, 3>();
                const Mat3<double> &M_BR = M_in.bottomRightCorner<3, 3>();

                const Mat3<double> r_hat = vectorToSkewMat(r_);

                M_out.topLeftCorner<3, 3>() = M_TL * E_ - M_TR * E_ * r_hat;
                M_out.topRightCorner<3, 3>() = M_TR * E_;
                M_out.bottomLeftCorner<3, 3>() = M_BL * E_ - M_BR * E_ * r_hat;
                M_out.bottomRightCorner<3, 3>() = M_BR * E_;

                return M_out;
            }

            Mat6<double> leftMultiplyForceTransform(const Mat6<double> &M_in) const
            {
                Mat6<double> M_out;
                const Mat3<double> &M_TL = M_in.topLeftCorner<3, 3>();
                const Mat3<double> &M_TR = M_in.topRightCorner<3, 3>();
                const Mat3<double> &M_BL = M_in.bottomLeftCorner<3, 3>();
                const Mat3<double> &M_BR = M_in.bottomRightCorner<3, 3>();

                const Mat3<double> E_trans = E_.transpose();
                const Mat3<double> r_hat = vectorToSkewMat(r_);

                M_out.topLeftCorner<3, 3>() = E_trans * M_TL + r_hat * E_trans * M_BL;
                M_out.topRightCorner<3, 3>() = E_trans * M_TR + r_hat * E_trans * M_BR;
                M_out.bottomLeftCorner<3, 3>() = E_trans * M_BL;
                M_out.bottomRightCorner<3, 3>() = E_trans * M_BR;

                return M_out;
            }

            const Mat3<double> &getRotation() const { return E_; }
            const Vec3<double> &getTranslation() const { return r_; }
            Mat3<double> getSkewTranslationMatrix() const { return vectorToSkewMat(r_); }

        private:
            Mat3<double> E_;
            Vec3<double> r_;
        };

        class GeneralizedAbsoluteSpatialTransform
        {
        public:
            GeneralizedAbsoluteSpatialTransform(){};

            void appendSpatialTransform(const SpatialTransform &X);

            int getNumOutputBodies() const { return num_output_bodies_; }
            const SpatialTransform &getTransformForOutputBody(int output_body_index) const;

            DMat<double> toMatrix() const;

            DVec<double> transformExternalForceVector(const DVec<double> &f_in) const;

            SpatialTransform &operator[](int output_body_index);

        private:
            int num_output_bodies_ = 0;
            std::vector<SpatialTransform> transforms_;
        };

        class GeneralizedSpatialTransform
        {
        public:
            GeneralizedSpatialTransform(int num_parent_bodies);

            void appendSpatialTransformWithClusterAncestorSubIndex(const SpatialTransform &X,
                                                                   const int subindex);

            int getNumOutputBodies() const { return num_output_bodies_; }
            int getNumParentBodies() const { return num_parent_bodies_; }

            DMat<double> toMatrix() const;
            GeneralizedAbsoluteSpatialTransform toAbsolute() const;

            const std::pair<SpatialTransform, int> &
            transform_and_parent_subindex(int output_body_index) const;

            DVec<double> transformMotionVector(const DVec<double> &m_in) const;
            DVec<double> inverseTransformForceVector(const DVec<double> &f_in) const;

            DMat<double> inverseTransformForceSubspace(const DMat<double> &F_in) const;

            DMat<double> inverseTransformSpatialInertia(const DMat<double> &I_in) const;

            SpatialTransform &operator[](int output_body_index);
            GeneralizedSpatialTransform operator*(const GeneralizedSpatialTransform &X_in) const;
            GeneralizedAbsoluteSpatialTransform operator*(const GeneralizedAbsoluteSpatialTransform &X_in) const;

            DMat<double> rightMultiplyMotionTransform(const DMat<double> &M_in) const
            {
                // TODO(@MatthewChignoli): M_in must be 6num_output_bodies_ * 6num_output_bodies_

                DMat<double> M_out = DMat<double>::Zero(6 * num_output_bodies_,
                                                        6 * num_parent_bodies_);

                int output_body = 0;
                for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
                {
                    const SpatialTransform &X = transform_and_parent_subindex.first;
                    const int &parent_subindex = transform_and_parent_subindex.second;

                    for (int i = 0; i < M_in.rows(); i += 6)
                    {
                        M_out.block<6, 6>(i, 6 * parent_subindex) +=
                            X.rightMultiplyMotionTransform(M_in.block<6, 6>(i, 6 * output_body));
                    }

                    output_body++;
                }

                return M_out;
            }

            DMat<double> leftMultiplyForceTransform(const DMat<double> &M_in) const
            {
                // TODO(@MatthewChignoli): M_in must be 6num_output_bodies_ * 6num_parent_bodies_

                DMat<double> M_out = DMat<double>::Zero(6 * num_parent_bodies_,
                                                        6 * num_parent_bodies_);

                int output_body = 0;
                for (const auto &transform_and_parent_subindex : transforms_and_parent_subindices_)
                {
                    const SpatialTransform &X = transform_and_parent_subindex.first;
                    const int &parent_subindex = transform_and_parent_subindex.second;

                    for (int i = 0; i < M_in.cols(); i += 6)
                    {
                        M_out.block<6, 6>(6 * parent_subindex, i) +=
                            X.leftMultiplyForceTransform(M_in.block<6, 6>(6 * output_body, i));
                    }

                    output_body++;
                }

                return M_out;
            }

        private:
            int num_output_bodies_ = 0;
            const int num_parent_bodies_ = 0;
            std::vector<std::pair<SpatialTransform, int>> transforms_and_parent_subindices_;
        };

    } // namespace spatial

} // namespace grbda
