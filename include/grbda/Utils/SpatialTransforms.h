#ifndef GRBDA_SPATIAL_TRANSFORMS_H
#define GRBDA_SPATIAL_TRANSFORMS_H

#include "OrientationTools.h"
#include "urdf_model/pose.h"

namespace grbda
{

    namespace spatial
    {

        template <typename Scalar = double>
        class Transform
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Transform(const Mat3<Scalar> &E = Mat3<Scalar>::Identity(),
                      const Vec3<Scalar> &r = Vec3<Scalar>::Zero());

            Transform(const urdf::Pose &pose);

            void setIdentity();
            Mat6<Scalar> toMatrix() const;

            SVec<Scalar> transformMotionVector(const SVec<Scalar> &m_in) const;
            SVec<Scalar> inverseTransformMotionVector(const SVec<Scalar> &m_in) const;

            SVec<Scalar> transformForceVector(const SVec<Scalar> &f_in) const;
            SVec<Scalar> inverseTransformForceVector(const SVec<Scalar> &f_in) const;

            Vec3<Scalar> transformPoint(const Vec3<Scalar> &local_offset) const;
            Vec3<Scalar> inverseTransformPoint(const Vec3<Scalar> &local_offset) const;

            Mat6<Scalar> inverseTransformSpatialInertia(const Mat6<Scalar> &I_in) const;

            // Transform spatial inertia from body frame to world frame
            // I_world = X^{-T} * I_body * X^{-1}  where X transforms world -> body
            // This is the inverse operation of inverseTransformSpatialInertia
            Mat6<Scalar> transformSpatialInertiaToWorld(const Mat6<Scalar> &I_in) const;

            D6Mat<Scalar> transformMotionSubspace(const D6Mat<Scalar> &S_in) const;
            void transformMotionSubspace(const D6Mat<Scalar> &S_in, D6Mat<Scalar> &S_out) const;

            D6Mat<Scalar> inverseTransformMotionSubspace(const D6Mat<Scalar> &S_in) const;
            void inverseTransformMotionSubspace(const D6Mat<Scalar> &S_in, D6Mat<Scalar> &S_out) const;

            D6Mat<Scalar> inverseTransformForceSubspace(const D6Mat<Scalar> &F_in) const;
            void inverseTransformForceSubspace(const D6Mat<Scalar> &F_in, D6Mat<Scalar> &F_out) const;

            // Batched version: transforms 4 force subspaces with a single computation of E^T and r_hat*E^T
            void inverseTransformForceSubspace(
                DMat<Scalar> &F1, DMat<Scalar> &F2, DMat<Scalar> &F3, DMat<Scalar> &F4) const;

            Transform<Scalar> operator*(const Transform<Scalar> &X_in) const;

            const Mat3<Scalar> &getRotation() const { return E_; }
            const Vec3<Scalar> &getTranslation() const { return r_; }
            Mat3<Scalar> getSkewTranslationMatrix() const { return ori::vectorToSkewMat(r_); }

            Mat6<Scalar> rightMultiplyMotionTransform(const Mat6<Scalar> &M_in) const;
            Mat6<Scalar> leftMultiplyForceTransform(const Mat6<Scalar> &M_in) const;

        private:
            Mat3<Scalar> E_;
            Vec3<Scalar> r_;
        };

        template <typename Scalar = double>
        class GeneralizedAbsoluteTransform
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            GeneralizedAbsoluteTransform(){};

            void appendTransform(const Transform<Scalar> &X);

            int getNumOutputBodies() const { return num_output_bodies_; }
            const Transform<Scalar> &getTransformForOutputBody(int output_body_index) const;

            DMat<Scalar> toMatrix() const;

            DVec<Scalar> transformExternalForceVector(const DVec<Scalar> &f_in) const;

            Transform<Scalar> &operator[](int output_body_index);

            // World-frame CRBA support methods
            // Transforms motion subspace from local body frames to world frame
            // S_local has rows grouped by body, each group in that body's local frame
            // Returns S_world with all rows in world frame
            DMat<Scalar> transformMotionSubspaceToWorld(const DMat<Scalar> &S_local) const;

        private:
            int num_output_bodies_ = 0;
            std::vector<Transform<Scalar>,Eigen::aligned_allocator<Transform<Scalar>>> transforms_;
        };

        template <typename Scalar = double>
        class GeneralizedTransform
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            GeneralizedTransform(int num_parent_bodies);

            void appendTransformWithClusterAncestorSubIndex(const Transform<Scalar> &X,
                                                            const int subindex);

            int getNumOutputBodies() const { return num_output_bodies_; }
            int getNumParentBodies() const { return num_parent_bodies_; }

            DMat<Scalar> toMatrix() const;
            GeneralizedAbsoluteTransform<Scalar> toAbsolute() const;

            const std::pair<Transform<Scalar>, int> &
            transform_and_parent_subindex(int output_body_index) const;

            DVec<Scalar> transformMotionVector(const DVec<Scalar> &m_in) const;
            DVec<Scalar> inverseTransformForceVector(const DVec<Scalar> &f_in) const;

            DMat<Scalar> inverseTransformForceSubspace(const DMat<Scalar> &F_in) const;
            void inverseTransformForceSubspace(const DMat<Scalar> &F_in, DMat<Scalar> &F_out) const;

            DMat<Scalar> inverseTransformSpatialInertia(const DMat<Scalar> &I_in) const;
            void inverseTransformSpatialInertia(const DMat<Scalar> &I_in, DMat<Scalar> &I_out) const;

            Transform<Scalar> &operator[](int output_body_index);
            const Transform<Scalar> &operator[](int output_body_index) const;
            GeneralizedTransform<Scalar> operator*(const GeneralizedTransform<Scalar> &X_in) const;
            GeneralizedAbsoluteTransform<Scalar> operator*(
                const GeneralizedAbsoluteTransform<Scalar> &X_in) const;

            DMat<Scalar> rightMultiplyMotionTransform(const DMat<Scalar> &M_in) const;
            void rightMultiplyMotionTransform(const DMat<Scalar> &M_in, DMat<Scalar> &M_out) const;
            DMat<Scalar> leftMultiplyForceTransform(const DMat<Scalar> &M_in) const;
            void leftMultiplyForceTransform(const DMat<Scalar> &M_in, DMat<Scalar> &M_out) const;

            // Accumulates child's composite inertia blocks to parent's composite inertia blocks.
            void accumulateBlockDiagonalInertia(
                const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &I_child,
                std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &I_parent) const;

            // Accumulates two block-diagonal child matrices to corresponding parent matrices.
            // Transforms and adds each 6x6 block; does not assume any structure within the blocks.
            void accumulateBlockDiagonalPair(
                const DMat<Scalar> &M1_child, DMat<Scalar> &M1_parent,
                const DMat<Scalar> &M2_child, DMat<Scalar> &M2_parent) const;

            // Computes F = Ic * S exploiting block structure of Ic (vector<Mat6> form).
            DMat<Scalar> blockDiagonalInertiaTimesMotionSubspace(
                const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &Ic,
                const DMat<Scalar> &S) const;
            void blockDiagonalInertiaTimesMotionSubspace(
                const std::vector<Mat6<Scalar>, Eigen::aligned_allocator<Mat6<Scalar>>> &Ic,
                const DMat<Scalar> &S, DMat<Scalar> &out) const;

            // Computes F = Ic * S exploiting block-diagonal structure (DMat form, for M_cup/B_cup).
            DMat<Scalar> blockDiagonalInertiaTimesMotionSubspace(
                const DMat<Scalar> &Ic_block_diag, const DMat<Scalar> &S) const;
            void blockDiagonalInertiaTimesMotionSubspace(
                const DMat<Scalar> &Ic_block_diag, const DMat<Scalar> &S, DMat<Scalar> &out) const;

            // Transforms F from child frame to parent frame, accumulating to connected parent bodies.
            // This is similar to inverseTransformForceSubspace but optimized for the CRBA pattern
            // where we know the structure comes from block-diagonal Ic * S.
            DMat<Scalar> transformForceSubspaceToParent(const DMat<Scalar> &F_in) const;
            void transformForceSubspaceToParent(const DMat<Scalar> &F_in, DMat<Scalar> &F_out) const;

            // Batched version: transforms 4 force subspaces in one call.
            // For single-body to single-body, delegates to Transform::inverseTransformForceSubspace (4-arg).
            // For multi-body clusters, shares rotation computation across bodies and matrices.
            void inverseTransformForceSubspace(
                DMat<Scalar> &F1, DMat<Scalar> &F2, DMat<Scalar> &F3, DMat<Scalar> &F4) const;

        private:
            int num_output_bodies_ = 0;
            const int num_parent_bodies_ = 0;
            std::vector<std::pair<Transform<Scalar>, int>, Eigen::aligned_allocator<std::pair<Transform<Scalar>, int>>> transforms_and_parent_subindices_;
        };

    } // namespace spatial

} // namespace grbda

#endif // GRBDA_SPATIAL_TRANSFORMS_H
