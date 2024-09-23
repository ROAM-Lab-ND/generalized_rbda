#ifndef GRBDA_SPATIAL_TRANSFORMS_H
#define GRBDA_SPATIAL_TRANSFORMS_H

#include "OrientationTools.h"
#include "grbda/Urdf/pose.h"

namespace grbda
{

    namespace spatial
    {

        template <typename Scalar = double>
        class Transform
        {
        public:
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

            D6Mat<Scalar> transformMotionSubspace(const D6Mat<Scalar> &S_in) const;
            D6Mat<Scalar> inverseTransformMotionSubspace(const D6Mat<Scalar> &S_in) const;
            D6Mat<Scalar> inverseTransformForceSubspace(const D6Mat<Scalar> &F_in) const;

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
            GeneralizedAbsoluteTransform(){};

            void appendTransform(const Transform<Scalar> &X);

            int getNumOutputBodies() const { return num_output_bodies_; }
            const Transform<Scalar> &getTransformForOutputBody(int output_body_index) const;

            DMat<Scalar> toMatrix() const;

            DVec<Scalar> transformExternalForceVector(const DVec<Scalar> &f_in) const;

            Transform<Scalar> &operator[](int output_body_index);

        private:
            int num_output_bodies_ = 0;
            std::vector<Transform<Scalar>> transforms_;
        };

        template <typename Scalar = double>
        class GeneralizedTransform
        {
        public:
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

            DMat<Scalar> inverseTransformSpatialInertia(const DMat<Scalar> &I_in) const;

            Transform<Scalar> &operator[](int output_body_index);
            GeneralizedTransform<Scalar> operator*(const GeneralizedTransform<Scalar> &X_in) const;
            GeneralizedAbsoluteTransform<Scalar> operator*(
                const GeneralizedAbsoluteTransform<Scalar> &X_in) const;

            DMat<Scalar> rightMultiplyMotionTransform(const DMat<Scalar> &M_in) const;
            DMat<Scalar> leftMultiplyForceTransform(const DMat<Scalar> &M_in) const;

        private:
            int num_output_bodies_ = 0;
            const int num_parent_bodies_ = 0;
            std::vector<std::pair<Transform<Scalar>, int>> transforms_and_parent_subindices_;
        };

    } // namespace spatial

} // namespace grbda

#endif // GRBDA_SPATIAL_TRANSFORMS_H
