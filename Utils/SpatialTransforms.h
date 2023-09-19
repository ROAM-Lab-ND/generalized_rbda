#ifndef GRBDA_SPATIAL_TRANSFORMS_H
#define GRBDA_SPATIAL_TRANSFORMS_H

#include "OrientationTools.h"

namespace grbda
{

    namespace spatial
    {

        class Transform
        {
        public:
            Transform(const Mat3<double> &E = Mat3<double>::Identity(),
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

            Transform operator*(const Transform &X_in) const;

            const Mat3<double> &getRotation() const { return E_; }
            const Vec3<double> &getTranslation() const { return r_; }
            Mat3<double> getSkewTranslationMatrix() const { return ori::vectorToSkewMat(r_); }

            Mat6<double> rightMultiplyMotionTransform(const Mat6<double> &M_in) const;
            Mat6<double> leftMultiplyForceTransform(const Mat6<double> &M_in) const;

        private:
            Mat3<double> E_;
            Vec3<double> r_;
        };

        class GeneralizedAbsoluteTransform
        {
        public:
            GeneralizedAbsoluteTransform(){};

            void appendTransform(const Transform &X);

            int getNumOutputBodies() const { return num_output_bodies_; }
            const Transform &getTransformForOutputBody(int output_body_index) const;

            DMat<double> toMatrix() const;

            DVec<double> transformExternalForceVector(const DVec<double> &f_in) const;

            Transform &operator[](int output_body_index);

        private:
            int num_output_bodies_ = 0;
            std::vector<Transform> transforms_;
        };

        class GeneralizedTransform
        {
        public:
            GeneralizedTransform(int num_parent_bodies);

            void appendTransformWithClusterAncestorSubIndex(const Transform &X,
                                                                   const int subindex);

            int getNumOutputBodies() const { return num_output_bodies_; }
            int getNumParentBodies() const { return num_parent_bodies_; }

            DMat<double> toMatrix() const;
            GeneralizedAbsoluteTransform toAbsolute() const;

            const std::pair<Transform, int> &
            transform_and_parent_subindex(int output_body_index) const;

            DVec<double> transformMotionVector(const DVec<double> &m_in) const;
            DVec<double> inverseTransformForceVector(const DVec<double> &f_in) const;

            DMat<double> inverseTransformForceSubspace(const DMat<double> &F_in) const;

            DMat<double> inverseTransformSpatialInertia(const DMat<double> &I_in) const;

            Transform &operator[](int output_body_index);
            GeneralizedTransform operator*(const GeneralizedTransform &X_in) const;
            GeneralizedAbsoluteTransform operator*(const GeneralizedAbsoluteTransform &X_in) const;

            DMat<double> rightMultiplyMotionTransform(const DMat<double> &M_in) const;
            DMat<double> leftMultiplyForceTransform(const DMat<double> &M_in) const;

        private:
            int num_output_bodies_ = 0;
            const int num_parent_bodies_ = 0;
            std::vector<std::pair<Transform, int>> transforms_and_parent_subindices_;
        };

    } // namespace spatial

} // namespace grbda

#endif // GRBDA_SPATIAL_TRANSFORMS_H
