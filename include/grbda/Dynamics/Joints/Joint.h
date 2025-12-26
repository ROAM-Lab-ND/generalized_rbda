#ifndef GRBDA_JOINT_H
#define GRBDA_JOINT_H

#include "grbda/Utils/SpatialTransforms.h"
#include "grbda/Dynamics/Joints/OrientationRepresentation.h"

namespace grbda
{

    namespace Joints
    {

        template <typename Scalar = double>
        class Base
        {
        public:
            Base(int num_positions, int num_velocities, std::string name)
                : name_(name), num_positions_(num_positions), num_velocities_(num_velocities)  {}
            virtual ~Base() {}

            virtual std::shared_ptr<Base<Scalar>> clone() const = 0;

            virtual void updateKinematics(const DVec<Scalar> &q, const DVec<Scalar> &qd) = 0;

            const std::string& name() const { return name_; }
            int numPositions() const { return num_positions_; }
            int numVelocities() const { return num_velocities_; }

            const DMat<Scalar> &S() const { return S_; }
            const DMat<Scalar> &Psi() const { return Psi_; }
            const spatial::Transform<Scalar> &XJ() const { return XJ_; }

        protected:
            const std::string name_;             
            const int num_positions_;
            const int num_velocities_;

            spatial::Transform<Scalar> XJ_;
            DMat<Scalar> S_;
            DMat<Scalar> Psi_;
        };

        template <typename Scalar = double,
                  typename OrientationRepresentation = ori_representation::Quaternion>
        class Free : public Base<Scalar>
        {
        public:
            Free(std::string name = "unnamed_free_joint")
                : Base<Scalar>(OrientationRepresentation::num_ori_parameter + 3, 6, name)
            {
                this->S_ = D6Mat<Scalar>::Identity(6, 6);
                this->Psi_ = D6Mat<Scalar>::Identity(6, 6);
            }
            ~Free() {}

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<Free<Scalar, OrientationRepresentation>>(*this);
            }

            void updateKinematics(const DVec<Scalar> &q, const DVec<Scalar> &qd) override
            {
                constexpr int num_ori_param = OrientationRepresentation::num_ori_parameter;
                // CRITICAL FIX: Quaternion floating base has q = [quat(4), pos(3)]
                // Orientation is the FIRST num_ori_param elements, position is the LAST 3 elements

                // Extract orientation (use fixed-size to satisfy static assertions in orientation functions)
                auto q_ori = q.template head<num_ori_param>().eval();

                // CRITICAL FIX #2: Normalize quaternion before use!
                // For quaternion representation (num_ori_param==4) with real scalars (double/float),
                // normalize to handle unnormalized quaternions from integration/perturbation
                normalizeIfQuaternion(q_ori);

                const RotMat<Scalar> R = OrientationRepresentation::getRotationMatrix(q_ori);
                const Vec3<Scalar> q_pos = q.template tail<3>();
                this->XJ_ = spatial::Transform<Scalar>(R, q_pos);
            }

        private:
            // Normalize only for quaternions with real scalar types
            template<typename Derived>
            void normalizeIfQuaternion(Eigen::MatrixBase<Derived>& q_ori) {
                constexpr int num_ori_param = OrientationRepresentation::num_ori_parameter;
                if (num_ori_param == 4 && q_ori.size() == 4) {
                    normalizeIfRealScalar(q_ori);
                }
            }

            // SFINAE: Normalize for real arithmetic types
            template<typename Derived>
            typename std::enable_if<std::is_arithmetic<typename Derived::Scalar>::value, void>::type
            normalizeIfRealScalar(Eigen::MatrixBase<Derived>& vec) {
                vec.normalize();
            }

            // SFINAE: No-op for non-arithmetic types (CasADi, complex)
            template<typename Derived>
            typename std::enable_if<!std::is_arithmetic<typename Derived::Scalar>::value, void>::type
            normalizeIfRealScalar(Eigen::MatrixBase<Derived>& vec) {
                // Do nothing
            }

        public:
            
            OrientationRepresentation orientation_representation_;
        };

        template <typename Scalar = double>
        class Revolute : public Base<Scalar>
        {
        public:
            Revolute(ori::CoordinateAxis axis, std::string name = "unnamed_revolute_joint")
                : Base<Scalar>(1, 1, name), axis_(axis)
            {
                spatial::JointType Rev = spatial::JointType::Revolute;

                this->S_ = D6Mat<Scalar>::Zero(6, 1);
                this->S_.template leftCols<1>() = spatial::jointMotionSubspace<Scalar>(Rev, axis);

                this->Psi_ = D6Mat<Scalar>::Zero(6, 1);
                this->Psi_.template leftCols<1>() = spatial::jointMotionSubspace<Scalar>(Rev, axis);
            }
            ~Revolute() {}

            std::shared_ptr<Base<Scalar>> clone() const override
            {
                return std::make_shared<Revolute<Scalar>>(*this);
            }

            void updateKinematics(const DVec<Scalar> &q, const DVec<Scalar> &qd) override
            {
                this->XJ_ = spatial::rotation<Scalar>(axis_, q[0]);
            }

        private:
            const ori::CoordinateAxis axis_;
        };

    }

} // namespace grbda

#endif // GRBDA_JOINT_H
