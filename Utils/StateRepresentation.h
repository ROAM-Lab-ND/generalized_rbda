#ifndef GRBDA_STATE_REPRESENTATION_H
#define GRBDA_STATE_REPRESENTATION_H

#include "cppTypes.h"

namespace grbda
{

    template <typename Scalar = double>
    class JointCoordinate : public DVec<Scalar>
    {
    public:
        JointCoordinate(const DVec<Scalar> &vec, bool is_spanning)
            : DVec<Scalar>(vec), _is_spanning(is_spanning) {}

        const bool &isSpanning() const { return _is_spanning; }

        // TODO(@MatthewChignoli): Should this be a copy constructor?
        // TODO(@MatthewChignoli): Regardless, it should deduce the scalar type, not assume it
        JointCoordinate(const JointCoordinate<Scalar> &other)
            : DVec<Scalar>(other), _is_spanning(other._is_spanning) {}

        JointCoordinate &operator=(const JointCoordinate<Scalar> &other)
        {
            this->DVec<Scalar>::operator=(other);
            _is_spanning = other._is_spanning;
            return *this;
        }

        template <typename Derived>
        JointCoordinate<Scalar> &operator=(const Eigen::DenseBase<Derived> &x)
        {
            this->DVec<Scalar>::operator=(x);
            return *this;
        }

    private:
        bool _is_spanning;
    };

    template <typename Scalar = double>
    struct JointState
    {
        JointState(const JointCoordinate<Scalar> &pos, const JointCoordinate<Scalar> &vel)
            : position(pos), velocity(vel) {}

        JointState(bool position_is_spanning, bool velocity_is_spanning)
            : position(JointCoordinate<Scalar>(DVec<Scalar>::Zero(0), position_is_spanning)),
              velocity(JointCoordinate<Scalar>(DVec<Scalar>::Zero(0), velocity_is_spanning)) {}

        JointState()
            : position(JointCoordinate<Scalar>(DVec<Scalar>::Zero(0), false)),
              velocity(JointCoordinate<Scalar>(DVec<Scalar>::Zero(0), false)) {}

        JointCoordinate<Scalar> position;
        JointCoordinate<Scalar> velocity;
    };

    template <typename Scalar = double>
    using ModelState = std::vector<JointState<Scalar>>;

    template <typename Scalar>
    inline std::pair<DVec<Scalar>, DVec<Scalar>> modelStateToVector(const ModelState<Scalar>& state)
    {
        DVec<Scalar> position(0);
        DVec<Scalar> velocity(0);
        for (int i = 0; i < state.size(); ++i)
        {
            position = appendEigenVector(position, state[i].position);
            velocity = appendEigenVector(velocity, state[i].velocity);
        }
        return std::make_pair(position, velocity);
    }

    template <typename Scalar>
    struct ExternalForceAndBodyIndexPair
    {
        ExternalForceAndBodyIndexPair(int index, const SVec<Scalar> &force)
            : index_(index), force_(force) {}
        const int index_;
        const SVec<Scalar> force_;
    };

    template <typename Scalar = double>
    struct ContactPoint
    {
        ContactPoint(const int body_index, const Vec3<Scalar> &local_offset, const std::string name,
                     const int num_jacobian_cols)
            : body_index_(body_index), local_offset_(local_offset), name_(name),
              is_end_effector_(false), end_effector_index_(-1),
              jacobian_(D6Mat<Scalar>::Zero(6, num_jacobian_cols)) {}

        ContactPoint(const int body_index, const Vec3<Scalar> &local_offset, const std::string name,
                     const int num_jacobian_cols, const int end_effector_index)
            : body_index_(body_index), local_offset_(local_offset), name_(name),
              is_end_effector_(true), end_effector_index_(end_effector_index),
              jacobian_(D6Mat<Scalar>::Zero(6, num_jacobian_cols)) {}

        ContactPoint(const ContactPoint<Scalar> &other)
            : body_index_(other.body_index_), local_offset_(other.local_offset_),
              name_(other.name_), is_end_effector_(other.is_end_effector_),
              end_effector_index_(other.end_effector_index_), position_(other.position_),
              velocity_(other.velocity_), jacobian_(other.jacobian_),
              supporting_nodes_(other.supporting_nodes_), ChiUp_(other.ChiUp_) {}

        ContactPoint<Scalar> &operator=(const ContactPoint<Scalar> &contact_point) { return *this; }

        const int body_index_;
        const Vec3<Scalar> local_offset_;
        const std::string name_;
        const bool is_end_effector_;
        const int end_effector_index_;

        Vec3<Scalar> position_;
        Vec3<Scalar> velocity_;
        D6Mat<Scalar> jacobian_;

        std::vector<int> supporting_nodes_;
        std::vector<DMat<Scalar>> ChiUp_;
    };

} // namespace grbda

#endif // GRBDA_STATE_REPRESENTATION_H
