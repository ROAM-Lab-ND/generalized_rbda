#pragma once

#include "Utils/cppTypes.h"

namespace grbda
{

    struct ExternalForceAndBodyIndexPair
    {
        ExternalForceAndBodyIndexPair(int index, const SVec<double> &force)
            : index_(index), force_(force) {}
        const int index_;
        const SVec<double> force_;
    };

    struct ContactPoint
    {
        ContactPoint(const int body_index, const Vec3<double> &local_offset, const std::string name,
                     const int num_jacobian_cols)
            : body_index_(body_index), local_offset_(local_offset), name_(name),
              is_end_effector_(false), end_effector_index_(-1),
              jacobian_(D6Mat<double>::Zero(6, num_jacobian_cols)) {}

        ContactPoint(const int body_index, const Vec3<double> &local_offset, const std::string name,
                     const int num_jacobian_cols, const int end_effector_index)
            : body_index_(body_index), local_offset_(local_offset), name_(name),
              is_end_effector_(true), end_effector_index_(end_effector_index),
              jacobian_(D6Mat<double>::Zero(6, num_jacobian_cols)) {}

        ContactPoint &operator=(const ContactPoint &contact_point) { return *this; }

        const int body_index_;
        const Vec3<double> local_offset_;
        const std::string name_;
        const bool is_end_effector_;
        const int end_effector_index_;

        Vec3<double> position_;
        Vec3<double> velocity_;
        D6Mat<double> jacobian_;

        std::vector<int> supporting_nodes_;
        std::vector<DMat<double>> ChiUp_;
    };

} // namespace grbda
