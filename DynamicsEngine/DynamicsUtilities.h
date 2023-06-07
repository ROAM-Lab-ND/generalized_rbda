#pragma once

#include "Utils/cppTypes.h"

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
          jacobian_(D6Mat<double>::Zero(6, num_jacobian_cols)) {}

    const int body_index_;
    const Vec3<double> local_offset_;
    const std::string name_;

    Vec3<double> position_;
    Vec3<double> velocity_;
    D6Mat<double> jacobian_;
};
