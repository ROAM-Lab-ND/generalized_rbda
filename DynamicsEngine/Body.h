#pragma once

#include <string>
#include "Utils/Utilities/SpatialInertia.h"
#include "Utils/Utilities/SpatialTransforms.h"

namespace grbda
{

  struct Body
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Body(const int index, const std::string name, const int parent_index,
         const SpatialTransform Xtree, const SpatialInertia<double> inertia,
         const int sub_index_within_cluster, const int cluster_ancestor_sub_index_within_cluster)
        : index_(index), name_(name), parent_index_(parent_index), Xtree_(Xtree),
          inertia_(inertia), sub_index_within_cluster_(sub_index_within_cluster),
          cluster_ancestor_sub_index_within_cluster_(cluster_ancestor_sub_index_within_cluster) {}

    const int index_;
    const std::string name_;
    const int parent_index_;
    const SpatialTransform Xtree_;
    const SpatialInertia<double> inertia_;
    const int sub_index_within_cluster_;
    const int cluster_ancestor_sub_index_within_cluster_;
  };

} // namespace grbda
