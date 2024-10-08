#ifndef GRBDA_BODY_H
#define GRBDA_BODY_H

#include <string>
#include "grbda/Utils/SpatialInertia.h"
#include "grbda/Utils/SpatialTransforms.h"

namespace grbda
{

  template <typename Scalar = double>
  struct Body
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Body(const int index, const std::string name, const int parent_index,
         const spatial::Transform<Scalar> Xtree, const SpatialInertia<Scalar> inertia,
         const int sub_index_within_cluster, const int cluster_ancestor_index,
         const int cluster_ancestor_sub_index_within_cluster)
        : index_(index), name_(name), parent_index_(parent_index), Xtree_(Xtree),
          inertia_(inertia), sub_index_within_cluster_(sub_index_within_cluster),
          cluster_ancestor_index_(cluster_ancestor_index),
          cluster_ancestor_sub_index_within_cluster_(cluster_ancestor_sub_index_within_cluster) {}

    Body(const Body<Scalar> &body)
        : index_(body.index_), name_(body.name_), parent_index_(body.parent_index_),
          Xtree_(body.Xtree_), inertia_(body.inertia_),
          sub_index_within_cluster_(body.sub_index_within_cluster_),
          cluster_ancestor_index_(body.cluster_ancestor_index_),
          cluster_ancestor_sub_index_within_cluster_(
              body.cluster_ancestor_sub_index_within_cluster_) {}

    Body<Scalar> &operator=(const Body<Scalar> &body) { return *this; }

    const int index_;
    const std::string name_;
    const int parent_index_;
    const spatial::Transform<Scalar> Xtree_;
    const SpatialInertia<Scalar> inertia_;
    const int sub_index_within_cluster_;
    const int cluster_ancestor_index_;
    const int cluster_ancestor_sub_index_within_cluster_;
  };

  template <typename Scalar>
  Body<Scalar> randomBody(const int index, const int parent_index,
                          const int sub_index_within_cluster, const int cluster_ancestor_index,
                          const int cluster_ancestor_sub_index_within_cluster)
  {
    const Mat3<Scalar> E = ori::rpyToRotMat(Vec3<Scalar>::Random());
    const Vec3<Scalar> r = Vec3<Scalar>::Random();
    const spatial::Transform<Scalar> Xtree(E, r);
    const SpatialInertia<Scalar> inertia = SpatialInertia<Scalar>::createRandomInertia();

    return Body<Scalar>(index, "random_body", parent_index, Xtree, inertia,
                        sub_index_within_cluster, cluster_ancestor_index,
                        cluster_ancestor_sub_index_within_cluster);
  }

} // namespace grbda

#endif // GRBDA_BODY_H
