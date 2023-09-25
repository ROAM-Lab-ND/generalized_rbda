#ifndef GRBDA_TEST_HELPERS_H
#define GRBDA_TEST_HELPERS_H

#include "Dynamics/ClusterTreeModel.h"

using namespace grbda;

inline ClusterTreeModel extractGenericJointModel(const ClusterTreeModel &model)
{
    using namespace ClusterJoints;

    ClusterTreeModel generic_model{};

    for (const auto &cluster : model.clusters())
    {
        std::vector<Body> bodies;
        std::vector<JointPtr<double>> joints;

        // Register bodies
        for (auto pair : cluster->bodiesAndJoints())
        {
            Body body_i = pair.first;
            bodies.push_back(body_i);

            JointPtr<double> joint_i = pair.second;
            joints.push_back(joint_i);

            bool is_base = body_i.parent_index_ == -1;
            std::string parent_name = is_base ? "ground" : model.body(body_i.parent_index_).name_;
            generic_model.registerBody(body_i.name_, body_i.inertia_, parent_name, body_i.Xtree_);
        }

        // Extract Loop Constraint and Append Cluster
        std::shared_ptr<LoopConstraint::Base<double>> constraint = cluster->joint_->cloneLoopConstraint();
        generic_model.appendRegisteredBodiesAsCluster<Generic<>>(cluster->name_, bodies,
                                                                 joints, constraint);
    }

    return generic_model;
}

#endif // GRBDA_TEST_HELPERS_H
