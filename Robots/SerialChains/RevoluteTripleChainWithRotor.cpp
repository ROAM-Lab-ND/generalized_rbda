#include "RevoluteTripleChainWithRotor.hpp"

namespace grbda
{

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar> 
    RevoluteTripleChainWithRotor<N, Scalar>::buildRandomClusterTreeModel() const
    {
        ClusterTreeModel<Scalar> model{};

        std::string parent_name = "ground";
        for (size_t i(0); i < N / 3; i++)
        {
            // Link A
            const std::string linkA_name = "link-A-" + std::to_string(i);
            const auto linkA_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkA_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkA_joint_axis = ori::randomCoordinateAxis();
            auto linkA = model.registerBody(linkA_name, linkA_inertia, parent_name, linkA_Xtree);

            // Link B
            const std::string linkB_name = "link-B-" + std::to_string(i);
            const auto linkB_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkB_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkB_joint_axis = ori::randomCoordinateAxis();
            auto linkB = model.registerBody(linkB_name, linkB_inertia, linkA_name, linkB_Xtree);

            // Link C
            const std::string linkC_name = "link-C-" + std::to_string(i);
            const auto linkC_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto linkC_inertia = this->randomLinkSpatialInertia();
            ori::CoordinateAxis linkC_joint_axis = ori::randomCoordinateAxis();
            auto linkC = model.registerBody(linkC_name, linkC_inertia, linkB_name, linkC_Xtree);

            // Rotor A
            const std::string rotorA_name = "rotor-A-" + std::to_string(i);
            const auto rotorA_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto rotorA_inertia = this->randomRotorSpatialInertia();
            ori::CoordinateAxis rotorA_joint_axis = ori::randomCoordinateAxis();
            auto rotorA = model.registerBody(rotorA_name, rotorA_inertia,
                                             parent_name, rotorA_Xtree);

            // Rotor B
            const std::string rotorB_name = "rotor-B-" + std::to_string(i);
            const auto rotorB_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto rotorB_inertia = this->randomRotorSpatialInertia();
            ori::CoordinateAxis rotorB_joint_axis = ori::randomCoordinateAxis();
            auto rotorB = model.registerBody(rotorB_name, rotorB_inertia,
                                             parent_name, rotorB_Xtree);

            // Rotor C
            const std::string rotorC_name = "rotor-C-" + std::to_string(i);
            const auto rotorC_Xtree = spatial::randomSpatialRotation<Scalar>();
            const auto rotorC_inertia = this->randomRotorSpatialInertia();
            ori::CoordinateAxis rotorC_joint_axis = ori::randomCoordinateAxis();
            auto rotorC = model.registerBody(rotorC_name, rotorC_inertia,
                                             parent_name, rotorC_Xtree);

            // Cluster
            TransmissionModule moduleA{linkA, rotorA, linkA_joint_axis, rotorA_joint_axis,
                                       this->randomGearRatio(), this->randomGearRatio()};
            TransmissionModule moduleB{linkB, rotorB, linkB_joint_axis, rotorB_joint_axis,
                                       this->randomGearRatio(), this->randomGearRatio()};
            TransmissionModule moduleC{linkC, rotorC, linkC_joint_axis, rotorC_joint_axis,
                                       this->randomGearRatio(), this->randomGearRatio()};

            const std::string cluster_name = "cluster-" + std::to_string(i);
            model.template appendRegisteredBodiesAsCluster<RevTripleWithRotor>(
                cluster_name, moduleA, moduleB, moduleC);

            // Contact points
            appendContactPoints(model, i, linkA_name, linkB_name, linkC_name);

            parent_name = linkC_name;
        }

        return model;
    }

    template <size_t N, typename Scalar>
    ClusterTreeModel<Scalar>
    RevoluteTripleChainWithRotor<N, Scalar>::buildUniformClusterTreeModel() const
    {
        throw std::runtime_error("Not implemented");
    }

    template <size_t N, typename Scalar>
    void RevoluteTripleChainWithRotor<N, Scalar>::appendContactPoints(
        ClusterTreeModel<Scalar> &model, const int i, const std::string linkA_name,
        const std::string linkB_name, const std::string linkC_name) const
    {
        const std::string cpA_name = "cp-A-" + std::to_string(i);
        const Vec3<Scalar> cpA_local_offset = Vec3<Scalar>::Random();
        model.appendContactPoint(linkA_name, cpA_local_offset, cpA_name);

        const std::string cpB_name = "cp-B-" + std::to_string(i);
        const Vec3<Scalar> cpB_local_offset = Vec3<Scalar>::Random();
        model.appendContactPoint(linkB_name, cpB_local_offset, cpB_name);

        const std::string cpC_name = "cp-C-" + std::to_string(i);
        const Vec3<Scalar> cpC_local_offset = Vec3<Scalar>::Random();
        if (i == N / 3 - 1)
            model.appendEndEffector(linkC_name, cpC_local_offset, cpC_name);
        else
            model.appendContactPoint(linkC_name, cpC_local_offset, cpC_name);
    }

    template class RevoluteTripleChainWithRotor<3ul>;
    template class RevoluteTripleChainWithRotor<6ul>;
    template class RevoluteTripleChainWithRotor<9ul>;
    template class RevoluteTripleChainWithRotor<12ul>;
    template class RevoluteTripleChainWithRotor<15ul>;

} // namespace grbda
