#ifndef GRBDA_ROBOTS_MIT_HUMANOID_LEG_H
#define GRBDA_ROBOTS_MIT_HUMANOID_LEG_H

#include "MIT_Humanoid.hpp"

namespace grbda
{

    template <typename Scalar = double>
    class MIT_Humanoid_Leg : public MIT_Humanoid<Scalar>
    {
    public:
        ClusterTreeModel<Scalar> buildClusterTreeModel() const override
        {
            typedef ClusterJoints::GearedTransmissionModule<Scalar> GearedTransModule;
            typedef ClusterJoints::ParallelBeltTransmissionModule<Scalar> ParallelBeltTransModule;
            typedef ClusterJoints::RevoluteWithRotor<Scalar> RevoluteWithRotor;
            typedef ClusterJoints::RevolutePairWithRotor<Scalar> RevolutePairWithRotor;

            ClusterTreeModel<Scalar> model;

            const Mat3<Scalar> I3 = Mat3<Scalar>::Identity();

            // HipRz
            const std::string hip_rz_parent_name = "ground";
            const std::string hip_rz_name = "hip_rz";
            const std::string hip_rz_link_name = "hip_rz_link";
            const std::string hip_rz_rotor_name = "hip_rz_rotor";

            SpatialInertia<Scalar> hip_rz_link_inertia(this->_hipRzLinearInertialParams);
            SpatialInertia<Scalar> hip_rz_rotor_inertia(this->_hipRzRotorLinearInertialParams);

            Mat3<Scalar> Xrot_HipZ = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y,
                                                                     this->_hipRzPitch);
            const spatial::Transform<Scalar> xtreeHipRz(Xrot_HipZ, this->_hipRzLocation);
            const spatial::Transform<Scalar> xtreeHipRzRotor(Xrot_HipZ, this->_hipRzRotorLocation);

            Body<Scalar> hip_rz_link = model.registerBody(hip_rz_link_name, hip_rz_link_inertia,
                                                          hip_rz_parent_name, xtreeHipRz);
            Body<Scalar> hip_rz_rotor = model.registerBody(hip_rz_rotor_name, hip_rz_rotor_inertia,
                                                           hip_rz_parent_name, xtreeHipRzRotor);
            GearedTransModule hip_rz_module{hip_rz_link, hip_rz_rotor,
                                            ori::CoordinateAxis::Z, ori::CoordinateAxis::Z,
                                            this->_hipRzGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_rz_name,
                                                                              hip_rz_module);

            // HipRx
            const std::string hip_rx_parent_name = hip_rz_link_name;
            const std::string hip_rx_name = "hip_rx";
            const std::string hip_rx_link_name = "hip_rx_link";
            const std::string hip_rx_rotor_name = "hip_rx_rotor";

            SpatialInertia<Scalar> hip_rx_link_inertia(this->_hipRxLinearInertialParams);
            SpatialInertia<Scalar> hip_rx_rotor_inertia(this->_hipRxRotorLinearInertialParams);

            Mat3<Scalar> Xrot_HipX = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y,
                                                                     this->_hipRxPitch);
            const spatial::Transform<Scalar> xtreeHipRx(Xrot_HipX, this->_hipRxLocation);
            const spatial::Transform<Scalar> xtreeHipRxRotor(Xrot_HipX, this->_hipRxRotorLocation);

            Body<Scalar> hip_rx_link = model.registerBody(hip_rx_link_name, hip_rx_link_inertia,
                                                          hip_rx_parent_name, xtreeHipRx);
            Body<Scalar> hip_rx_rotor = model.registerBody(hip_rx_rotor_name, hip_rx_rotor_inertia,
                                                           hip_rx_parent_name, xtreeHipRxRotor);
            GearedTransModule hip_rx_module{hip_rx_link, hip_rx_rotor,
                                            ori::CoordinateAxis::X, ori::CoordinateAxis::X,
                                            this->_hipRxGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_rx_name,
                                                                              hip_rx_module);

            // HipRy
            const std::string hip_ry_parent_name = hip_rx_link_name;
            const std::string hip_ry_name = "hip_ry";
            const std::string hip_ry_link_name = "hip_ry_link";
            const std::string hip_ry_rotor_name = "hip_ry_rotor";

            SpatialInertia<Scalar> hip_ry_link_inertia(this->_hipRyLinearInertialParams);
            SpatialInertia<Scalar> hip_ry_rotor_inertia(this->_hipRyRotorLinearInertialParams);

            Mat3<Scalar> Xrot_HipY = ori::coordinateRotation<Scalar>(ori::CoordinateAxis::Y,
                                                                     this->_hipRyPitch);
            const Vec3<Scalar> hipRyLocation = this->_hipRyLocation;
            const Vec3<Scalar> hipRyRotorLocation = this->_hipRyRotorLocation;
            const spatial::Transform<Scalar> xtreeHipRy(Xrot_HipY, hipRyLocation);
            const spatial::Transform<Scalar> xtreeHipRyRotor(Xrot_HipY, hipRyRotorLocation);

            Body<Scalar> hip_ry_link = model.registerBody(hip_ry_link_name, hip_ry_link_inertia,
                                                          hip_ry_parent_name, xtreeHipRy);
            Body<Scalar> hip_ry_rotor = model.registerBody(hip_ry_rotor_name, hip_ry_rotor_inertia,
                                                           hip_ry_parent_name, xtreeHipRyRotor);
            GearedTransModule hip_ry_module{hip_ry_link, hip_ry_rotor,
                                            ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                            this->_hipRyGearRatio};
            model.template appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_ry_name,
                                                                              hip_ry_module);

            const std::string knee_contact_name = "knee_contact";
            model.appendContactPoint(hip_ry_link_name, Vec3<Scalar>(0, 0, -this->_thighLength),
                                     knee_contact_name);

            // Knee
            const std::string knee_parent_name = "hip_ry_link";
            const std::string knee_link_name = "knee_link";
            const std::string knee_rotor_name = "knee_rotor";

            SpatialInertia<Scalar> knee_link_inertia(this->_kneeLinearInertialParams);
            SpatialInertia<Scalar> knee_rotor_inertia(this->_kneeRotorLinearInertialParams);

            const spatial::Transform<Scalar> xtreeKnee(I3, this->_kneeLocation);
            const spatial::Transform<Scalar> xtreeKneeRotor(I3, this->_kneeRotorLocation);

            Body<Scalar> knee_link = model.registerBody(knee_link_name, knee_link_inertia,
                                                        knee_parent_name, xtreeKnee);
            Body<Scalar> knee_rotor = model.registerBody(knee_rotor_name, knee_rotor_inertia,
                                                         knee_parent_name, xtreeKneeRotor);
            ParallelBeltTransModule knee_module{knee_link, knee_rotor,
                                                ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                                this->_kneeGearRatio, this->_kneeBeltRatio};

            // Ankle
            const std::string ankle_parent_name = knee_link_name;
            const std::string ankle_link_name = "ankle_link";
            const std::string ankle_rotor_name = "ankle_rotor";

            SpatialInertia<Scalar> ankle_link_inertia(this->_ankleLinearInertialParams);
            SpatialInertia<Scalar> ankle_rotor_inertia(this->_ankleRotorLinearInertialParams);

            const spatial::Transform<Scalar> xtreeAnkle(I3, this->_ankleLocation);
            const spatial::Transform<Scalar> xtreeAnkleRotor(I3, this->_ankleRotorLocation);

            Body<Scalar> ankle_rotor = model.registerBody(ankle_rotor_name, ankle_rotor_inertia,
                                                          knee_parent_name, xtreeAnkleRotor);
            Body<Scalar> ankle_link = model.registerBody(ankle_link_name, ankle_link_inertia,
                                                         ankle_parent_name, xtreeAnkle);
            ParallelBeltTransModule ankle_module{ankle_link, ankle_rotor,
                                                 ori::CoordinateAxis::Y, ori::CoordinateAxis::Y,
                                                 this->_ankleGearRatio, this->_ankleBeltRatio};

            // Cluster
            const std::string knee_and_ankle_name = "knee_and_ankle";
            model.template appendRegisteredBodiesAsCluster<RevolutePairWithRotor>(
                knee_and_ankle_name, knee_module, ankle_module);

            // Contact Points
            const std::string toe_contact_name = "toe_contact";
            const std::string heel_contact_name = "heel_contact";
            model.appendContactPoint(ankle_link_name,
                                     Vec3<Scalar>(this->_footToeLength, 0, -this->_footHeight),
                                     toe_contact_name);
            model.appendContactPoint(ankle_link_name,
                                     Vec3<Scalar>(-this->_footHeelLength, 0, -this->_footHeight),
                                     heel_contact_name);

            return model;
        }
    };

} // namespace grbda

#endif // GRBDA_ROBOTS_MIT_HUMANOID_LEG_H
