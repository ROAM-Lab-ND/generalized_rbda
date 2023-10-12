#include "JVRC1_Humanoid.hpp"

namespace grbda
{

    ClusterTreeModel JVRC1_Humanoid::buildClusterTreeModel() const
    {
        using namespace GeneralizedJoints;

        ClusterTreeModel model{};

        // Set gravity in z direction
        model.setGravity(Vec3<double>{0., 0., grav});

        // Pelvis
        const std::string pelvis_name = "pelvis";
        const std::string pelvis_parent_name = "ground";
        const SpatialInertia<double> pelvis_spatial_inertia = SpatialInertia<double>{pelvis_mass,
            pelvis_CoM, pelvis_inertia};
        model.appendBody<Free>(pelvis_name, pelvis_spatial_inertia,
                               pelvis_parent_name, spatial::Transform{});

        // Waist yaw
        const std::string waist_y_name = "waist_y";
        const std::string waist_y_parent_name = "pelvis";
        const spatial::Transform waist_y_Xtree = spatial::Transform(R_waist_y, p_waist_y);
        const SpatialInertia<double> waist_y_spatial_inertia = SpatialInertia<double>{waist_y_mass,
            waist_y_CoM, waist_y_inertia};
        auto waist_y = model.registerBody(waist_y_name, waist_y_spatial_inertia,
                                          waist_y_parent_name, waist_y_Xtree);

        // Waist yaw rotor
        const std::string waist_y_rotor_name = "waist_y_rotor";
        const std::string waist_y_rotor_parent_name = "pelvis";
        const spatial::Transform waist_y_rotor_Xtree = spatial::Transform(R_waist_y_rotor, p_waist_y_rotor);
        const SpatialInertia<double> waist_y_rotor_spatial_inertia = SpatialInertia<double>{rotor_mass,
            rotor_CoM, rotor_inertia};
        auto waist_y_rotor = model.registerBody(waist_y_rotor_name, waist_y_rotor_spatial_inertia,
                                                waist_y_rotor_parent_name, waist_y_rotor_Xtree);

        // Waist yaw cluster
        const std::string waist_y_cluster_name = "waist_y";
        GearedTransmissionModule waist_y_module{waist_y, waist_y_rotor,
                                                ori::CoordinateAxis::Z,
                                                ori::CoordinateAxis::Z,
                                                gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(waist_y_cluster_name,
                                                                 waist_y_module);

        // Waist pitch
        const std::string waist_p_name = "waist_p";
        const std::string waist_p_parent_name = "waist_y";
        const spatial::Transform waist_p_Xtree = spatial::Transform(R_waist_p, p_waist_p);
        const SpatialInertia<double> waist_p_spatial_inertia = SpatialInertia<double>{waist_p_mass,
            waist_p_CoM, waist_p_inertia};
        auto waist_p = model.registerBody(waist_p_name, waist_p_spatial_inertia,
                                          waist_p_parent_name, waist_p_Xtree);

        // Waist pitch rotor
        const std::string waist_p_rotor_name = "waist_p_rotor";
        const std::string waist_p_rotor_parent_name = "waist_y";
        const spatial::Transform waist_p_rotor_Xtree = spatial::Transform(R_waist_p_rotor, p_waist_p_rotor);
        const SpatialInertia<double> waist_p_rotor_spatial_inertia = SpatialInertia<double>{rotor_mass,
            rotor_CoM, rotor_inertia};
        auto waist_p_rotor = model.registerBody(waist_p_rotor_name, waist_p_rotor_spatial_inertia,
                                                waist_p_rotor_parent_name, waist_p_rotor_Xtree);

        // Waist pitch cluster
        const std::string waist_p_cluster_name = "waist_p";
        GearedTransmissionModule waist_p_module{waist_p, waist_p_rotor,
                                                ori::CoordinateAxis::Y,
                                                ori::CoordinateAxis::Y,
                                                gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(waist_p_cluster_name,
                                                                 waist_p_module);

        // Waist roll
        const std::string waist_r_name = "waist_r";
        const std::string waist_r_parent_name = "waist_p";
        const spatial::Transform waist_r_Xtree = spatial::Transform(R_waist_r, p_waist_r);
        const SpatialInertia<double> waist_r_spatial_inertia = SpatialInertia<double>{waist_r_mass,
            waist_r_CoM, waist_r_inertia};
        auto waist_r = model.registerBody(waist_r_name, waist_r_spatial_inertia,
                                          waist_r_parent_name, waist_r_Xtree);

        // Waist roll rotor
        const std::string waist_r_rotor_name = "waist_r_rotor";
        const std::string waist_r_rotor_parent_name = "waist_p";
        const spatial::Transform waist_r_rotor_Xtree = spatial::Transform(R_waist_r_rotor, p_waist_r_rotor);
        const SpatialInertia<double> waist_r_rotor_spatial_inertia = SpatialInertia<double>{rotor_mass,
            rotor_CoM, rotor_inertia};
        auto waist_r_rotor = model.registerBody(waist_r_rotor_name, waist_r_rotor_spatial_inertia,
                                                waist_r_rotor_parent_name, waist_r_rotor_Xtree);

        // Waist roll cluster
        const std::string waist_r_cluster_name = "waist_r";
        GearedTransmissionModule waist_r_module{waist_r, waist_r_rotor,
                                                ori::CoordinateAxis::X,
                                                ori::CoordinateAxis::X,
                                                gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(waist_r_cluster_name,
                                                                 waist_r_module);

        // Neck yaw
        const std::string neck_y_name = "neck_y";
        const std::string neck_y_parent_name = "waist_r";
        const spatial::Transform neck_y_Xtree = spatial::Transform(R_neck_y, p_neck_y);
        const SpatialInertia<double> neck_y_spatial_inertia = SpatialInertia<double>{neck_y_mass,
            neck_y_CoM, neck_y_inertia};
        auto neck_y = model.registerBody(neck_y_name, neck_y_spatial_inertia,
                                         neck_y_parent_name, neck_y_Xtree);

        // Neck yaw rotor
        const std::string neck_y_rotor_name = "neck_y_rotor";
        const std::string neck_y_rotor_parent_name = "waist_r";
        const spatial::Transform neck_y_rotor_Xtree = spatial::Transform(R_neck_y_rotor, p_neck_y_rotor);
        const SpatialInertia<double> neck_y_rotor_spatial_inertia = SpatialInertia<double>{rotor_mass,
            rotor_CoM, rotor_inertia};
        auto neck_y_rotor = model.registerBody(neck_y_rotor_name, neck_y_rotor_spatial_inertia,
                                               neck_y_rotor_parent_name, neck_y_rotor_Xtree);

        // Neck yaw cluster
        const std::string neck_y_cluster_name = "neck_y";
        GearedTransmissionModule neck_y_module{neck_y, neck_y_rotor,
                                               ori::CoordinateAxis::Z,
                                               ori::CoordinateAxis::Z,
                                               gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(neck_y_cluster_name,
                                                                 neck_y_module);

        // Neck roll
        const std::string neck_r_name = "neck_r";
        const std::string neck_r_parent_name = "neck_y";
        const spatial::Transform neck_r_Xtree = spatial::Transform(R_neck_r, p_neck_r);
        const SpatialInertia<double> neck_r_spatial_inertia = SpatialInertia<double>{neck_r_mass,
            neck_r_CoM, neck_r_inertia};
        auto neck_r = model.registerBody(neck_r_name, neck_r_spatial_inertia,
                                         neck_r_parent_name, neck_r_Xtree);

        // Neck roll rotor
        const std::string neck_r_rotor_name = "neck_r_rotor";
        const std::string neck_r_rotor_parent_name = "neck_y";
        const spatial::Transform neck_r_rotor_Xtree = spatial::Transform(R_neck_r_rotor, p_neck_r_rotor);
        const SpatialInertia<double> neck_r_rotor_spatial_inertia = SpatialInertia<double>{rotor_mass,
            rotor_CoM, rotor_inertia};
        auto neck_r_rotor = model.registerBody(neck_r_rotor_name, neck_r_rotor_spatial_inertia,
                                               neck_r_rotor_parent_name, neck_r_rotor_Xtree);

        // Neck roll cluster
        const std::string neck_r_cluster_name = "neck_r";
        GearedTransmissionModule neck_r_module{neck_r, neck_r_rotor,
                                               ori::CoordinateAxis::X,
                                               ori::CoordinateAxis::X,
                                               gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(neck_r_cluster_name,
                                                                 neck_r_module);

        // Neck pitch
        const std::string neck_p_name = "neck_p";
        const std::string neck_p_parent_name = "neck_r";
        const spatial::Transform neck_p_Xtree = spatial::Transform(R_neck_p, p_neck_p);
        const SpatialInertia<double> neck_p_spatial_inertia = SpatialInertia<double>{neck_p_mass,
            neck_p_CoM, neck_p_inertia};
        auto neck_p = model.registerBody(neck_p_name, neck_p_spatial_inertia,
                                         neck_p_parent_name, neck_p_Xtree);

        // Neck pitch rotor
        const std::string neck_p_rotor_name = "neck_p_rotor";
        const std::string neck_p_rotor_parent_name = "neck_r";
        const spatial::Transform neck_p_rotor_Xtree = spatial::Transform(R_neck_p_rotor, p_neck_p_rotor);
        const SpatialInertia<double> neck_p_rotor_spatial_inertia = SpatialInertia<double>{rotor_mass,
            rotor_CoM, rotor_inertia};
        auto neck_p_rotor = model.registerBody(neck_p_rotor_name, neck_p_rotor_spatial_inertia,
                                               neck_p_rotor_parent_name, neck_p_rotor_Xtree);

        // Neck pitch cluster
        const std::string neck_p_cluster_name = "neck_p";
        GearedTransmissionModule neck_p_module{neck_p, neck_p_rotor,
                                               ori::CoordinateAxis::Y,
                                               ori::CoordinateAxis::Y,
                                               gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(neck_p_cluster_name,
                                                                 neck_p_module);

        std::vector<std::string> sides = {"left","right"};
        const std::string hip_p_parent_name = pelvis_name;
        const std::string hip_p_rotor_parent_name = pelvis_name;

        // legs
        for (size_t i(0); i < 2; i++)
        {
            const std::string side = sides[i];

            // Hip pitch
            const Mat3<double> R_hip_p = i == 0 ? R_left_hip_p : R_right_hip_p;
            const Vec3<double> p_hip_p = i == 0 ? p_left_hip_p : p_right_hip_p;
            const spatial::Transform hip_p_Xtree = spatial::Transform(R_hip_p, p_hip_p);
            const std::string hip_p_name = side + "_hip_p";
            const SpatialInertia<double> hip_p_spatial_inertia =
                SpatialInertia<double>{hip_p_mass, hip_p_CoM, hip_p_inertia};
            auto hip_p = model.registerBody(hip_p_name, hip_p_spatial_inertia,
                                            hip_p_parent_name, hip_p_Xtree);

            // Hip pitch rotor
            const Mat3<double> R_hip_p_rotor = i == 0 ? R_left_hip_p_rotor
                                                      : R_right_hip_p_rotor;
            const Vec3<double> p_hip_p_rotor = i == 0 ? p_left_hip_p_rotor
                                                      : p_right_hip_p_rotor;
            const spatial::Transform hip_p_rotor_Xtree = spatial::Transform(R_hip_p_rotor,
                                                                            p_hip_p_rotor);
            const std::string hip_p_rotor_name = side + "_hip_p_rotor";
            const SpatialInertia<double> hip_p_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM,
                                       rotor_inertia};
            auto hip_p_rotor = model.registerBody(hip_p_rotor_name,
                                                  hip_p_rotor_spatial_inertia,
                                                  hip_p_rotor_parent_name,
                                                  hip_p_rotor_Xtree);

            // Hip pitch cluster
            const std::string hip_p_cluster_name = side + "_hip_p";
            GearedTransmissionModule hip_p_module{hip_p, hip_p_rotor,
                                                  ori::CoordinateAxis::Y,
                                                  ori::CoordinateAxis::Y,
                                                  gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_p_cluster_name,
                                                                     hip_p_module);

            // Hip roll
            const Mat3<double> R_hip_r = i == 0 ? R_left_hip_r : R_right_hip_r;
            const Vec3<double> p_hip_r = i == 0 ? p_left_hip_r : p_right_hip_r;
            const spatial::Transform hip_r_Xtree = spatial::Transform(R_hip_r, p_hip_r);
            const std::string hip_r_name = side + "_hip_r";
            const std::string hip_r_parent_name = hip_p_name;
            const SpatialInertia<double> hip_r_spatial_inertia =
                SpatialInertia<double>{hip_r_mass, hip_r_CoM, hip_r_inertia};
            auto hip_r = model.registerBody(hip_r_name, hip_r_spatial_inertia,
                                            hip_r_parent_name, hip_r_Xtree);

            // Hip roll rotor
            const Mat3<double> R_hip_r_rotor = i == 0 ? R_left_hip_r_rotor
                                                      : R_right_hip_r_rotor;
            const Vec3<double> p_hip_r_rotor = i == 0 ? p_left_hip_r_rotor
                                                      : p_right_hip_r_rotor;
            const spatial::Transform hip_r_rotor_Xtree = spatial::Transform(R_hip_r_rotor,
                                                                            p_hip_r_rotor);
            const std::string hip_r_rotor_name = side + "_hip_r_rotor";
            const std::string hip_r_rotor_parent_name = hip_p_name;
            const SpatialInertia<double> hip_r_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM,
                                       rotor_inertia};
            auto hip_r_rotor = model.registerBody(hip_r_rotor_name,
                                                  hip_r_rotor_spatial_inertia,
                                                  hip_r_rotor_parent_name,
                                                  hip_r_rotor_Xtree);

            // Hip roll cluster
            const std::string hip_r_cluster_name = side + "_hip_r";
            GearedTransmissionModule hip_r_module{hip_r, hip_r_rotor,
                                                  ori::CoordinateAxis::X,
                                                  ori::CoordinateAxis::X,
                                                  gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_r_cluster_name,
                                                                     hip_r_module);

            // Hip yaw
            const Mat3<double> R_hip_y = i == 0 ? R_left_hip_y : R_right_hip_y;
            const Vec3<double> p_hip_y = i == 0 ? p_left_hip_y : p_right_hip_y;
            const spatial::Transform hip_y_Xtree = spatial::Transform(R_hip_y, p_hip_y);
            const std::string hip_y_name = side + "_hip_y";
            const std::string hip_y_parent_name = hip_r_name;
            const SpatialInertia<double> hip_y_spatial_inertia =
                SpatialInertia<double>{hip_y_mass, hip_y_CoM, hip_y_inertia};
            auto hip_y = model.registerBody(hip_y_name, hip_y_spatial_inertia,
                                            hip_y_parent_name, hip_y_Xtree);

            // Hip yaw rotor
            const Mat3<double> R_hip_y_rotor = i == 0 ? R_left_hip_y_rotor
                                                      : R_right_hip_y_rotor;
            const Vec3<double> p_hip_y_rotor = i == 0 ? p_left_hip_y_rotor
                                                      : p_right_hip_y_rotor;
            const spatial::Transform hip_y_rotor_Xtree = spatial::Transform(R_hip_y_rotor,
                                                                            p_hip_y_rotor);
            const std::string hip_y_rotor_name = side + "_hip_y_rotor";
            const std::string hip_y_rotor_parent_name =  hip_r_name;
            const SpatialInertia<double> hip_y_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM,
                                       rotor_inertia};
            auto hip_y_rotor = model.registerBody(hip_y_rotor_name,
                                                  hip_y_rotor_spatial_inertia,
                                                  hip_y_rotor_parent_name,
                                                  hip_y_rotor_Xtree);

            // Hip yaw cluster
            const std::string hip_y_cluster_name = side + "_hip_y";
            GearedTransmissionModule hip_y_module{hip_y, hip_y_rotor,
                                                  ori::CoordinateAxis::Z,
                                                  ori::CoordinateAxis::Z,
                                                  gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(hip_y_cluster_name,
                                                                     hip_y_module);

            // Knee
            const Mat3<double> R_knee = i == 0 ? R_left_knee : R_right_knee;
            const Vec3<double> p_knee = i == 0 ? p_left_knee : p_right_knee;
            const spatial::Transform knee_Xtree = spatial::Transform(R_knee, p_knee);
            const std::string knee_name = side + "_knee";
            const std::string knee_parent_name = hip_y_name;
            const SpatialInertia<double> knee_spatial_inertia =
                SpatialInertia<double>{knee_mass, knee_CoM, knee_inertia};
            auto knee = model.registerBody(knee_name, knee_spatial_inertia,
                                           knee_parent_name, knee_Xtree);

            // Knee rotor
            const Mat3<double> R_knee_rotor = i == 0 ? R_left_knee_rotor
                                                     : R_right_knee_rotor;
            const Vec3<double> p_knee_rotor = i == 0 ? p_left_knee_rotor
                                                     : p_right_knee_rotor;
            const spatial::Transform knee_rotor_Xtree = spatial::Transform(R_knee_rotor,
                                                                           p_knee_rotor);
            const std::string knee_rotor_name = side + "_knee_rotor";
            const std::string knee_rotor_parent_name = hip_y_name;
            const SpatialInertia<double> knee_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM,
                                       rotor_inertia};
            auto knee_rotor = model.registerBody(knee_rotor_name,
                                                 knee_rotor_spatial_inertia,
                                                 knee_rotor_parent_name,
                                                 knee_rotor_Xtree);

            // Knee cluster
            const std::string knee_cluster_name = side + "_knee";
            GearedTransmissionModule knee_module{knee, knee_rotor,
                                                 ori::CoordinateAxis::Y,
                                                 ori::CoordinateAxis::Y,
                                                 gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(knee_cluster_name,
                                                                     knee_module);

            // Ankle roll
            const Mat3<double> R_ankle_r = i == 0 ? R_left_ankle_r : R_right_ankle_r;
            const Vec3<double> p_ankle_r = i == 0 ? p_left_ankle_r : p_right_ankle_r;
            const spatial::Transform ankle_r_Xtree = spatial::Transform(R_ankle_r, p_ankle_r);
            const std::string ankle_r_name = side + "_ankle_r";
            const std::string ankle_r_parent_name = knee_name;
            const SpatialInertia<double> ankle_r_spatial_inertia =
                SpatialInertia<double>{ankle_r_mass, ankle_r_CoM, ankle_r_inertia};
            auto ankle_r = model.registerBody(ankle_r_name, ankle_r_spatial_inertia,
                                              ankle_r_parent_name, ankle_r_Xtree);

            // Ankle roll rotor
            const Mat3<double> R_ankle_r_rotor = i == 0 ? R_left_ankle_r_rotor
                                                        : R_right_ankle_r_rotor;
            const Vec3<double> p_ankle_r_rotor = i == 0 ? p_left_ankle_r_rotor
                                                        : p_right_ankle_r_rotor;
            const spatial::Transform ankle_r_rotor_Xtree = spatial::Transform(R_ankle_r_rotor,
                                                                              p_ankle_r_rotor);
            const std::string ankle_r_rotor_name = side + "_ankle_r_rotor";
            const std::string ankle_r_rotor_parent_name = knee_name;
            const SpatialInertia<double> ankle_r_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM,
                                       rotor_inertia};
            auto ankle_r_rotor = model.registerBody(ankle_r_rotor_name,
                                                   ankle_r_rotor_spatial_inertia,
                                                   ankle_r_rotor_parent_name,
                                                   ankle_r_rotor_Xtree);

            // Ankle roll cluster
            const std::string ankle_r_cluster_name = side + "_ankle_r";
            GearedTransmissionModule ankle_r_module{ankle_r, ankle_r_rotor,
                                                  ori::CoordinateAxis::X,
                                                  ori::CoordinateAxis::X,
                                                  gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(ankle_r_cluster_name,
                                                                     ankle_r_module);

            // Ankle pitch
            const Mat3<double> R_ankle_p = i == 0 ? R_left_ankle_p : R_right_ankle_p;
            const Vec3<double> p_ankle_p = i == 0 ? p_left_ankle_p : p_right_ankle_p;
            const spatial::Transform ankle_p_Xtree = spatial::Transform(R_ankle_p, p_ankle_p);
            const std::string ankle_p_name = side + "_ankle_p";
            const std::string ankle_p_parent_name = ankle_r_name;
            const SpatialInertia<double> ankle_p_spatial_inertia =
                SpatialInertia<double>{ankle_p_mass, ankle_p_CoM, ankle_p_inertia};
            auto ankle_p = model.registerBody(ankle_p_name, ankle_p_spatial_inertia,
                                              ankle_p_parent_name, ankle_p_Xtree);

            // Ankle pitch rotor
            const Mat3<double> R_ankle_p_rotor = i == 0 ? R_left_ankle_p_rotor
                                                        : R_right_ankle_p_rotor;
            const Vec3<double> p_ankle_p_rotor = i == 0 ? p_left_ankle_p_rotor
                                                        : p_right_ankle_p_rotor;
            const spatial::Transform ankle_p_rotor_Xtree = spatial::Transform(R_ankle_p_rotor,
                                                                              p_ankle_p_rotor);
            const std::string ankle_p_rotor_name = side + "_ankle_p_rotor";
            const std::string ankle_p_rotor_parent_name = ankle_r_name;
            const SpatialInertia<double> ankle_p_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM,
                                       rotor_inertia};
            auto ankle_p_rotor = model.registerBody(ankle_p_rotor_name,
                                                   ankle_p_rotor_spatial_inertia,
                                                   ankle_p_rotor_parent_name,
                                                   ankle_p_rotor_Xtree);

            // Ankle pitch cluster
            const std::string ankle_p_cluster_name = side + "_ankle_p";
            GearedTransmissionModule ankle_p_module{ankle_p, ankle_p_rotor,
                                                  ori::CoordinateAxis::Y,
                                                  ori::CoordinateAxis::Y,
                                                  gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(ankle_p_cluster_name,
                                                                     ankle_p_module);
        }

        const std::string shoulder_p_parent_name = waist_r_name;
        const std::string shoulder_p_rotor_parent_name = waist_r_name;
        
        // arms
        for (size_t i(0); i < 2; i++)
        {
            const std::string side = sides[i];

            // Shoulder pitch
            const Mat3<double> R_shoulder_p = i == 0 ? R_left_shoulder_p : R_right_shoulder_p;
            const Vec3<double> p_shoulder_p = i == 0 ? p_left_shoulder_p : p_right_shoulder_p;
            const spatial::Transform shoulder_p_Xtree = spatial::Transform(R_shoulder_p,
                                                                           p_shoulder_p);
            const std::string shoulder_p_name = side + "_shoulder_p";
            const SpatialInertia<double> shoulder_p_spatial_inertia =
                SpatialInertia<double>{shoulder_p_mass, shoulder_p_CoM, shoulder_p_inertia};
            auto shoulder_p = model.registerBody(shoulder_p_name, shoulder_p_spatial_inertia,
                                                 shoulder_p_parent_name, shoulder_p_Xtree);

            // Shoulder pitch rotor
            const Mat3<double> R_shoulder_p_rotor = i == 0 ? R_left_shoulder_p_rotor
                                                           : R_right_shoulder_p_rotor;
            const Vec3<double> p_shoulder_p_rotor = i == 0 ? p_left_shoulder_p_rotor
                                                           : p_right_shoulder_p_rotor;
            const spatial::Transform shoulder_p_rotor_Xtree =
                spatial::Transform(R_shoulder_p_rotor, p_shoulder_p_rotor);
            const std::string shoulder_p_rotor_name = side + "_shoulder_p_rotor";
            const SpatialInertia<double> shoulder_p_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
            auto shoulder_p_rotor = model.registerBody(shoulder_p_rotor_name,
                                                       shoulder_p_rotor_spatial_inertia,
                                                       shoulder_p_rotor_parent_name,
                                                       shoulder_p_rotor_Xtree);

            // Shoulder pitch cluster
            const std::string shoulder_p_cluster_name = side + "_shoulder_p";
            GearedTransmissionModule shoulder_p_module{shoulder_p, shoulder_p_rotor,
                                                       ori::CoordinateAxis::Y,
                                                       ori::CoordinateAxis::Y,
                                                       gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(shoulder_p_cluster_name,
                                                                     shoulder_p_module);

            // Shoulder roll
            const Mat3<double> R_shoulder_r = i == 0 ? R_left_shoulder_r : R_right_shoulder_r;
            const Vec3<double> p_shoulder_r = i == 0 ? p_left_shoulder_r : p_right_shoulder_r;
            const spatial::Transform shoulder_r_Xtree = spatial::Transform(R_shoulder_r,
                                                                           p_shoulder_r);
            const std::string shoulder_r_name = side + "_shoulder_r";
            const std::string shoulder_r_parent_name = shoulder_p_name;
            const SpatialInertia<double> shoulder_r_spatial_inertia =
                SpatialInertia<double>{shoulder_r_mass, shoulder_r_CoM, shoulder_r_inertia};
            auto shoulder_r = model.registerBody(shoulder_r_name, shoulder_r_spatial_inertia,
                                                 shoulder_r_parent_name, shoulder_r_Xtree);

            // Shoulder roll rotor
            const Mat3<double> R_shoulder_r_rotor = i == 0 ? R_left_shoulder_r_rotor
                                                           : R_right_shoulder_r_rotor;
            const Vec3<double> p_shoulder_r_rotor = i == 0 ? p_left_shoulder_r_rotor
                                                           : p_right_shoulder_r_rotor;
            const spatial::Transform shoulder_r_rotor_Xtree =
                spatial::Transform(R_shoulder_r_rotor, p_shoulder_r_rotor);
            const std::string shoulder_r_rotor_name = side + "_shoulder_r_rotor";
            const std::string shoulder_r_rotor_parent_name = shoulder_p_name;
            const SpatialInertia<double> shoulder_r_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
            auto shoulder_r_rotor = model.registerBody(shoulder_r_rotor_name,
                                                       shoulder_r_rotor_spatial_inertia,
                                                       shoulder_r_rotor_parent_name,
                                                       shoulder_r_rotor_Xtree);

            // Shoulder roll cluster
            const std::string shoulder_r_cluster_name = side + "_shoulder_r";
            GearedTransmissionModule shoulder_r_module{shoulder_r, shoulder_r_rotor,
                                                       ori::CoordinateAxis::X,
                                                       ori::CoordinateAxis::X,
                                                       gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(shoulder_r_cluster_name,
                                                                     shoulder_r_module);

            // Shoulder yaw
            const Mat3<double> R_shoulder_y = i == 0 ? R_left_shoulder_y : R_right_shoulder_y;
            const Vec3<double> p_shoulder_y = i == 0 ? p_left_shoulder_y : p_right_shoulder_y;
            const spatial::Transform shoulder_y_Xtree = spatial::Transform(R_shoulder_y,
                                                                           p_shoulder_y);
            const std::string shoulder_y_name = side + "_shoulder_y";
            const std::string shoulder_y_parent_name = shoulder_r_name;
            const SpatialInertia<double> shoulder_y_spatial_inertia =
                SpatialInertia<double>{shoulder_y_mass, shoulder_y_CoM, shoulder_y_inertia};
            auto shoulder_y = model.registerBody(shoulder_y_name, shoulder_y_spatial_inertia,
                                                 shoulder_y_parent_name, shoulder_y_Xtree);

            // Shoulder yaw rotor
            const Mat3<double> R_shoulder_y_rotor = i == 0 ? R_left_shoulder_y_rotor
                                                           : R_right_shoulder_y_rotor;
            const Vec3<double> p_shoulder_y_rotor = i == 0 ? p_left_shoulder_y_rotor
                                                           : p_right_shoulder_y_rotor;
            const spatial::Transform shoulder_y_rotor_Xtree =
                spatial::Transform(R_shoulder_y_rotor, p_shoulder_y_rotor);
            const std::string shoulder_y_rotor_name = side + "_shoulder_y_rotor";
            const std::string shoulder_y_rotor_parent_name = shoulder_r_name;
            const SpatialInertia<double> shoulder_y_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
            auto shoulder_y_rotor = model.registerBody(shoulder_y_rotor_name,
                                                       shoulder_y_rotor_spatial_inertia,
                                                       shoulder_y_rotor_parent_name,
                                                       shoulder_y_rotor_Xtree);

            // Shoulder yaw cluster
            const std::string shoulder_y_cluster_name = side + "_shoulder_y";
            GearedTransmissionModule shoulder_y_module{shoulder_y, shoulder_y_rotor,
                                                       ori::CoordinateAxis::Z,
                                                       ori::CoordinateAxis::Z,
                                                       gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(shoulder_y_cluster_name,
                                                                     shoulder_y_module);

            // Elbow pitch
            const Mat3<double> R_elbow_p = i == 0 ? R_left_elbow_p : R_right_elbow_p;
            const Vec3<double> p_elbow_p = i == 0 ? p_left_elbow_p : p_right_elbow_p;
            const spatial::Transform elbow_p_Xtree = spatial::Transform(R_elbow_p,
                                                                        p_elbow_p);
            const std::string elbow_p_name = side + "_elbow_p";
            const std::string elbow_p_parent_name = shoulder_y_name;
            const SpatialInertia<double> elbow_p_spatial_inertia =
                SpatialInertia<double>{elbow_p_mass, elbow_p_CoM, elbow_p_inertia};
            auto elbow_p = model.registerBody(elbow_p_name, elbow_p_spatial_inertia,
                                              elbow_p_parent_name, elbow_p_Xtree);

            // Elbow pitch rotor
            const Mat3<double> R_elbow_p_rotor = i == 0 ? R_left_elbow_p_rotor
                                                        : R_right_elbow_p_rotor;
            const Vec3<double> p_elbow_p_rotor = i == 0 ? p_left_elbow_p_rotor
                                                        : p_right_elbow_p_rotor;
            const spatial::Transform elbow_p_rotor_Xtree =
                spatial::Transform(R_elbow_p_rotor, p_elbow_p_rotor);
            const std::string elbow_p_rotor_name = side + "_elbow_p_rotor";
            const std::string elbow_p_rotor_parent_name = shoulder_y_name;
            const SpatialInertia<double> elbow_p_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
            auto elbow_p_rotor = model.registerBody(elbow_p_rotor_name,
                                                    elbow_p_rotor_spatial_inertia,
                                                    elbow_p_rotor_parent_name,
                                                    elbow_p_rotor_Xtree);

            // Elbow pitch cluster
            const std::string elbow_p_cluster_name = side + "_elbow_p";
            GearedTransmissionModule elbow_p_module{elbow_p, elbow_p_rotor,
                                                    ori::CoordinateAxis::Y,
                                                    ori::CoordinateAxis::Y,
                                                    gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(elbow_p_cluster_name,
                                                                     elbow_p_module);

            // Elbow yaw
            const Mat3<double> R_elbow_y = i == 0 ? R_left_elbow_y : R_right_elbow_y;
            const Vec3<double> p_elbow_y = i == 0 ? p_left_elbow_y : p_right_elbow_y;
            const spatial::Transform elbow_y_Xtree = spatial::Transform(R_elbow_y,
                                                                        p_elbow_y);
            const std::string elbow_y_name = side + "_elbow_y";
            const std::string elbow_y_parent_name = elbow_p_name;
            const SpatialInertia<double> elbow_y_spatial_inertia =
                SpatialInertia<double>{elbow_y_mass, elbow_y_CoM, elbow_y_inertia};
            auto elbow_y = model.registerBody(elbow_y_name, elbow_y_spatial_inertia,
                                              elbow_y_parent_name, elbow_y_Xtree);

            // Elbow yaw rotor
            const Mat3<double> R_elbow_y_rotor = i == 0 ? R_left_elbow_y_rotor
                                                        : R_right_elbow_y_rotor;
            const Vec3<double> p_elbow_y_rotor = i == 0 ? p_left_elbow_y_rotor
                                                        : p_right_elbow_y_rotor;
            const spatial::Transform elbow_y_rotor_Xtree =
                spatial::Transform(R_elbow_y_rotor, p_elbow_y_rotor);
            const std::string elbow_y_rotor_name = side + "_elbow_y_rotor";
            const std::string elbow_y_rotor_parent_name = elbow_p_name;
            const SpatialInertia<double> elbow_y_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
            auto elbow_y_rotor = model.registerBody(elbow_y_rotor_name,
                                                    elbow_y_rotor_spatial_inertia,
                                                    elbow_y_rotor_parent_name,
                                                    elbow_y_rotor_Xtree);

            // Elbow yaw cluster
            const std::string elbow_y_cluster_name = side + "_elbow_y";
            GearedTransmissionModule elbow_y_module{elbow_y, elbow_y_rotor,
                                                    ori::CoordinateAxis::Z,
                                                    ori::CoordinateAxis::Z,
                                                    gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(elbow_y_cluster_name,
                                                                     elbow_y_module);

            // Wrist roll
            const Mat3<double> R_wrist_r = i == 0 ? R_left_wrist_r : R_right_wrist_r;
            const Vec3<double> p_wrist_r = i == 0 ? p_left_wrist_r : p_right_wrist_r;
            const spatial::Transform wrist_r_Xtree = spatial::Transform(R_wrist_r,
                                                                        p_wrist_r);
            const std::string wrist_r_name = side + "_wrist_r";
            const std::string wrist_r_parent_name = elbow_y_name;
            const SpatialInertia<double> wrist_r_spatial_inertia =
                SpatialInertia<double>{wrist_r_mass, wrist_r_CoM, wrist_r_inertia};
            auto wrist_r = model.registerBody(wrist_r_name, wrist_r_spatial_inertia,
                                              wrist_r_parent_name, wrist_r_Xtree);

            // Wrist roll rotor
            const Mat3<double> R_wrist_r_rotor = i == 0 ? R_left_wrist_r_rotor
                                                        : R_right_wrist_r_rotor;
            const Vec3<double> p_wrist_r_rotor = i == 0 ? p_left_wrist_r_rotor
                                                        : p_right_wrist_r_rotor;
            const spatial::Transform wrist_r_rotor_Xtree =
                spatial::Transform(R_wrist_r_rotor, p_wrist_r_rotor);
            const std::string wrist_r_rotor_name = side + "_wrist_r_rotor";
            const std::string wrist_r_rotor_parent_name = elbow_y_name;
            const SpatialInertia<double> wrist_r_rotor_spatial_inertia = 
                SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
            auto wrist_r_rotor = model.registerBody(wrist_r_rotor_name,
                                                    wrist_r_rotor_spatial_inertia,
                                                    wrist_r_rotor_parent_name,
                                                    wrist_r_rotor_Xtree);

            // Wrist roll cluster
            const std::string wrist_r_cluster_name = side + "_wrist_r";
            GearedTransmissionModule wrist_r_module{wrist_r, wrist_r_rotor,
                                                    ori::CoordinateAxis::X,
                                                    ori::CoordinateAxis::X,
                                                    gear_ratio};
            model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(wrist_r_cluster_name,
                                                                     wrist_r_module);

        }
            
        // Left wrist yaw
        const std::string left_wrist_y_name = "left_wrist_y";
        const std::string left_wrist_y_parent_name = "left_wrist_r";
        const spatial::Transform left_wrist_y_Xtree = spatial::Transform(R_left_wrist_y, p_left_wrist_y);
        const SpatialInertia<double> wrist_y_spatial_inertia = SpatialInertia<double>{wrist_y_mass,
            left_wrist_y_CoM, wrist_y_inertia};
        auto left_wrist_y = model.registerBody(left_wrist_y_name, wrist_y_spatial_inertia,
                                               left_wrist_y_parent_name, left_wrist_y_Xtree);

        // Left wrist yaw rotor
        const std::string left_wrist_y_rotor_name = "left_wrist_y_rotor";
        const std::string left_wrist_y_rotor_parent_name = "left_wrist_r";
        const spatial::Transform left_wrist_y_rotor_Xtree = spatial::Transform(R_left_wrist_y_rotor,
                                                                               p_left_wrist_y_rotor);
        const SpatialInertia<double> left_wrist_y_rotor_spatial_inertia =
            SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
        auto left_wrist_y_rotor = model.registerBody(left_wrist_y_rotor_name,
                                                     left_wrist_y_rotor_spatial_inertia,
                                                     left_wrist_y_rotor_parent_name,
                                                     left_wrist_y_rotor_Xtree);

        // Left wrist yaw cluster
        const std::string left_wrist_y_cluster_name = "left_wrist_y";
        GearedTransmissionModule left_wrist_y_module{left_wrist_y, left_wrist_y_rotor,
                                                     ori::CoordinateAxis::Z,
                                                     ori::CoordinateAxis::Z,
                                                     gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(left_wrist_y_cluster_name,
                                                                 left_wrist_y_module);

        // Right wrist yaw
        const std::string right_wrist_y_name = "right_wrist_y";
        const std::string right_wrist_y_parent_name = "right_wrist_r";
        const spatial::Transform right_wrist_y_Xtree = spatial::Transform(R_right_wrist_y, p_right_wrist_y);
        const SpatialInertia<double> right_wrist_y_spatial_inertia = SpatialInertia<double>{wrist_y_mass,
            right_wrist_y_CoM, wrist_y_inertia};
        auto right_wrist_y = model.registerBody(right_wrist_y_name, right_wrist_y_spatial_inertia,
                                                right_wrist_y_parent_name, right_wrist_y_Xtree);

        // Right wrist yaw rotor
        const std::string right_wrist_y_rotor_name = "right_wrist_y_rotor";
        const std::string right_wrist_y_rotor_parent_name = "right_wrist_r";
        const spatial::Transform right_wrist_y_rotor_Xtree = spatial::Transform(R_right_wrist_y_rotor,
                                                                               p_right_wrist_y_rotor);
        const SpatialInertia<double> right_wrist_y_rotor_spatial_inertia =
            SpatialInertia<double>{rotor_mass, rotor_CoM, rotor_inertia};
        auto right_wrist_y_rotor = model.registerBody(right_wrist_y_rotor_name,
                                                     right_wrist_y_rotor_spatial_inertia,
                                                     right_wrist_y_rotor_parent_name,
                                                     right_wrist_y_rotor_Xtree);

        // Right wrist yaw cluster
        const std::string right_wrist_y_cluster_name = "right_wrist_y";
        GearedTransmissionModule right_wrist_y_module{right_wrist_y, right_wrist_y_rotor,
                                                     ori::CoordinateAxis::Z,
                                                     ori::CoordinateAxis::Z,
                                                     gear_ratio};
        model.appendRegisteredBodiesAsCluster<RevoluteWithRotor>(right_wrist_y_cluster_name,
                                                                 right_wrist_y_module);

        return model;
    }
}
