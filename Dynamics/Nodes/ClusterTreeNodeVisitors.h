#pragma once

#include <vector>
#include <boost/variant.hpp>
#include <type_traits>

#include "Dynamics/Body.h"
#include "Dynamics/GeneralizedJoints/GeneralizedJointTypes.h"

// TODO(@MatthewChignoli): Breakout a cpp or hxx file somehow

namespace grbda
{
    // using namespace spatial;

    namespace ClusterNodeVisitors
    {

        template <typename NodeType>
        struct BodiesVisitor : public boost::static_visitor<const std::vector<Body> &>
        {
            template <typename T>
            const std::vector<Body> &operator()(const T &node) const
            {
                return node.bodies();
            }

            static const std::vector<Body> &run(const NodeType &node)
            {
                return boost::apply_visitor(BodiesVisitor(), node);
            }
        };

        template <typename NodeType>
        inline const std::vector<Body> &bodies(const NodeType &node)
        {
            return BodiesVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ContainsBodyVisitor : public boost::static_visitor<bool>
        {
            int body_index_;

            ContainsBodyVisitor(int body_index) : body_index_(body_index) {}

            template <typename T>
            bool operator()(const T &node) const
            {
                return node.containsBody(body_index_);
            }

            static bool run(const NodeType &node, int body_index)
            {
                return boost::apply_visitor(ContainsBodyVisitor(body_index), node);
            }
        };

        template <typename NodeType>
        inline bool containsBody(const NodeType &node, int body_index)
        {
            return ContainsBodyVisitor<NodeType>::run(node, body_index);
        }

        template <typename NodeType>
        struct QddSubtreeVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename T>
            DMat<double> &operator()(T &node) const
            {
                return node.qdd_for_subtree_due_to_subtree_root_joint_qdd;
            }

            static DMat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(QddSubtreeVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DMat<double> &qddSubtree(NodeType &node)
        {
            return QddSubtreeVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct UpdateArticulatedBiasForceVisitor : public boost::static_visitor<>
        {
            template <typename T>
            void operator()(T &node) const
            {
                const DVec<double> Iv = node.I_ * node.v_;
                node.pA_ = generalForceCrossProduct(node.v_, Iv);
            }

            static void run(NodeType &node)
            {
                boost::apply_visitor(UpdateArticulatedBiasForceVisitor(), node);
            }
        };

        template <typename NodeType>
        inline void updateArticulatedBiasForce(NodeType &node)
        {
            UpdateArticulatedBiasForceVisitor<NodeType>::run(node);
        }

        // TODO(@MatthewChignoli): Should probably return a reference...
        // TODO(@MatthewChignoli): This should be templated on multiple things? Because the return type is different for different node types

        // TODO(@MatthewChignoli): I don't think we will be able to access the joint directly... that sucks
        // template <typename NodeType>
        // struct JointVisitor : public boost::static_visitor<auto>
        // {
        //     template <typename T>
        //     typename T::GenJointType operator()(T &node) const
        //     {
        //         return node.joint_;
        //     }

        //     static auto run(NodeType &node)
        //     {
        //         return boost::apply_visitor(JointVisitor(), node);
        //     }
        // };

        // template <typename NodeType>
        // inline auto getJoint(NodeType &node)
        // {
        //     return JointVisitor<NodeType>::run(node);
        // }

        template <typename NodeType>
        struct PositionIsSpanningVisitor : public boost::static_visitor<bool>
        {
            template <typename T>
            bool operator()(const T &node) const
            {
                return node.joint_.positionIsSpanning();
            }

            static bool run(const NodeType &node)
            {
                return boost::apply_visitor(PositionIsSpanningVisitor(), node);
            }
        };

        template <typename NodeType>
        inline bool positionIsSpanning(const NodeType &node)
        {
            return PositionIsSpanningVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct VelocityIsSpanningVisitor : public boost::static_visitor<bool>
        {
            template <typename T>
            bool operator()(const T &node) const
            {
                return node.joint_.velocityIsSpanning();
            }

            static bool run(const NodeType &node)
            {
                return boost::apply_visitor(VelocityIsSpanningVisitor(), node);
            }
        };

        template <typename NodeType>
        inline bool velocityIsSpanning(const NodeType &node)
        {
            return VelocityIsSpanningVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct CloneLoopConstraintVisitor
            : public boost::static_visitor<std::shared_ptr<LoopConstraint::Base>>
        {
            template <typename T>
            std::shared_ptr<LoopConstraint::Base> operator()(const T &node) const
            {
                return node.joint_.cloneLoopConstraint();
            }

            static std::shared_ptr<LoopConstraint::Base> run(const NodeType &node)
            {
                return boost::apply_visitor(CloneLoopConstraintVisitor(), node);
            }
        };

        template <typename NodeType>
        inline std::shared_ptr<LoopConstraint::Base> cloneLoopConstraint(const NodeType &node)
        {
            return CloneLoopConstraintVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct SpanningToIndependentVisitor : public boost::static_visitor<const DMat<double> &>
        {
            template <typename T>
            const DMat<double> &operator()(const T &node) const
            {
                return node.joint_.spanningTreeToIndependentCoordsConversion();
            }

            static const DMat<double> &run(const NodeType &node)
            {
                return boost::apply_visitor(SpanningToIndependentVisitor(), node);
            }
        };

        template <typename NodeType>
        inline const DMat<double> &spanningToIndependent(const NodeType &node)
        {
            return SpanningToIndependentVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct RandomJointStateVisitor : public boost::static_visitor<JointState>
        {
            template <typename T>
            JointState operator()(const T &node) const
            {
                return node.joint_.randomJointState();
            }

            static JointState run(const NodeType &node)
            {
                return boost::apply_visitor(RandomJointStateVisitor(), node);
            }
        };

        template <typename NodeType>
        inline JointState randomJointState(const NodeType &node)
        {
            return RandomJointStateVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ToSpanningTreStateVisitor : public boost::static_visitor<JointState>
        {
            const JointState &joint_state_;

            ToSpanningTreStateVisitor(const JointState &joint_state) : joint_state_(joint_state) {}

            template <typename T>
            JointState operator()(T &node) const
            {
                return node.joint_.toSpanningTreeState(joint_state_);
            }

            static JointState run(NodeType &node, const JointState &joint_state)
            {
                return boost::apply_visitor(ToSpanningTreStateVisitor(joint_state), node);
            }
        };

        template <typename NodeType>
        inline JointState toSpanningTreeState(NodeType &node, const JointState &joint_state)
        {
            return ToSpanningTreStateVisitor<NodeType>::run(node, joint_state);
        }

        template <typename NodeType>
        struct UpdateJointKinematicsVisitor : public boost::static_visitor<>
        {
            const JointState &joint_state_;

            UpdateJointKinematicsVisitor(const JointState &joint_state)
                : joint_state_(joint_state) {}

            template <typename T>
            void operator()(T &node) const
            {
                node.joint_.updateKinematics(joint_state_);
            }

            static void run(NodeType &node, const JointState &joint_state)
            {
                boost::apply_visitor(UpdateJointKinematicsVisitor(joint_state), node);
            }
        };

        template <typename NodeType>
        inline void updateJointKinematics(NodeType &node, const JointState &joint_state)
        {
            UpdateJointKinematicsVisitor<NodeType>::run(node, joint_state);
        }

        template <typename NodeType>
        struct GetJointTypeVisitor : public boost::static_visitor<GeneralizedJointTypes>
        {
            template <typename T>
            GeneralizedJointTypes operator()(const T &node) const
            {
                return node.joint_.type();
            }

            static GeneralizedJointTypes run(const NodeType &node)
            {
                return boost::apply_visitor(GetJointTypeVisitor(), node);
            }
        };

        template <typename NodeType>
        inline GeneralizedJointTypes getJointType(const NodeType &node)
        {
            return GetJointTypeVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ArticulatedInertiaVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename T>
            DMat<double> &operator()(T &node) const
            {
                return node.IA_;
            }

            static DMat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(ArticulatedInertiaVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DMat<double> &IA(NodeType &node)
        {
            return ArticulatedInertiaVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ArticulatedBiasForceVisitor : public boost::static_visitor<DVec<double> &>
        {
            template <typename T>
            DVec<double> &operator()(T &node) const
            {
                return node.pA_;
            }

            static DVec<double> &run(NodeType &node)
            {
                return boost::apply_visitor(ArticulatedBiasForceVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DVec<double> &pA(NodeType &node)
        {
            return ArticulatedBiasForceVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct UVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename T>
            DMat<double> &operator()(T &node) const
            {
                return node.U_;
            }

            static DMat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(UVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DMat<double> &U(NodeType &node)
        {
            return UVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct uVisitor : public boost::static_visitor<DVec<double> &>
        {
            template <typename T>
            DVec<double> &operator()(T &node) const
            {
                return node.u_;
            }

            static DVec<double> &run(NodeType &node)
            {
                return boost::apply_visitor(uVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DVec<double> &u(NodeType &node)
        {
            return uVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct D_invVisitor
            : public boost::static_visitor<Eigen::ColPivHouseholderQR<DMat<double>> &>
        {
            template <typename T>
            Eigen::ColPivHouseholderQR<DMat<double>> &operator()(T &node) const
            {
                return node.D_inv_;
            }

            static Eigen::ColPivHouseholderQR<DMat<double>> &run(NodeType &node)
            {
                return boost::apply_visitor(D_invVisitor(), node);
            }
        };

        template <typename NodeType>
        inline Eigen::ColPivHouseholderQR<DMat<double>> &D_inv(NodeType &node)
        {
            return D_invVisitor<NodeType>::run(node);
        }

        // TODO(@MatthewChignoli): Could probably do more efficient caching than this...
        template <typename NodeType>
        struct D_inv_uVisitor : public boost::static_visitor<DVec<double> &>
        {
            template <typename T>
            DVec<double> &operator()(T &node) const
            {
                return node.D_inv_u_;
            }

            static DVec<double> &run(NodeType &node)
            {
                return boost::apply_visitor(D_inv_uVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DVec<double> &D_inv_u(NodeType &node)
        {
            return D_inv_uVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct D_inv_UTVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename T>
            DMat<double> &operator()(T &node) const
            {
                return node.D_inv_UT_;
            }

            static DMat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(D_inv_UTVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DMat<double> &D_inv_UT(NodeType &node)
        {
            return D_inv_UTVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct IaVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename T>
            DMat<double> &operator()(T &node) const
            {
                return node.Ia_;
            }

            static DMat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(IaVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DMat<double> &Ia(NodeType &node)
        {
            return IaVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ArticulatedTransformVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename T>
            DMat<double> &operator()(T &node) const
            {
                return node.ChiUp_;
            }

            static DMat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(ArticulatedTransformVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DMat<double> &ChiUp(NodeType &node)
        {
            return ArticulatedTransformVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ResetArticulatedInertiaVisitor : public boost::static_visitor<>
        {
            template <typename T>
            void operator()(T &node) const
            {
                node.IA_ = node.I_;
            }

            static void run(NodeType &node)
            {
                boost::apply_visitor(ResetArticulatedInertiaVisitor(), node);
            }
        };

        template <typename NodeType>
        inline void resetArticulatedInertia(NodeType &node)
        {
            ResetArticulatedInertiaVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct UpdateDinvVisitor : public boost::static_visitor<>
        {
            const DMat<double> &D_;

            UpdateDinvVisitor(const DMat<double> &D) : D_(D) {}

            template <typename T>
            void operator()(T &node) const
            {
                node.updateDinv(D_);
            }

            static void run(NodeType &node, const DMat<double> &D)
            {
                boost::apply_visitor(UpdateDinvVisitor(D), node);
            }
        };

        template <typename NodeType>
        inline void updateDinv(NodeType &node, const DMat<double> &D)
        {
            UpdateDinvVisitor<NodeType>::run(node, D);
        }

        template <typename NodeType>
        struct BodiesAndJointsVisitor
            : public boost::static_visitor<std::vector<std::pair<Body, JointPtr>>>
        {
            template <typename T>
            std::vector<std::pair<Body, JointPtr>> operator()(T &node) const
            {
                return node.bodiesAndJoints();
            }

            static std::vector<std::pair<Body, JointPtr>> run(NodeType &node)
            {
                return boost::apply_visitor(BodiesAndJointsVisitor(), node);
            }
        };

        template <typename NodeType>
        inline std::vector<std::pair<Body, JointPtr>> bodiesAndJoints(NodeType &node)
        {
            return BodiesAndJointsVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct BodiesJointsAndReflectedInertiasVisitor
            : public boost::static_visitor<std::vector<std::tuple<Body, JointPtr, DMat<double>>>>
        {
            typedef std::tuple<Body, JointPtr, DMat<double>> BodyJointRefInertiaTuple;

            template <typename T>
            std::vector<BodyJointRefInertiaTuple> operator()(const T &node) const
            {
                return node.bodiesJointsAndReflectedInertias();
            }

            static std::vector<BodyJointRefInertiaTuple> run(const NodeType &node)
            {
                return boost::apply_visitor(BodiesJointsAndReflectedInertiasVisitor(), node);
            }
        };

        template <typename NodeType>
        inline std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        bodiesJointsAndReflectedInertias(const NodeType &node)
        {
            return BodiesJointsAndReflectedInertiasVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct IntegratePositionVisitor : public boost::static_visitor<JointCoordinate>
        {
            const JointState &joint_state_;
            const double &dt_;

            IntegratePositionVisitor(const JointState &joint_state, const double &dt)
                : joint_state_(joint_state), dt_(dt) {}

            template <typename T>
            JointCoordinate operator()(const T &node) const
            {
                return node.integratePosition(joint_state_, dt_);
            }

            static JointCoordinate run(const NodeType &node, JointState joint_state, double dt)
            {
                return boost::apply_visitor(IntegratePositionVisitor(joint_state, dt), node);
            }
        };

        template <typename NodeType>
        inline JointCoordinate integratePosition(const NodeType &node, JointState joint_state, double dt)
        {
            return IntegratePositionVisitor<NodeType>::run(node, joint_state, dt);
        }

    }

} // namespace grbda
