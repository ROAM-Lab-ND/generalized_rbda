#pragma once

#include <vector>
#include <boost/variant.hpp>

#include "Dynamics/Body.h"
#include "Dynamics/GeneralizedJoints/GeneralizedJointTypes.h"

// TODO(@MatthewChignoli): Breakout a cpp or hxx file somehow

namespace grbda
{
    // using namespace spatial;

    namespace ClusterNodeVisitors
    {

        template <typename NodeTypeVariants>
        struct BodiesVisitor : public boost::static_visitor<const std::vector<Body> &>
        {
            template <typename NodeType>
            const std::vector<Body> &operator()(const NodeType &node) const
            {
                return node.bodies();
            }

            static const std::vector<Body> &run(const NodeTypeVariants &node)
            {
                return boost::apply_visitor(BodiesVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline const std::vector<Body> &bodies(const NodeTypeVariants &node)
        {
            return BodiesVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct ContainsBodyVisitor : public boost::static_visitor<bool>
        {
            int body_index_;

            ContainsBodyVisitor(int body_index) : body_index_(body_index) {}

            template <typename NodeType>
            bool operator()(const NodeType &node) const
            {
                return node.containsBody(body_index_);
            }

            static bool run(const NodeTypeVariants &node, int body_index)
            {
                return boost::apply_visitor(ContainsBodyVisitor(body_index), node);
            }
        };

        template <typename NodeTypeVariants>
        inline bool containsBody(const NodeTypeVariants &node, int body_index)
        {
            return ContainsBodyVisitor<NodeTypeVariants>::run(node, body_index);
        }

        template <typename NodeTypeVariants>
        struct QddSubtreeVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename NodeType>
            DMat<double> &operator()(NodeType &node) const
            {
                return node.qdd_for_subtree_due_to_subtree_root_joint_qdd;
            }

            static DMat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(QddSubtreeVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DMat<double> &qddSubtree(NodeTypeVariants &node)
        {
            return QddSubtreeVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct UpdateArticulatedBiasForceVisitor : public boost::static_visitor<>
        {
            template <typename NodeType>
            void operator()(NodeType &node) const
            {
                const DVec<double> Iv = node.I_ * node.v_;
                node.pA_ = generalForceCrossProduct(node.v_, Iv);
            }

            static void run(NodeTypeVariants &node)
            {
                boost::apply_visitor(UpdateArticulatedBiasForceVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline void updateArticulatedBiasForce(NodeTypeVariants &node)
        {
            UpdateArticulatedBiasForceVisitor<NodeTypeVariants>::run(node);
        }

        // TODO(@MatthewChignoli): This should be templated on multiple things? Because the return type is different for different node types
        template <typename NodeTypeVariants>
        struct JointVisitor : public boost::static_visitor<std::shared_ptr<GeneralizedJoints::Base>>
        {
            template <typename NodeType>
            std::shared_ptr<GeneralizedJoints::Base> operator()(NodeType &node) const
            {
                return node.joint_;
            }

            static std::shared_ptr<GeneralizedJoints::Base> run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(JointVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline std::shared_ptr<GeneralizedJoints::Base> getJoint(NodeTypeVariants &node)
        {
            return JointVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct ArticulatedInertiaVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename NodeType>
            DMat<double> &operator()(NodeType &node) const
            {
                return node.IA_;
            }

            static DMat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(ArticulatedInertiaVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DMat<double> &IA(NodeTypeVariants &node)
        {
            return ArticulatedInertiaVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct ArticulatedBiasForceVisitor : public boost::static_visitor<DVec<double> &>
        {
            template <typename NodeType>
            DVec<double> &operator()(NodeType &node) const
            {
                return node.pA_;
            }

            static DVec<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(ArticulatedBiasForceVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DVec<double> &pA(NodeTypeVariants &node)
        {
            return ArticulatedBiasForceVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct UVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename NodeType>
            DMat<double> &operator()(NodeType &node) const
            {
                return node.U_;
            }

            static DMat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(UVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DMat<double> &U(NodeTypeVariants &node)
        {
            return UVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct uVisitor : public boost::static_visitor<DVec<double> &>
        {
            template <typename NodeType>
            DVec<double> &operator()(NodeType &node) const
            {
                return node.u_;
            }

            static DVec<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(uVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DVec<double> &u(NodeTypeVariants &node)
        {
            return uVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct D_invVisitor
            : public boost::static_visitor<Eigen::ColPivHouseholderQR<DMat<double>> &>
        {
            template <typename NodeType>
            Eigen::ColPivHouseholderQR<DMat<double>> &operator()(NodeType &node) const
            {
                return node.D_inv_;
            }

            static Eigen::ColPivHouseholderQR<DMat<double>> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(D_invVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline Eigen::ColPivHouseholderQR<DMat<double>> &D_inv(NodeTypeVariants &node)
        {
            return D_invVisitor<NodeTypeVariants>::run(node);
        }

        // TODO(@MatthewChignoli): Could probably do more efficient caching than this...
        template <typename NodeTypeVariants>
        struct D_inv_uVisitor : public boost::static_visitor<DVec<double> &>
        {
            template <typename NodeType>
            DVec<double> &operator()(NodeType &node) const
            {
                return node.D_inv_u_;
            }

            static DVec<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(D_inv_uVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DVec<double> &D_inv_u(NodeTypeVariants &node)
        {
            return D_inv_uVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct D_inv_UTVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename NodeType>
            DMat<double> &operator()(NodeType &node) const
            {
                return node.D_inv_UT_;
            }

            static DMat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(D_inv_UTVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DMat<double> &D_inv_UT(NodeTypeVariants &node)
        {
            return D_inv_UTVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct IaVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename NodeType>
            DMat<double> &operator()(NodeType &node) const
            {
                return node.Ia_;
            }

            static DMat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(IaVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DMat<double> &Ia(NodeTypeVariants &node)
        {
            return IaVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct ArticulatedTransformVisitor : public boost::static_visitor<DMat<double> &>
        {
            template <typename NodeType>
            DMat<double> &operator()(NodeType &node) const
            {
                return node.ChiUp_;
            }

            static DMat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(ArticulatedTransformVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DMat<double> &ChiUp(NodeTypeVariants &node)
        {
            return ArticulatedTransformVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct ResetArticulatedInertiaVisitor : public boost::static_visitor<>
        {
            template <typename NodeType>
            void operator()(NodeType &node) const
            {
                node.IA_= node.I_;
            }

            static void run(NodeTypeVariants &node)
            {
                boost::apply_visitor(ResetArticulatedInertiaVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline void resetArticulatedInertia(NodeTypeVariants &node)
        {
            ResetArticulatedInertiaVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct UpdateDinvVisitor : public boost::static_visitor<>
        {
            const DMat<double>& D_;

            UpdateDinvVisitor(const DMat<double>& D) : D_(D) {}

            template <typename NodeType>
            void operator()(NodeType &node) const
            {
                node.updateDinv(D_);
            }

            static void run(NodeTypeVariants &node, const DMat<double>& D)
            {
                boost::apply_visitor(UpdateDinvVisitor(D), node);
            }
        };

        template <typename NodeTypeVariants>
        inline void updateDinv(NodeTypeVariants &node, const DMat<double>& D)
        {
            UpdateDinvVisitor<NodeTypeVariants>::run(node, D);
        }

        template <typename NodeTypeVariants>
        struct BodiesAndJointsVisitor
            : public boost::static_visitor<std::vector<std::pair<Body, JointPtr>>>
        {
            template <typename NodeType>
            std::vector<std::pair<Body, JointPtr>> operator()(NodeType &node) const
            {
                return node.bodiesAndJoints();
            }

            static std::vector<std::pair<Body, JointPtr>> run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(BodiesAndJointsVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline std::vector<std::pair<Body, JointPtr>> bodiesAndJoints(NodeTypeVariants &node)
        {
            return BodiesAndJointsVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct BodiesJointsAndReflectedInertiasVisitor
            : public boost::static_visitor<std::vector<std::tuple<Body, JointPtr, DMat<double>>>>
        {
            typedef std::tuple<Body, JointPtr, DMat<double>> BodyJointRefInertiaTuple;

            template <typename NodeType>
            std::vector<BodyJointRefInertiaTuple> operator()(const NodeType &node) const
            {
                return node.bodiesJointsAndReflectedInertias();
            }

            static std::vector<BodyJointRefInertiaTuple> run(const NodeTypeVariants &node)
            {
                return boost::apply_visitor(BodiesJointsAndReflectedInertiasVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline std::vector<std::tuple<Body, JointPtr, DMat<double>>>
        bodiesJointsAndReflectedInertias(const NodeTypeVariants &node)
        {
            return BodiesJointsAndReflectedInertiasVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct IntegratePositionVisitor : public boost::static_visitor<JointCoordinate>
        {
            const JointState& joint_state_;
            const double& dt_;

            IntegratePositionVisitor(const JointState &joint_state, const double &dt)
                : joint_state_(joint_state), dt_(dt) {}

            template <typename NodeType>
            JointCoordinate operator()(const NodeType &node) const
            {
                return node.integratePosition(joint_state_, dt_);
            }

            static JointCoordinate run(const NodeTypeVariants &node, JointState joint_state, double dt)
            {
                return boost::apply_visitor(IntegratePositionVisitor(joint_state, dt), node);
            }
        };

        template <typename NodeTypeVariants>
        inline JointCoordinate integratePosition(const NodeTypeVariants &node, JointState joint_state, double dt)
        {
            return IntegratePositionVisitor<NodeTypeVariants>::run(node, joint_state, dt);
        }


    }

} // namespace grbda
