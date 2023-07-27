#pragma once

#include <boost/variant.hpp>
#include "Dynamics/Body.h"

namespace grbda
{
    namespace ReflectedInertiaNodeVisitors
    {

        template <typename NodeTypeVariants>
        struct GetLinkVisitor : public boost::static_visitor<const Body &>
        {
            template <typename NodeType>
            const Body &operator()(const NodeType &node) const
            {
                return node.link_;
            }

            static const Body &run(const NodeTypeVariants &node)
            {
                return boost::apply_visitor(GetLinkVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline const Body &getLink(const NodeTypeVariants &node)
        {
            return GetLinkVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct GetJointVisitor : public boost::static_visitor<std::shared_ptr<Joints::Base>>
        {
            template <typename NodeType>
            std::shared_ptr<Joints::Base> operator()(const NodeType &node) const
            {
                return node.joint_;
            }

            static std::shared_ptr<Joints::Base> run(const NodeTypeVariants &node)
            {
                return boost::apply_visitor(GetJointVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline std::shared_ptr<Joints::Base> getJoint(const NodeTypeVariants &node)
        {
            return GetJointVisitor<NodeTypeVariants>::run(node);
        }

        // TODO(@MatthewChignoli): Some of this stuff is shared with ClusterTreeNodeVisitors and RigidsTreeNodeVisitors... how to make common and reduce redundancy?
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

        template <typename NodeTypeVariants>
        struct ArticulatedInertiaVisitor : public boost::static_visitor<Mat6<double> &>
        {
            template <typename NodeType>
            Mat6<double> &operator()(NodeType &node) const
            {
                return node.IA_;
            }

            static Mat6<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(ArticulatedInertiaVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline Mat6<double> &IA(NodeTypeVariants &node)
        {
            return ArticulatedInertiaVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct ArticulatedBiasForceVisitor : public boost::static_visitor<SVec<double> &>
        {
            template <typename NodeType>
            SVec<double> &operator()(NodeType &node) const
            {
                return node.pA_;
            }

            static SVec<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(ArticulatedBiasForceVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline SVec<double> &pA(NodeTypeVariants &node)
        {
            return ArticulatedBiasForceVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct UVisitor : public boost::static_visitor<D6Mat<double> &>
        {
            template <typename NodeType>
            D6Mat<double> &operator()(NodeType &node) const
            {
                return node.U_;
            }

            static D6Mat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(UVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline D6Mat<double> &U(NodeTypeVariants &node)
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
            : public boost::static_visitor<DMat<double> &>
        {
            template <typename NodeType>
            DMat<double> &operator()(NodeType &node) const
            {
                return node.D_inv_;
            }

            static DMat<double> &run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(D_invVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline DMat<double> &D_inv(NodeTypeVariants &node)
        {
            return D_invVisitor<NodeTypeVariants>::run(node);
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

    } // namespace ReflectedInertiaNodeVisitors

} // namespace grbda
