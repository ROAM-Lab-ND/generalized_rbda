#pragma once

#include <boost/variant.hpp>
#include "Dynamics/Body.h"

namespace grbda
{
    namespace ReflectedInertiaNodeVisitors
    {

        template <typename NodeType>
        struct GetLinkVisitor : public boost::static_visitor<const Body &>
        {
            template <typename T>
            const Body &operator()(const T &node) const
            {
                return node.link_;
            }

            static const Body &run(const NodeType &node)
            {
                return boost::apply_visitor(GetLinkVisitor(), node);
            }
        };

        template <typename NodeType>
        inline const Body &getLink(const NodeType &node)
        {
            return GetLinkVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct GetJointVisitor : public boost::static_visitor<std::shared_ptr<Joints::Base>>
        {
            template <typename T>
            std::shared_ptr<Joints::Base> operator()(const T &node) const
            {
                return node.joint_;
            }

            static std::shared_ptr<Joints::Base> run(const NodeType &node)
            {
                return boost::apply_visitor(GetJointVisitor(), node);
            }
        };

        template <typename NodeType>
        inline std::shared_ptr<Joints::Base> getJoint(const NodeType &node)
        {
            return GetJointVisitor<NodeType>::run(node);
        }

        // TODO(@MatthewChignoli): Some of this stuff is shared with ClusterTreeNodeVisitors and RigidsTreeNodeVisitors... how to make common and reduce redundancy?
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

        template <typename NodeType>
        struct ArticulatedInertiaVisitor : public boost::static_visitor<Mat6<double> &>
        {
            template <typename T>
            Mat6<double> &operator()(T &node) const
            {
                return node.IA_;
            }

            static Mat6<double> &run(NodeType &node)
            {
                return boost::apply_visitor(ArticulatedInertiaVisitor(), node);
            }
        };

        template <typename NodeType>
        inline Mat6<double> &IA(NodeType &node)
        {
            return ArticulatedInertiaVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ArticulatedBiasForceVisitor : public boost::static_visitor<SVec<double> &>
        {
            template <typename T>
            SVec<double> &operator()(T &node) const
            {
                return node.pA_;
            }

            static SVec<double> &run(NodeType &node)
            {
                return boost::apply_visitor(ArticulatedBiasForceVisitor(), node);
            }
        };

        template <typename NodeType>
        inline SVec<double> &pA(NodeType &node)
        {
            return ArticulatedBiasForceVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct UVisitor : public boost::static_visitor<D6Mat<double> &>
        {
            template <typename T>
            D6Mat<double> &operator()(T &node) const
            {
                return node.U_;
            }

            static D6Mat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(UVisitor(), node);
            }
        };

        template <typename NodeType>
        inline D6Mat<double> &U(NodeType &node)
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
            : public boost::static_visitor<DMat<double> &>
        {
            template <typename T>
            DMat<double> &operator()(T &node) const
            {
                return node.D_inv_;
            }

            static DMat<double> &run(NodeType &node)
            {
                return boost::apply_visitor(D_invVisitor(), node);
            }
        };

        template <typename NodeType>
        inline DMat<double> &D_inv(NodeType &node)
        {
            return D_invVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct ResetArticulatedInertiaVisitor : public boost::static_visitor<>
        {
            template <typename T>
            void operator()(T &node) const
            {
                node.IA_= node.I_;
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

    } // namespace ReflectedInertiaNodeVisitors

} // namespace grbda
