#pragma once

#include <boost/variant.hpp>
#include "Dynamics/Joints/Joint.h"

namespace grbda
{
    namespace RigidBodyNodeVisitors
    {

        template <typename NodeTypeVariants>
        struct GetJointVisitor : public boost::static_visitor<std::shared_ptr<Joints::Base>>
        {
            template <typename NodeType>
            std::shared_ptr<Joints::Base> operator()(NodeType &node) const
            {
                return node.joint_;
            }

            static std::shared_ptr<Joints::Base> run(NodeTypeVariants &node)
            {
                return boost::apply_visitor(GetJointVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline std::shared_ptr<Joints::Base> getJoint(NodeTypeVariants &node)
        {
            return GetJointVisitor<NodeTypeVariants>::run(node);
        }

        template <typename NodeTypeVariants>
        struct GetBodyVisitor : public boost::static_visitor<const Body &>
        {
            template <typename NodeType>
            const Body &operator()(const NodeType &node) const
            {
                return node.body_;
            }

            static const Body &run(const NodeTypeVariants &node)
            {
                return boost::apply_visitor(GetBodyVisitor(), node);
            }
        };

        template <typename NodeTypeVariants>
        inline const Body &getBody(const NodeTypeVariants &node)
        {
            return GetBodyVisitor<NodeTypeVariants>::run(node);
        }

    } // namespace RigidBodyNodeVisitors

} // namespace grbda
