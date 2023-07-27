#pragma once

#include <boost/variant.hpp>
#include "Dynamics/Joints/Joint.h"

namespace grbda
{
    namespace RigidBodyNodeVisitors
    {

        template <typename NodeType>
        struct GetJointVisitor : public boost::static_visitor<std::shared_ptr<Joints::Base>>
        {
            template <typename T>
            std::shared_ptr<Joints::Base> operator()(T &node) const
            {
                return node.joint_;
            }

            static std::shared_ptr<Joints::Base> run(NodeType &node)
            {
                return boost::apply_visitor(GetJointVisitor(), node);
            }
        };

        template <typename NodeType>
        inline std::shared_ptr<Joints::Base> getJoint(NodeType &node)
        {
            return GetJointVisitor<NodeType>::run(node);
        }

        template <typename NodeType>
        struct GetBodyVisitor : public boost::static_visitor<const Body &>
        {
            template <typename T>
            const Body &operator()(const T &node) const
            {
                return node.body_;
            }

            static const Body &run(const NodeType &node)
            {
                return boost::apply_visitor(GetBodyVisitor(), node);
            }
        };

        template <typename NodeType>
        inline const Body &getBody(const NodeType &node)
        {
            return GetBodyVisitor<NodeType>::run(node);
        }

    } // namespace RigidBodyNodeVisitors

} // namespace grbda
