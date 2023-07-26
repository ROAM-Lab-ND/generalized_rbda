#pragma once

#include <boost/variant.hpp>
#include "Utils/Utilities/SpatialTransforms.h"

// TODO(@MatthewChignoli): Breakout a cpp or hxx file somehow

// TODO(@MatthewChignoli): Feels like there is a lot of copying going on in this file. Can we return references instead? Or do we need to have some data structure like pinocchio?

namespace grbda
{
    using namespace spatial;

    template <typename NodeTypeVariants>
    struct IndexVisitor : public boost::static_visitor<int>
    {
        template <typename NodeType>
        int operator()(const NodeType &node) const
        {
            return node.index_;
        }

        static int run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(IndexVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline int index(const NodeTypeVariants &node)
    {
        return IndexVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct ParentIndexVisitor : public boost::static_visitor<int>
    {
        template <typename NodeType>
        int operator()(const NodeType &node) const
        {
            return node.parent_index_;
        }

        static int run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(ParentIndexVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline int parentIndex(const NodeTypeVariants &node)
    {
        return ParentIndexVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct VelocityVisitor : public boost::static_visitor<const DVec<double> &>
    {
        template <typename NodeType>
        const DVec<double> &operator()(const NodeType &node) const
        {
            return node.v_;
        }

        static const DVec<double> &run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(VelocityVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline const DVec<double> &velocity(const NodeTypeVariants &node)
    {
        return VelocityVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct XaVisitor : public boost::static_visitor<const GeneralizedAbsoluteSpatialTransform &>
    {
        template <typename NodeType>
        const GeneralizedAbsoluteSpatialTransform &operator()(const NodeType &node) const
        {
            return node.Xa_;
        }

        static const GeneralizedAbsoluteSpatialTransform& run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(XaVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline const GeneralizedAbsoluteSpatialTransform& Xa(const NodeTypeVariants &node)
    {
        return XaVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct SetJointStateVisitor : public boost::static_visitor<>
    {
        JointState joint_state_;

        SetJointStateVisitor(const JointState &joint_state) : joint_state_(joint_state) {}

        template <typename NodeType>
        void operator()(NodeType &node) const
        {
            node.joint_state_ = joint_state_;
        }

        static void run(NodeTypeVariants &node, const JointState &joint_state)
        {
            boost::apply_visitor(SetJointStateVisitor(joint_state), node);
        }
    };

    template <typename NodeTypeVariants>
    inline void setJointState(NodeTypeVariants &node, JointState joint_state)
    {
        SetJointStateVisitor<NodeTypeVariants>::run(node, joint_state);
    }

    template <typename NodeTypeVariants>
    struct GetJointStateVisitor : public boost::static_visitor<const JointState>
    {
        template <typename NodeType>
        const JointState operator()(const NodeType &node) const
        {
            return node.joint_state_;
        }

        static const JointState run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(GetJointStateVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline const JointState getJointState(const NodeTypeVariants &node)
    {
        return GetJointStateVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct ForwardKinematicsVisitor : public boost::static_visitor<>
    {
        // TODO(@MatthewChignoli): Pinocchio does something with fusion and algo...
        const DVec<double> &v_parent_;
        const GeneralizedAbsoluteSpatialTransform &Xa_parent_;

        ForwardKinematicsVisitor(const DVec<double> &v_parent,
                                 const GeneralizedAbsoluteSpatialTransform &Xa_parent)
            : v_parent_(v_parent), Xa_parent_(Xa_parent) {}

        template <typename NodeType>
        void operator()(NodeType &node) const
        {
            node.updateKinematics();
            node.v_ = node.Xup_.transformMotionVector(v_parent_) + node.vJ();
            node.Xa_ = node.Xup_ * Xa_parent_;
            node.c_ = node.S_ring() * node.joint_state_.velocity +
                      generalMotionCrossProduct(node.v_, node.vJ());
        }

        static void run(NodeTypeVariants &node, const DVec<double> &v_parent,
                        const GeneralizedAbsoluteSpatialTransform &Xa_parent)
        {
            boost::apply_visitor(ForwardKinematicsVisitor(v_parent, Xa_parent), node);
        }
    };

    template <typename NodeTypeVariants>
    inline void nodeKinematics(NodeTypeVariants &node, const DVec<double> &v_parent,
                               const GeneralizedAbsoluteSpatialTransform &Xa_parent)
    {
        ForwardKinematicsVisitor<NodeTypeVariants>::run(node, v_parent, Xa_parent);
    }

    template <typename NodeTypeVariants>
    struct ForwardKinematicsNoParentVisitor : public boost::static_visitor<>
    {
        template <typename NodeType>
        void operator()(NodeType &node) const
        {
            node.updateKinematics();
            node.v_ = node.vJ();
            node.Xa_ = node.Xup_.toAbsolute();
            node.c_ = node.S_ring() * node.joint_state_.velocity +
                      generalMotionCrossProduct(node.v_, node.vJ());
        }

        static void run(NodeTypeVariants &node)
        {
            boost::apply_visitor(ForwardKinematicsNoParentVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline void nodeKinematics(NodeTypeVariants &node)
    {
        ForwardKinematicsNoParentVisitor<NodeTypeVariants>::run(node);
    }

    // TODO(@MatthewChignoli): Are these totally necessary?
    template <typename NodeTypeVariants>
    struct GetAbsoluteTransformForBodyVisitor : public boost::static_visitor<SpatialTransform>
    {
        const Body &body_;

        GetAbsoluteTransformForBodyVisitor(const Body &body) : body_(body) {}

        template <typename NodeType>
        SpatialTransform operator()(const NodeType &node) const
        {
            return node.getAbsoluteTransformForBody(body_);
        }

        static SpatialTransform run(const NodeTypeVariants &node, const Body &body)
        {
            return boost::apply_visitor(GetAbsoluteTransformForBodyVisitor(body), node);
        }
    };

    template <typename NodeTypeVariants>
    inline SpatialTransform getAbsoluteTransformForBody(NodeTypeVariants &node, const Body &body)
    {
        return GetAbsoluteTransformForBodyVisitor<NodeTypeVariants>::run(node, body);
    }

    template <typename NodeTypeVariants>
    struct GetVelocityForBodyVisitor : public boost::static_visitor<DVec<double>>
    {
        const Body &body_;

        GetVelocityForBodyVisitor(const Body &body) : body_(body) {}

        template <typename NodeType>
        DVec<double> operator()(const NodeType &node) const
        {
            return node.getVelocityForBody(body_);
        }

        static DVec<double> run(const NodeTypeVariants &node, const Body &body)
        {
            return boost::apply_visitor(GetVelocityForBodyVisitor(body), node);
        }
    };

    template <typename NodeTypeVariants>
    inline DVec<double> getVelocityForBody(NodeTypeVariants &node, const Body &body)
    {
        return GetVelocityForBodyVisitor<NodeTypeVariants>::run(node, body);
    }

    template <typename NodeTypeVariants>
    struct ApplyForceToBodyVisitor : public boost::static_visitor<>
    {
        const SVec<double> &force_;
        const Body &body_;

        ApplyForceToBodyVisitor(const SVec<double> &force, const Body &body)
            : force_(force), body_(body) {}

        template <typename NodeType>
        void operator()(NodeType &node) const
        {
            node.applyForceToBody(force_, body_);
        }

        static void run(NodeTypeVariants &node, const SVec<double> &force, const Body &body)
        {
            boost::apply_visitor(ApplyForceToBodyVisitor(force, body), node);
        }
    };

    template <typename NodeTypeVariants>    
    inline void applyForceToBody(NodeTypeVariants &node, const SVec<double> &force, const Body &body)
    {
        ApplyForceToBodyVisitor<NodeTypeVariants>::run(node, force, body);
    }

    template <typename NodeTypeVariants>
    struct ResetCompositeRigidBodyInertiaVisitor : public boost::static_visitor<>
    {
        template <typename NodeType>
        void operator()(NodeType &node) const
        {
            node.Ic_ = node.I_;
        }

        static void run(NodeTypeVariants &node)
        {
            boost::apply_visitor(ResetCompositeRigidBodyInertiaVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline void resetCompositeRigidBodyInertia(NodeTypeVariants &node)
    {
        ResetCompositeRigidBodyInertiaVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct VelIndexVisitor : public boost::static_visitor<int>
    {
        template <typename NodeType>
        int operator()(const NodeType &node) const
        {
            return node.velocity_index_;
        }

        static int run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(VelIndexVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline int velocityIndex(NodeTypeVariants &node)
    {
        return VelIndexVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct NumVelocitiesVisitor : public boost::static_visitor<int>
    {
        template <typename NodeType>
        int operator()(const NodeType &node) const
        {
            return node.num_velocities_;
        }

        static int run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(NumVelocitiesVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline int numVelocities(NodeTypeVariants &node)
    {
        return NumVelocitiesVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct CompositeInertiaVisitor : public boost::static_visitor<DMat<double> &>
    {
        template <typename NodeType>
        DMat<double> &operator()(NodeType &node) const
        {
            return node.Ic_;
        }

        static DMat<double> &run(NodeTypeVariants &node)
        {
            return boost::apply_visitor(CompositeInertiaVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    struct MotionSubspaceDimVisitor : public boost::static_visitor<int>
    {
        template <typename NodeType>
        int operator()(const NodeType &node) const
        {
            return node.motion_subspace_dimension_;
        }

        static int run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(MotionSubspaceDimVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline int motionSubspaceDimension(const NodeTypeVariants &node)
    {
        return MotionSubspaceDimVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    inline DMat<double> &compositeInertia(NodeTypeVariants &node)
    {
        return CompositeInertiaVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct MotionSubspaceVisitor : public boost::static_visitor<const DMat<double> &>
    {
        template <typename NodeType>
        const DMat<double> &operator()(NodeType &node) const
        {
            return node.S();
        }

        static const DMat<double> &run(NodeTypeVariants &node)
        {
            return boost::apply_visitor(MotionSubspaceVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline const DMat<double> &motionSubspace(NodeTypeVariants &node)
    {
        return MotionSubspaceVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct XupVisitor : public boost::static_visitor<const GeneralizedSpatialTransform &>
    {
        template <typename NodeType>
        const GeneralizedSpatialTransform &operator()(const NodeType &node) const
        {
            return node.Xup_;
        }

        static const GeneralizedSpatialTransform &run(const NodeTypeVariants &node)
        {
            return boost::apply_visitor(XupVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline const GeneralizedSpatialTransform &Xup(NodeTypeVariants &node)
    {
        return XupVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct AccelerationVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename NodeType>
        DVec<double> &operator()(NodeType &node) const
        {
            return node.a_;
        }

        static DVec<double> &run(NodeTypeVariants &node)
        {
            return boost::apply_visitor(AccelerationVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline DVec<double> &acceleration(NodeTypeVariants &node)
    {
        return AccelerationVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct VelocityProductVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename NodeType>
        DVec<double> &operator()(NodeType &node) const
        {
            return node.c_;
        }

        static DVec<double> &run(NodeTypeVariants &node)
        {
            return boost::apply_visitor(VelocityProductVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline DVec<double> &velocityProduct(NodeTypeVariants &node)
    {
        return VelocityProductVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct SetForceFromAccelerationVisitor : public boost::static_visitor<>
    {
        template <typename NodeType>
        void operator()(NodeType &node) const
        {
            const DVec<double> Iv = node.I_ * node.v_;
            node.f_ = node.I_ * node.a_ + generalForceCrossProduct(node.v_, Iv);
        }

        static void run(NodeTypeVariants &node)
        {
            boost::apply_visitor(SetForceFromAccelerationVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline void setForceFromAcceleration(NodeTypeVariants &node)
    {
        SetForceFromAccelerationVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct NetForceVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename NodeType>
        DVec<double> &operator()(NodeType &node) const
        {
            return node.f_;
        }

        static DVec<double> &run(NodeTypeVariants &node)
        {
            return boost::apply_visitor(NetForceVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline DVec<double> &netForce(NodeTypeVariants &node)
    {
        return NetForceVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct NameVisitor : public boost::static_visitor<const string &>
    {
        template <typename NodeType>
        const string &operator()(NodeType &node) const
        {
            return node.name_;
        }

        static const string &run(NodeTypeVariants &node)
        {
            return boost::apply_visitor(NameVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline const string &name(NodeTypeVariants &node)
    {
        return NameVisitor<NodeTypeVariants>::run(node);
    }

    template <typename NodeTypeVariants>
    struct ExternalForceVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename NodeType>
        DVec<double> &operator()(NodeType &node) const
        {
            return node.f_ext_;
        }

        static DVec<double> &run(NodeTypeVariants &node)
        {
            return boost::apply_visitor(ExternalForceVisitor(), node);
        }
    };

    template <typename NodeTypeVariants>
    inline DVec<double> &externalForce(NodeTypeVariants &node)
    {
        return ExternalForceVisitor<NodeTypeVariants>::run(node);
    }

}
