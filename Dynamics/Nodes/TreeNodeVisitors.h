#pragma once

#include <boost/variant.hpp>
#include "Utils/Utilities/SpatialTransforms.h"

// TODO(@MatthewChignoli): Breakout a cpp or hxx file somehow

// TODO(@MatthewChignoli): Feels like there is a lot of copying going on in this file. Can we return references instead? Or do we need to have some data structure like pinocchio?

namespace grbda
{
    using namespace spatial;

    template <typename NodeType>
    struct IndexVisitor : public boost::static_visitor<int>
    {
        template <typename T>
        int operator()(const T &node) const
        {
            return node.index_;
        }

        static int run(const NodeType &node)
        {
            return boost::apply_visitor(IndexVisitor(), node);
        }
    };

    template <typename NodeType>
    inline int index(const NodeType &node)
    {
        return IndexVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct ParentIndexVisitor : public boost::static_visitor<int>
    {
        template <typename T>
        int operator()(const T &node) const
        {
            return node.parent_index_;
        }

        static int run(const NodeType &node)
        {
            return boost::apply_visitor(ParentIndexVisitor(), node);
        }
    };

    template <typename NodeType>
    inline int parentIndex(const NodeType &node)
    {
        return ParentIndexVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct VelocityVisitor : public boost::static_visitor<const DVec<double> &>
    {
        template <typename T>
        const DVec<double> &operator()(const T &node) const
        {
            return node.v_;
        }

        static const DVec<double> &run(const NodeType &node)
        {
            return boost::apply_visitor(VelocityVisitor(), node);
        }
    };

    template <typename NodeType>
    inline const DVec<double> &velocity(const NodeType &node)
    {
        return VelocityVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct XaVisitor : public boost::static_visitor<const GeneralizedAbsoluteSpatialTransform &>
    {
        template <typename T>
        const GeneralizedAbsoluteSpatialTransform &operator()(const T &node) const
        {
            return node.Xa_;
        }

        static const GeneralizedAbsoluteSpatialTransform& run(const NodeType &node)
        {
            return boost::apply_visitor(XaVisitor(), node);
        }
    };

    template <typename NodeType>
    inline const GeneralizedAbsoluteSpatialTransform& Xa(const NodeType &node)
    {
        return XaVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct SetJointStateVisitor : public boost::static_visitor<>
    {
        JointState joint_state_;

        SetJointStateVisitor(const JointState &joint_state) : joint_state_(joint_state) {}

        template <typename T>
        void operator()(T &node) const
        {
            node.joint_state_ = joint_state_;
        }

        static void run(NodeType &node, const JointState &joint_state)
        {
            boost::apply_visitor(SetJointStateVisitor(joint_state), node);
        }
    };

    template <typename NodeType>
    inline void setJointState(NodeType &node, JointState joint_state)
    {
        SetJointStateVisitor<NodeType>::run(node, joint_state);
    }

    template <typename NodeType>
    struct GetJointStateVisitor : public boost::static_visitor<const JointState>
    {
        template <typename T>
        const JointState operator()(const T &node) const
        {
            return node.joint_state_;
        }

        static const JointState run(const NodeType &node)
        {
            return boost::apply_visitor(GetJointStateVisitor(), node);
        }
    };

    template <typename NodeType>
    inline const JointState getJointState(const NodeType &node)
    {
        return GetJointStateVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct ForwardKinematicsVisitor : public boost::static_visitor<>
    {
        // TODO(@MatthewChignoli): Pinocchio does something with fusion and algo...
        const DVec<double> &v_parent_;
        const GeneralizedAbsoluteSpatialTransform &Xa_parent_;

        ForwardKinematicsVisitor(const DVec<double> &v_parent,
                                 const GeneralizedAbsoluteSpatialTransform &Xa_parent)
            : v_parent_(v_parent), Xa_parent_(Xa_parent) {}

        template <typename T>
        void operator()(T &node) const
        {
            node.updateKinematics();
            node.v_ = node.Xup_.transformMotionVector(v_parent_) + node.vJ();
            node.Xa_ = node.Xup_ * Xa_parent_;
            node.c_ = node.S_ring() * node.joint_state_.velocity +
                      generalMotionCrossProduct(node.v_, node.vJ());
        }

        static void run(NodeType &node, const DVec<double> &v_parent,
                        const GeneralizedAbsoluteSpatialTransform &Xa_parent)
        {
            boost::apply_visitor(ForwardKinematicsVisitor(v_parent, Xa_parent), node);
        }
    };

    template <typename NodeType>
    inline void nodeKinematics(NodeType &node, const DVec<double> &v_parent,
                               const GeneralizedAbsoluteSpatialTransform &Xa_parent)
    {
        ForwardKinematicsVisitor<NodeType>::run(node, v_parent, Xa_parent);
    }

    template <typename NodeType>
    struct ForwardKinematicsNoParentVisitor : public boost::static_visitor<>
    {
        template <typename T>
        void operator()(T &node) const
        {
            node.updateKinematics();
            node.v_ = node.vJ();
            node.Xa_ = node.Xup_.toAbsolute();
            node.c_ = node.S_ring() * node.joint_state_.velocity +
                      generalMotionCrossProduct(node.v_, node.vJ());
        }

        static void run(NodeType &node)
        {
            boost::apply_visitor(ForwardKinematicsNoParentVisitor(), node);
        }
    };

    template <typename NodeType>
    inline void nodeKinematics(NodeType &node)
    {
        ForwardKinematicsNoParentVisitor<NodeType>::run(node);
    }

    // TODO(@MatthewChignoli): Are these totally necessary?
    template <typename NodeType>
    struct GetAbsoluteTransformForBodyVisitor : public boost::static_visitor<SpatialTransform>
    {
        const Body &body_;

        GetAbsoluteTransformForBodyVisitor(const Body &body) : body_(body) {}

        template <typename T>
        SpatialTransform operator()(const T &node) const
        {
            return node.getAbsoluteTransformForBody(body_);
        }

        static SpatialTransform run(const NodeType &node, const Body &body)
        {
            return boost::apply_visitor(GetAbsoluteTransformForBodyVisitor(body), node);
        }
    };

    template <typename NodeType>
    inline SpatialTransform getAbsoluteTransformForBody(NodeType &node, const Body &body)
    {
        return GetAbsoluteTransformForBodyVisitor<NodeType>::run(node, body);
    }

    template <typename NodeType>
    struct GetVelocityForBodyVisitor : public boost::static_visitor<DVec<double>>
    {
        const Body &body_;

        GetVelocityForBodyVisitor(const Body &body) : body_(body) {}

        template <typename T>
        DVec<double> operator()(const T &node) const
        {
            return node.getVelocityForBody(body_);
        }

        static DVec<double> run(const NodeType &node, const Body &body)
        {
            return boost::apply_visitor(GetVelocityForBodyVisitor(body), node);
        }
    };

    template <typename NodeType>
    inline DVec<double> getVelocityForBody(NodeType &node, const Body &body)
    {
        return GetVelocityForBodyVisitor<NodeType>::run(node, body);
    }

    template <typename NodeType>
    struct ApplyForceToBodyVisitor : public boost::static_visitor<>
    {
        const SVec<double> &force_;
        const Body &body_;

        ApplyForceToBodyVisitor(const SVec<double> &force, const Body &body)
            : force_(force), body_(body) {}

        template <typename T>
        void operator()(T &node) const
        {
            node.applyForceToBody(force_, body_);
        }

        static void run(NodeType &node, const SVec<double> &force, const Body &body)
        {
            boost::apply_visitor(ApplyForceToBodyVisitor(force, body), node);
        }
    };

    template <typename NodeType>    
    inline void applyForceToBody(NodeType &node, const SVec<double> &force, const Body &body)
    {
        ApplyForceToBodyVisitor<NodeType>::run(node, force, body);
    }

    template <typename NodeType>
    struct ResetCompositeRigidBodyInertiaVisitor : public boost::static_visitor<>
    {
        template <typename T>
        void operator()(T &node) const
        {
            node.Ic_ = node.I_;
        }

        static void run(NodeType &node)
        {
            boost::apply_visitor(ResetCompositeRigidBodyInertiaVisitor(), node);
        }
    };

    template <typename NodeType>
    inline void resetCompositeRigidBodyInertia(NodeType &node)
    {
        ResetCompositeRigidBodyInertiaVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct PosIndexVisitor : public boost::static_visitor<int>
    {
        template <typename T>
        int operator()(const T &node) const
        {
            return node.position_index_;
        }

        static int run(const NodeType &node)
        {
            return boost::apply_visitor(PosIndexVisitor(), node);
        }
    };

    template <typename NodeType>
    inline int positionIndex(NodeType &node)
    {
        return PosIndexVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct NumPositionsVisitor : public boost::static_visitor<int>
    {
        template <typename T>
        int operator()(const T &node) const
        {
            return node.num_positions_;
        }

        static int run(const NodeType &node)
        {
            return boost::apply_visitor(NumPositionsVisitor(), node);
        }
    };

    template <typename NodeType>
    inline int numPositions(NodeType &node)
    {
        return NumPositionsVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct VelIndexVisitor : public boost::static_visitor<int>
    {
        template <typename T>
        int operator()(const T &node) const
        {
            return node.velocity_index_;
        }

        static int run(const NodeType &node)
        {
            return boost::apply_visitor(VelIndexVisitor(), node);
        }
    };

    template <typename NodeType>
    inline int velocityIndex(NodeType &node)
    {
        return VelIndexVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct NumVelocitiesVisitor : public boost::static_visitor<int>
    {
        template <typename T>
        int operator()(const T &node) const
        {
            return node.num_velocities_;
        }

        static int run(const NodeType &node)
        {
            return boost::apply_visitor(NumVelocitiesVisitor(), node);
        }
    };

    template <typename NodeType>
    inline int numVelocities(NodeType &node)
    {
        return NumVelocitiesVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct CompositeInertiaVisitor : public boost::static_visitor<DMat<double> &>
    {
        template <typename T>
        DMat<double> &operator()(T &node) const
        {
            return node.Ic_;
        }

        static DMat<double> &run(NodeType &node)
        {
            return boost::apply_visitor(CompositeInertiaVisitor(), node);
        }
    };

    template <typename NodeType>
    struct MotionSubspaceDimVisitor : public boost::static_visitor<int>
    {
        template <typename T>
        int operator()(const T &node) const
        {
            return node.motion_subspace_dimension_;
        }

        static int run(const NodeType &node)
        {
            return boost::apply_visitor(MotionSubspaceDimVisitor(), node);
        }
    };

    template <typename NodeType>
    inline int motionSubspaceDimension(const NodeType &node)
    {
        return MotionSubspaceDimVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    inline DMat<double> &compositeInertia(NodeType &node)
    {
        return CompositeInertiaVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct MotionSubspaceVisitor : public boost::static_visitor<const DMat<double> &>
    {
        template <typename T>
        const DMat<double> &operator()(T &node) const
        {
            return node.S();
        }

        static const DMat<double> &run(NodeType &node)
        {
            return boost::apply_visitor(MotionSubspaceVisitor(), node);
        }
    };

    template <typename NodeType>
    inline const DMat<double> &motionSubspace(NodeType &node)
    {
        return MotionSubspaceVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct MotionSubspaceRingVisitor : public boost::static_visitor<const DMat<double> &>
    {
        template <typename T>
        const DMat<double> &operator()(T &node) const
        {
            return node.S_ring();
        }

        static const DMat<double> &run(NodeType &node)
        {
            return boost::apply_visitor(MotionSubspaceRingVisitor(), node);
        }
    };

    template <typename NodeType>
    inline const DMat<double> &motionSubspaceRing(NodeType &node)
    {
        return MotionSubspaceRingVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct XupVisitor : public boost::static_visitor<const GeneralizedSpatialTransform &>
    {
        template <typename T>
        const GeneralizedSpatialTransform &operator()(const T &node) const
        {
            return node.Xup_;
        }

        static const GeneralizedSpatialTransform &run(const NodeType &node)
        {
            return boost::apply_visitor(XupVisitor(), node);
        }
    };

    template <typename NodeType>
    inline const GeneralizedSpatialTransform &Xup(NodeType &node)
    {
        return XupVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct AccelerationVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename T>
        DVec<double> &operator()(T &node) const
        {
            return node.a_;
        }

        static DVec<double> &run(NodeType &node)
        {
            return boost::apply_visitor(AccelerationVisitor(), node);
        }
    };

    template <typename NodeType>
    inline DVec<double> &acceleration(NodeType &node)
    {
        return AccelerationVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct VelocityProductVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename T>
        DVec<double> &operator()(T &node) const
        {
            return node.c_;
        }

        static DVec<double> &run(NodeType &node)
        {
            return boost::apply_visitor(VelocityProductVisitor(), node);
        }
    };

    template <typename NodeType>
    inline DVec<double> &velocityProduct(NodeType &node)
    {
        return VelocityProductVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct SetForceFromAccelerationVisitor : public boost::static_visitor<>
    {
        template <typename T>
        void operator()(T &node) const
        {
            const DVec<double> Iv = node.I_ * node.v_;
            node.f_ = node.I_ * node.a_ + generalForceCrossProduct(node.v_, Iv);
        }

        static void run(NodeType &node)
        {
            boost::apply_visitor(SetForceFromAccelerationVisitor(), node);
        }
    };

    template <typename NodeType>
    inline void setForceFromAcceleration(NodeType &node)
    {
        SetForceFromAccelerationVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct NetForceVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename T>
        DVec<double> &operator()(T &node) const
        {
            return node.f_;
        }

        static DVec<double> &run(NodeType &node)
        {
            return boost::apply_visitor(NetForceVisitor(), node);
        }
    };

    template <typename NodeType>
    inline DVec<double> &netForce(NodeType &node)
    {
        return NetForceVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct NameVisitor : public boost::static_visitor<const string &>
    {
        template <typename T>
        const string &operator()(T &node) const
        {
            return node.name_;
        }

        static const string &run(NodeType &node)
        {
            return boost::apply_visitor(NameVisitor(), node);
        }
    };

    template <typename NodeType>
    inline const string &name(NodeType &node)
    {
        return NameVisitor<NodeType>::run(node);
    }

    template <typename NodeType>
    struct ExternalForceVisitor : public boost::static_visitor<DVec<double> &>
    {
        template <typename T>
        DVec<double> &operator()(T &node) const
        {
            return node.f_ext_;
        }

        static DVec<double> &run(NodeType &node)
        {
            return boost::apply_visitor(ExternalForceVisitor(), node);
        }
    };

    template <typename NodeType>
    inline DVec<double> &externalForce(NodeType &node)
    {
        return ExternalForceVisitor<NodeType>::run(node);
    }

}
