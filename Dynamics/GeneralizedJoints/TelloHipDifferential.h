#pragma once

#include "GeneralizedJoint.h"
#include "CasadiGen.h"

namespace grbda
{

    namespace GeneralizedJoints
    {

	class TelloHipDifferential : public Base
	{
	public:
	    TelloHipDifferential(Body &rotor_1, Body &rotor_2, Body &link_1, Body &link_2,
				 CoordinateAxis rotor_axis_1, CoordinateAxis rotor_axis_2,
				 CoordinateAxis joint_axis_1, CoordinateAxis joint_axis_2);
	    virtual ~TelloHipDifferential() {}

	    GeneralizedJointTypes type() const override { return GeneralizedJointTypes::TelloHipDifferential; }

		void updateKinematics(const State<double> &joint_pos,
							  const State<double> &joint_vel) override;

		void computeSpatialTransformFromParentToCurrentCluster(
		GeneralizedSpatialTransform &Xup) const override;

	private:
	    JointPtr rotor_1_joint_;
	    JointPtr rotor_2_joint_;
	    JointPtr link_1_joint_;
	    JointPtr link_2_joint_;

	    SpatialTransform X21_;
	    
	    const Body rotor_1_;
	    const Body rotor_2_;
	    const Body link_1_;
	    const Body link_2_;

	};

    }

} // namespace grbda
