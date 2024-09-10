#!/bin/bash

xacro mini_cheetah.xacro > mini_cheetah.urdf
xacro mini_cheetah_fr_leg.xacro > mini_cheetah_fr_leg.urdf
xacro mini_cheetah_fl_leg.xacro > mini_cheetah_fl_leg.urdf
xacro mini_cheetah_hr_leg.xacro > mini_cheetah_hr_leg.urdf
xacro mini_cheetah_hl_leg.xacro > mini_cheetah_hl_leg.urdf
xacro mit_humanoid_leg.xacro > mit_humanoid_leg.urdf
xacro mit_humanoid.xacro > mit_humanoid.urdf
