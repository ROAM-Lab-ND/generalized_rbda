/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Wim Meeussen */

#ifndef URDF_INTERFACE_JOINT_H
#define URDF_INTERFACE_JOINT_H

#include <string>
#include <vector>
#include <memory>

#include "pose.h"

namespace urdf
{

  class Link;
  class Joint
  {
  public:
    Joint() { this->clear(); };

    std::string name;
    enum
    {
      UNKNOWN,
      REVOLUTE,
      CONTINUOUS,
      PRISMATIC,
      FLOATING,
      PLANAR,
      UNIVERSAL,
      FIXED
    } type;

    /// \brief     type_       meaning of axis_
    /// ------------------------------------------------------
    ///            UNKNOWN     unknown type
    ///            REVOLUTE    rotation axis
    ///            PRISMATIC   translation axis
    ///            FLOATING    N/A
    ///            PLANAR      plane normal axis
    ///            UNIVERSAL   rotation axes
    ///            FIXED       N/A
    Vector3 axis;

    // TODO(@MatthewChignoli): I am not sure if we can represent the universal joint with a single axis

    bool independent;

    /// child Link element
    ///   child link frame is the same as the Joint frame
    std::string child_link_name;

    /// parent Link element
    ///   origin specifies the transform from Parent Link to Joint Frame
    std::string parent_link_name;
    /// transform from Parent Link frame to Joint frame
    Pose parent_to_joint_origin_transform;

    void clear()
    {
      this->axis.clear();
      this->child_link_name.clear();
      this->parent_link_name.clear();
      this->parent_to_joint_origin_transform.clear();
      this->type = UNKNOWN;
    };
  };

}
#endif
