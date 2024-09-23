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

#ifndef URDF_INTERFACE_LINK_H
#define URDF_INTERFACE_LINK_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "lifo_map.h"
#include "joint.h"

namespace urdf
{

  class Inertial
  {
  public:
    Inertial() { this->clear(); };
    Pose origin;
    double mass;
    double ixx, ixy, ixz, iyy, iyz, izz;

    void clear()
    {
      origin.clear();
      mass = 0;
      ixx = ixy = ixz = iyy = iyz = izz = 0;
    };
  };

  class Link
  {
  public:
    Link() { this->clear(); };

    std::string name;

    /// inertial element
    std::shared_ptr<Inertial> inertial;

    /// Parent Joint element
    ///   explicitly stating "parent" because we want directional-ness for tree structure
    ///   every link can have one parent
    std::shared_ptr<Joint> parent_joint;

    std::map<int, std::shared_ptr<Joint>> child_joints;
    std::map<int, std::shared_ptr<Link>> child_links;
    std::map<int, std::shared_ptr<Link>> neighbors;

    std::vector<std::string> constraint_names;

    std::shared_ptr<Link> getParent() const { return parent_link_.lock(); }

    void setParent(const std::shared_ptr<Link> &parent) { parent_link_ = parent; }

    void clear()
    {
      this->name.clear();
      this->inertial.reset();
      this->parent_joint.reset();
      this->child_joints.clear();
      this->child_links.clear();
      this->neighbors.clear();
      this->constraint_names.clear();
    };

  private:
    std::weak_ptr<Link> parent_link_;
  };

}
#endif
