/*********************************************************************
 * Software Ligcense Agreement (BSD License)
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

/* Author: John Hsu */

#include <sstream>
#include "grbda/Urdf/joint.h"
#include <algorithm>
#include "grbda/Urdf/tinyxml.h"
#include "grbda/Urdf/urdf_parser.h"
namespace urdf
{

  bool parsePose(Pose &pose, TiXmlElement *xml);

  bool parseJoint(Joint &joint, TiXmlElement *config)
  {
    joint.clear();

    // Get Joint Name
    const char *name = config->Attribute("name");
    if (!name)
    {
      return false;
    }
    joint.name = name;

    // Get whether the joint is included in the independent coordinates
    const char *independent = config->Attribute("independent");
    joint.independent = true;
    if (independent)
    {
      std::string independent_str = independent;
      if (independent_str != "true")
        joint.independent = false;
    }

    // Get transform from Parent Link to Joint Frame
    TiXmlElement *origin_xml = config->FirstChildElement("origin");
    if (!origin_xml)
    {
      joint.parent_to_joint_origin_transform.clear();
    }
    else
    {
      if (!parsePose(joint.parent_to_joint_origin_transform, origin_xml))
      {
        joint.parent_to_joint_origin_transform.clear();
        return false;
      }
    }

    // Get Parent Link
    TiXmlElement *parent_xml = config->FirstChildElement("parent");
    if (parent_xml)
    {
      const char *pname = parent_xml->Attribute("link");
      if (!pname)
      {
        printf("[joint] no parent link name\n");
      }
      else
      {
        joint.parent_link_name = std::string(pname);
      }
    }

    // Get Child Link
    TiXmlElement *child_xml = config->FirstChildElement("child");
    if (child_xml)
    {
      const char *pname = child_xml->Attribute("link");
      if (!pname)
      {
      }
      else
      {
        joint.child_link_name = std::string(pname);
      }
    }

    // Get Joint type
    const char *type_char = config->Attribute("type");
    if (!type_char)
    {
      return false;
    }

    std::string type_str = type_char;
    if (type_str == "planar")
      joint.type = Joint::PLANAR;
    else if (type_str == "floating")
      joint.type = Joint::FLOATING;
    else if (type_str == "revolute")
      joint.type = Joint::REVOLUTE;
    else if (type_str == "continuous")
      joint.type = Joint::CONTINUOUS;
    else if (type_str == "prismatic")
      joint.type = Joint::PRISMATIC;
    else if (type_str == "fixed")
      joint.type = Joint::FIXED;
    else
    {
      return false;
    }

    // Get Joint Axis
    if (joint.type != Joint::FLOATING && joint.type != Joint::FIXED)
    {
      // axis
      TiXmlElement *axis_xml = config->FirstChildElement("axis");
      if (!axis_xml)
      {
        joint.axis = Vector3(1.0, 0.0, 0.0);
      }
      else
      {
        if (axis_xml->Attribute("xyz"))
        {
          try
          {
            joint.axis.init(axis_xml->Attribute("xyz"));
          }
          catch (ParseError &e)
          {
            joint.axis.clear();
            return false;
          }
        }
      }
    }

    return true;
  }

  /* exports */
  bool exportPose(Pose &pose, TiXmlElement *xml);

  bool exportJoint(Joint &joint, TiXmlElement *xml)
  {
    TiXmlElement *joint_xml = new TiXmlElement("joint");
    joint_xml->SetAttribute("name", joint.name);
    joint_xml->SetAttribute("independent", joint.independent ? "true" : "false");
    if (joint.type == urdf::Joint::PLANAR)
      joint_xml->SetAttribute("type", "planar");
    else if (joint.type == urdf::Joint::FLOATING)
      joint_xml->SetAttribute("type", "floating");
    else if (joint.type == urdf::Joint::REVOLUTE)
      joint_xml->SetAttribute("type", "revolute");
    else if (joint.type == urdf::Joint::CONTINUOUS)
      joint_xml->SetAttribute("type", "continuous");
    else if (joint.type == urdf::Joint::PRISMATIC)
      joint_xml->SetAttribute("type", "prismatic");
    else if (joint.type == urdf::Joint::FIXED)
      joint_xml->SetAttribute("type", "fixed");
    else
      throw ParseError("Joint has no type");

    // origin
    exportPose(joint.parent_to_joint_origin_transform, joint_xml);

    // axis
    TiXmlElement *axis_xml = new TiXmlElement("axis");
    axis_xml->SetAttribute("xyz", urdf_export_helpers::values2str(joint.axis));
    joint_xml->LinkEndChild(axis_xml);

    // parent
    TiXmlElement *parent_xml = new TiXmlElement("parent");
    parent_xml->SetAttribute("link", joint.parent_link_name);
    joint_xml->LinkEndChild(parent_xml);

    // child
    TiXmlElement *child_xml = new TiXmlElement("child");
    child_xml->SetAttribute("link", joint.child_link_name);
    joint_xml->LinkEndChild(child_xml);

    xml->LinkEndChild(joint_xml);
    return true;
  }

}
