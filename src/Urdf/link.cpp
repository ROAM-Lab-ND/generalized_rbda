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

#include "grbda/Urdf/urdf_parser.h"
#include "grbda/Urdf/link.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include "grbda/Urdf/tinyxml.h"
namespace urdf
{

  bool parsePose(Pose &pose, TiXmlElement *xml);

  bool parseInertial(Inertial &i, TiXmlElement *config)
  {
    i.clear();

    // Origin
    TiXmlElement *o = config->FirstChildElement("origin");
    if (o)
    {
      if (!parsePose(i.origin, o))
        return false;
    }

    TiXmlElement *mass_xml = config->FirstChildElement("mass");
    if (!mass_xml)
    {
      return false;
    }
    if (!mass_xml->Attribute("value"))
    {
      return false;
    }

    try
    {
      i.mass = std::stod(mass_xml->Attribute("value"));
    }
    catch (int e)
    {
      std::stringstream stm;
      stm << "Inertial: mass [" << mass_xml->Attribute("value")
          << "] is not a float";
      return false;
    }

    TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
    if (!inertia_xml)
    {
      return false;
    }
    if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
          inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
          inertia_xml->Attribute("izz")))
    {
      return false;
    }
    try
    {
      i.ixx = std::stod(inertia_xml->Attribute("ixx"));
      i.ixy = std::stod(inertia_xml->Attribute("ixy"));
      i.ixz = std::stod(inertia_xml->Attribute("ixz"));
      i.iyy = std::stod(inertia_xml->Attribute("iyy"));
      i.iyz = std::stod(inertia_xml->Attribute("iyz"));
      i.izz = std::stod(inertia_xml->Attribute("izz"));
    }
    catch (int e)
    {
      std::stringstream stm;
      stm << "Inertial: one of the inertia elements is not a valid double:"
          << " ixx [" << inertia_xml->Attribute("ixx") << "]"
          << " ixy [" << inertia_xml->Attribute("ixy") << "]"
          << " ixz [" << inertia_xml->Attribute("ixz") << "]"
          << " iyy [" << inertia_xml->Attribute("iyy") << "]"
          << " iyz [" << inertia_xml->Attribute("iyz") << "]"
          << " izz [" << inertia_xml->Attribute("izz") << "]";
      return false;
    }
    return true;
  }

  bool parseLink(Link &link, TiXmlElement *config)
  {

    link.clear();

    const char *name_char = config->Attribute("name");
    if (!name_char)
    {
      return false;
    }
    link.name = std::string(name_char);

    // Inertial (optional)
    TiXmlElement *i = config->FirstChildElement("inertial");
    if (i)
    {
      link.inertial.reset(new Inertial());
      if (!parseInertial(*link.inertial, i))
      {
        return false;
      }
    }

    return true;
  }

  /* exports */
  bool exportPose(Pose &pose, TiXmlElement *xml);

  bool exportInertial(Inertial &i, TiXmlElement *xml)
  {
    TiXmlElement *inertial_xml = new TiXmlElement("inertial");

    TiXmlElement *mass_xml = new TiXmlElement("mass");
    mass_xml->SetAttribute("value", urdf_export_helpers::values2str(i.mass));
    inertial_xml->LinkEndChild(mass_xml);

    exportPose(i.origin, inertial_xml);

    TiXmlElement *inertia_xml = new TiXmlElement("inertia");
    inertia_xml->SetAttribute("ixx", urdf_export_helpers::values2str(i.ixx));
    inertia_xml->SetAttribute("ixy", urdf_export_helpers::values2str(i.ixy));
    inertia_xml->SetAttribute("ixz", urdf_export_helpers::values2str(i.ixz));
    inertia_xml->SetAttribute("iyy", urdf_export_helpers::values2str(i.iyy));
    inertia_xml->SetAttribute("iyz", urdf_export_helpers::values2str(i.iyz));
    inertia_xml->SetAttribute("izz", urdf_export_helpers::values2str(i.izz));
    inertial_xml->LinkEndChild(inertia_xml);

    xml->LinkEndChild(inertial_xml);

    return true;
  }

  bool exportLink(Link &link, TiXmlElement *xml)
  {
    TiXmlElement *link_xml = new TiXmlElement("link");
    link_xml->SetAttribute("name", link.name);

    if (link.inertial)
      exportInertial(*link.inertial, link_xml);

    xml->LinkEndChild(link_xml);

    return true;
  }

}
