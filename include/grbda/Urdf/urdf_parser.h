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

// TODO(@MatthewChignoli): Comments before merging
// - Where did Donghyun get this code? Who can we credit?
// - Can we make this an external dependency? Or do we have to provide the source directly?
// - Should we put all this code in the grbda namespace?
// - Should we modify the code to conform to our style guide?
// - Where should we put the urdfs?
// - I plan on using this code to help write mine, but I will make substanstial changes. Do I still need to include the copyright at the top?
// - Might be nice if we could make better use of xacro to make the urdfs more readable

#ifndef URDF_PARSER_URDF_PARSER_H
#define URDF_PARSER_URDF_PARSER_H

#include <string>
#include <map>
#include "tinyxml.h"
#include "model.h"

#include "exportdecl.h"
namespace urdf_export_helpers
{

  URDFDOM_DLLAPI std::string values2str(unsigned int count, const double *values, double (*conv)(double) = NULL);
  URDFDOM_DLLAPI std::string values2str(urdf::Vector3 vec);
  URDFDOM_DLLAPI std::string values2str(urdf::Rotation rot);
  URDFDOM_DLLAPI std::string values2str(double d);

}

namespace urdf
{

  URDFDOM_DLLAPI std::shared_ptr<ModelInterface> parseURDF(const std::string &xml_string, bool verbose = false);
  URDFDOM_DLLAPI std::shared_ptr<ModelInterface> parseURDFFile(const std::string &path, bool verbose = false);
  URDFDOM_DLLAPI std::shared_ptr<ModelInterface> parseURDFFiles(const std::vector<std::string> &path, bool verbose = false);
  URDFDOM_DLLAPI TiXmlDocument *exportURDF(std::shared_ptr<ModelInterface> &model);
  URDFDOM_DLLAPI TiXmlDocument *exportURDF(const ModelInterface &model);
  URDFDOM_DLLAPI bool parsePose(Pose &, TiXmlElement *);
}
#endif
