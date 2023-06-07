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

/* Author: Josh Faust */

#ifndef DYNACORE_URDF_INTERFACE_COLOR_H
#define DYNACORE_URDF_INTERFACE_COLOR_H

#include <string>
#include <vector>
#include <math.h>
//#include <std/algorithm/string.hpp>
//#include <std/lexical_cast.hpp>

namespace dynacore{
namespace urdf
{

class Color
{
public:
  Color() {this->clear();};
  float r;
  float g;
  float b;
  float a;

  void clear()
  {
    r = g = b = 0.0f;
    a = 1.0f;
  }
  bool init(const std::string &vector_str)
  {
    this->clear();
    std::vector<std::string> pieces;
    std::vector<float> rgba;

    //std::split( pieces, vector_str, std::is_any_of(" "));
    //for (unsigned int i = 0; i < pieces.size(); ++i)
    //{
      //if (!pieces[i].empty())
      //{
        //try
        //{
          //rgba.push_back(std::stod(pieces[i].c_str()));
        //}
        //catch (int e)
        //{
          //return false;
        //}
      //}
    //}
    std::istringstream ss(vector_str);
    std::string s;
    while(getline(ss, s, ' ')){
        try{
            rgba.push_back(std::stod(s.c_str()));
        }
        catch(int e){
            throw ParseError("Unable to parse component [" + s + "] to a double (while parsing a vector value)");
        }
    }
    if (rgba.size() != 4)
    {
      return false;
    }

    this->r = rgba[0];
    this->g = rgba[1];
    this->b = rgba[2];
    this->a = rgba[3];

    return true;
  };
};

}
}
#endif

