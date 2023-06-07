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

#include <vector>
#include "urdf_parser.h"
// #include <console_bridge/console.h>
#include <fstream>
namespace dynacore{
namespace urdf{

bool parseMaterial(Material &material, TiXmlElement *config, bool only_name_is_ok);
bool parseLink(Link &link, TiXmlElement *config);
bool parseJoint(Joint &joint, TiXmlElement *config);

std::shared_ptr<ModelInterface>  parseURDFFile(const std::string &path, bool verbose)
{
    std::ifstream stream( path.c_str() );
    if (!stream)
    {
        printf("couldn't find the file: %s\n", path.c_str());
      return std::shared_ptr<ModelInterface>();
    }

    std::string xml_str((std::istreambuf_iterator<char>(stream)),
	                     std::istreambuf_iterator<char>());
    return urdf::parseURDF( xml_str , verbose);
}

std::shared_ptr<ModelInterface>  parseURDF(const std::string &xml_string, bool verbose)
{
  std::shared_ptr<ModelInterface> model(new ModelInterface);
  model->clear();

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  if (xml_doc.Error())
  {
    xml_doc.ClearError();
    model.reset();
    return model;
  }
  
  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    model.reset();
    return model;
  }

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    model.reset();
    return model;
  }
  model->name_ = std::string(name);

  if(verbose)  printf("name: %s\n", model->name_.c_str());

  // Get all Material elements
  for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
  {
    std::shared_ptr<Material> material;
    material.reset(new Material);

    try {
      parseMaterial(*material, material_xml, false); // material needs to be fully defined here
      if (model->getMaterial(material->name))
      {
        material.reset();
        model.reset();
        return model;
      }
      else
      {
        model->materials_.insert(make_pair(material->name,material));
      }
    }
    catch (ParseError &/*e*/) {
      material.reset();
      model.reset();
      return model;
    }
  }
  int count(0);
  // Get all Link elements
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {

    std::shared_ptr<Link> link;
    link.reset(new Link);

    try {
      parseLink(*link, link_xml);
      if (model->getLink(link->name))
      {
        model.reset();
        return model;
      }
      else
      {
        // set link visual material
        if (link->visual)
        {
          if (!link->visual->material_name.empty())
          {
            if (model->getMaterial(link->visual->material_name))
            {
              link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
            }
            else
            {
              if (link->visual->material)
              {
                model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
              }
              else
              {
                model.reset();
                return model;
              }
            }
          }
        }
        model->links_.insert(make_pair(link->name,link));
        ++count;
        if (verbose) printf("%d th link: %s\n", count, link->name.c_str());
      }
    }
    catch (ParseError &/*e*/) {
        printf("[URDF] link parsing error\n");
      model.reset();
      return model;
    }
  }
  if (model->links_.empty()){
      printf("[URDF] No Link\n");
    model.reset();
    return model;
  }
  // Get all Joint elements
  count = 0;
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    std::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (parseJoint(*joint, joint_xml))
    {
      if (model->getJoint(joint->name))
      {
        model.reset();
        return model;
      }
      else
      {
        model->joints_.insert(make_pair(joint->name,joint));
        ++count;
        if (verbose) {
          printf("%d th joint: %s\n", count, joint->name.c_str());
          if(joint->actuator) joint->actuator->print();
        }
      }
    }
    else
    {
      model.reset();
      return model;
    }
  }


  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  try 
  {
    model->initTree(parent_link_tree);
  }
  catch(ParseError &e)
  {
    model.reset();
    return model;
  }

  // find the root link
  try
  {
    model->initRoot(parent_link_tree);
  }
  catch(ParseError &e)
  {
    printf("[URDF Parser] Cannot Find the Root Link\n");
    model.reset();
    return model;
  }
  
  return model;
}

bool exportMaterial(Material &material, TiXmlElement *config);
bool exportLink(Link &link, TiXmlElement *config);
bool exportJoint(Joint &joint, TiXmlElement *config);
TiXmlDocument*  exportURDF(const ModelInterface &model)
{
  TiXmlDocument *doc = new TiXmlDocument();

  TiXmlElement *robot = new TiXmlElement("robot");
  robot->SetAttribute("name", model.name_);
  doc->LinkEndChild(robot);


  for (std::map<std::string, std::shared_ptr<Material> >::const_iterator m=model.materials_.begin(); m!=model.materials_.end(); m++)
  {
    exportMaterial(*(m->second), robot);
  }

  for (std::map<std::string, std::shared_ptr<Link> >::const_iterator l=model.links_.begin(); l!=model.links_.end(); l++)  
  {
    exportLink(*(l->second), robot);
  }
  	
  for (std::map<std::string, std::shared_ptr<Joint> >::const_iterator j=model.joints_.begin(); j!=model.joints_.end(); j++)  
  {
    exportJoint(*(j->second), robot);
  }

  return doc;
}
    
TiXmlDocument*  exportURDF(std::shared_ptr<ModelInterface> &model)
{
  return exportURDF(*model);
}


}
}
