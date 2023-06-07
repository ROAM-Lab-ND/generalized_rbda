#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <lcm/lcm-cpp.hpp>
#include <cppTypes.h>

template <typename T>
class Manipulator{
  public:
    Manipulator(){}
    virtual ~Manipulator(){}

    virtual bool step() = 0;
    virtual void send_message() = 0;

  protected:
    int _dof = 0;
};

#endif
