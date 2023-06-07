#ifndef TEST_ENGINE
#define TEST_ENGINE

#include <string>
#include <cTypes.h>
#include <URDF/urdf_parser.h>
#include <URDF/model.h>

template <typename T>
class TestEngine{
  public:
    TestEngine(std::string urdf_file_name);
    ~TestEngine(){}

  private:
    size_t _nDof;
    std::shared_ptr<dynacore::urdf::ModelInterface> _urdf_model;
};

#endif
