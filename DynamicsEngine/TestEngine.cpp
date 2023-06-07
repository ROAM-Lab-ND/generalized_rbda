#include <TestEngine.hpp>

#include <Configuration.h>


template <typename T>
TestEngine<T>::TestEngine(std::string urdf_file){
  _urdf_model = dynacore::urdf::parseURDFFile(THIS_COM + urdf_file, true);
}

template class TestEngine<double>;
template class TestEngine<float>;
