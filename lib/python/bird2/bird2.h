#ifndef PSIM_LIB_PYTHON_BIRD2_H
#define PSIM_LIB_PYTHON_BIRD2_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

namespace bird2 {
void define_module(py::module& m);
};

#endif
