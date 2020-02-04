#ifndef PSIM_LIB_PYTHON_GOO1_H
#define PSIM_LIB_PYTHON_GOO1_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

namespace goo1 {
void define_module(py::module& m);
};

#endif
