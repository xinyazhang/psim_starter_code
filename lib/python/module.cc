// We took a revers order to include headers because there are too many
// headers to add
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
namespace py = pybind11;

#ifndef PYTHON_MODULE_NAME
#define PYTHON_MODULE_NAME pypsim
#endif

#include <core/PhysicsCore.h>
#if PSIM_ENABLE_VISUALIZER
#include <vis/IglVisualizer.h>
#endif

#ifdef PSIM_HAS_GOO1
#include "goo1/goo1.h"
#endif

#ifdef PSIM_HAS_GOO2
#include "goo2/goo2.h"
#endif

#ifdef PSIM_HAS_BIRD1
#include "bird1/bird1.h"
#endif

PYBIND11_MODULE(PYTHON_MODULE_NAME, m) {
	m.doc() = "Core of Phyisical Simulation";

	py::class_<PhysicsCore, std::shared_ptr<PhysicsCore>>(m, "PhysicsCore", py::module_local())
		.def("init_simulation", &PhysicsCore::initSimulation)
		.def("simulate_one_step", &PhysicsCore::simulateOneStep)
		;
#if PSIM_ENABLE_VISUALIZER
	py::class_<IglVisualizer>(m, "IglVisualizer", py::module_local())
		.def(py::init<>())
		.def("init", &IglVisualizer::init)
		.def("run", &IglVisualizer::run)
		;
#endif

#ifdef PSIM_HAS_GOO1
	py::module goo1m = m.def_submodule("goo1", "Goo1 project");
	goo1::define_module(goo1m);
#endif

#ifdef PSIM_HAS_GOO2
	py::module goo2m = m.def_submodule("goo2", "Goo2 project");
	goo2::define_module(goo2m);
#endif

#ifdef PSIM_HAS_BIRD1
	py::module bird1m = m.def_submodule("bird1", "Bird1 project");
	bird1::define_module(bird1m);
#endif
}
