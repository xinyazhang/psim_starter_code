#include "goo1.h"
#include <core/goo1/GooCore.h>
#include <core/goo1/SceneObjects.h>

#if PSIM_ENABLE_VISUALIZER
#include <vis/goo1/GooVisualizer.h>
#endif

namespace goo1 {

void define_module(py::module& m)
{
	py::class_<SimParameters, std::shared_ptr<SimParameters> > SimP(m, "SimParameters", py::module_local());
	SimP.def(py::init<>())
	    .def_readwrite("timeStep", &SimParameters::timeStep)
	    .def_readwrite("NewtonMaxIters", &SimParameters::NewtonMaxIters)
	    .def_readwrite("NewtonTolerance", &SimParameters::NewtonTolerance)
	    .def_readwrite("gravityEnabled", &SimParameters::gravityEnabled)
	    .def_readwrite("gravityG", &SimParameters::gravityG)
	    .def_readwrite("springsEnabled", &SimParameters::springsEnabled)
	    .def_readwrite("springStiffness", &SimParameters::springStiffness)
	    .def_readwrite("maxSpringStrain", &SimParameters::maxSpringStrain)
	    .def_readwrite("dampingEnabled", &SimParameters::dampingEnabled)
	    .def_readwrite("dampingStiffness", &SimParameters::dampingStiffness)
	    .def_readwrite("floorEnabled", &SimParameters::floorEnabled)
	    .def_readwrite("particleMass", &SimParameters::particleMass)
	    .def_readwrite("maxSpringDist", &SimParameters::maxSpringDist)
	    .def_readwrite("particleFixed", &SimParameters::particleFixed)
	    .def_readwrite("sawRadius", &SimParameters::sawRadius)
	    // clickMode makes no sense
	    // .def_readwrite("clickMode", &SimParameters::clickMode)
	    .def_readwrite("integrator", &SimParameters::integrator)
	;

	py::enum_<SimParameters::TimeIntegrator>(SimP, "TimeIntegrator", py::module_local())
		.value("TI_EXPLICIT_EULER", SimParameters::TimeIntegrator::TI_EXPLICIT_EULER)
		.value("TI_IMPLICIT_EULER", SimParameters::TimeIntegrator::TI_IMPLICIT_EULER)
		.value("TI_IMPLICIT_MIDPOINT", SimParameters::TimeIntegrator::TI_IMPLICIT_MIDPOINT)
		.value("TI_VELOCITY_VERLET", SimParameters::TimeIntegrator::TI_VELOCITY_VERLET)
		.export_values();

	py::class_<Particle>(m, "Particle", py::module_local())
		.def_readonly("pos", &Particle::pos)
		.def_readonly("prevpos", &Particle::prevpos)
		.def_readonly("vel", &Particle::vel)
		.def_readonly("mass", &Particle::mass)
		.def_readonly("fixed", &Particle::fixed)
		.def_readonly("inert", &Particle::inert)
	;

	py::class_<GooCore, PhysicsCore, std::shared_ptr<GooCore>>(m, "GooCore", py::module_local())
		.def(py::init<>())
		.def("add_particle", &GooCore::addParticle)
		.def("add_saw", &GooCore::addSaw)
		.def("query_particle", &GooCore::queryParticle)
		.def("get_current_mesh", &GooCore::getCurrentMesh)
		.def("reference_sim_parameters", &GooCore::getPointerToSimParameters)
		;
#if PSIM_ENABLE_VISUALIZER
	py::class_<GooVisualizer, IglVisualizer>(m, "GooVisualizer", py::module_local())
		.def(py::init<>())
		;
#endif
}

};
