#include "goo2.h"
#include <core/goo2/GooCore.h>
#include <core/goo2/SceneObjects.h>

#if PSIM_ENABLE_VISUALIZER
#include <vis/goo2/GooVisualizer.h>
#endif

namespace goo2 {

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
	;

	SimP.def_readwrite("constraintHandling", &SimParameters::constraintHandling)
	    .def_readwrite("connectorType", &SimParameters::connectorType)
	    .def_readwrite("penaltyStiffness", &SimParameters::penaltyStiffness)
	    .def_readwrite("bendingEnabled", &SimParameters::bendingEnabled)
	    .def_readwrite("rodDensity", &SimParameters::rodDensity)
	    .def_readwrite("rodStretchingStiffness", &SimParameters::rodStretchingStiffness)
	    .def_readwrite("rodBendingStiffness", &SimParameters::rodBendingStiffness)
	    .def_readwrite("rodSegments", &SimParameters::rodSegments)
	;

	py::enum_<SimParameters::ConstraintHandling>(SimP, "ConstraintHandling", py::module_local())
		.value("CH_PENALTY", SimParameters::ConstraintHandling::CH_PENALTY)
		.value("CH_STEPPROJECT", SimParameters::ConstraintHandling::CH_STEPPROJECT)
		.value("CH_LAGRANGEMULT", SimParameters::ConstraintHandling::CH_LAGRANGEMULT)
		.export_values();

	py::enum_<SimParameters::ConnectorType>(SimP, "ConnectorType", py::module_local())
		.value("CT_SPRING", SimParameters::ConnectorType::CT_SPRING)
		.value("CT_RIGIDROD", SimParameters::ConnectorType::CT_RIGIDROD)
		.value("CT_FLEXROD", SimParameters::ConnectorType::CT_FLEXROD)
		.export_values();

	py::class_<Particle>(m, "Particle", py::module_local())
		.def_readonly("pos", &Particle::pos)
		.def_readonly("vel", &Particle::vel)
		.def_readonly("mass", &Particle::mass)
		.def_readonly("fixed", &Particle::fixed)
		.def_readonly("inert", &Particle::inert)
		.def_readonly("uid", &Particle::uid)
	;

	py::class_<GooCore, PhysicsCore, std::shared_ptr<GooCore>>(m, "GooCore", py::module_local())
		.def(py::init<>())
		.def("add_particle", &GooCore::addParticle)
		.def("add_saw", &GooCore::addSaw)
		.def("query_particle", &GooCore::queryParticle)
		.def("query_connectivity", &GooCore::queryConnectivity)
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
