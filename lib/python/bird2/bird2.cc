#include "bird2.h"
#include <core/bird2/BirdsCore.h>
#include <core/bird2/RigidBodyInstance.h>
#include <core/bird2/RigidBodyTemplate.h>
#include <core/bird2/CollisionDetection.h>
#include <core/bird2/helper.h>

#if PSIM_ENABLE_VISUALIZER
#include <vis/bird2/BirdsVisualizer.h>
#include <vis/bird2/BirdsHook.h>
#endif

#include <pybind11/stl.h>

namespace bird2 {

#if PSIM_ENABLE_VISUALIZER
class PyBirdsVisualizer : public BirdsVisualizer {
public:
    /* Inherit the constructors */
    using BirdsVisualizer::BirdsVisualizer;

    /* Trampoline (need one for each virtual function) */
    virtual void load_scene(const std::string& file_name) override
    {
        PYBIND11_OVERLOAD_PURE(
            void, /* Return type */
            BirdsVisualizer, /* Parent class */
            load_scene, /* Name of function in C++ (must match Python name) */
            file_name /* Argument(s) */
        );
    }
};
#endif

void define_module(py::module& m)
{
    m.def("loadOBJ", &bird1::loadOBJ);
    py::class_<SimParameters, std::shared_ptr<SimParameters>> SimP(m, "SimParameters", py::module_local());
    SimP.def(py::init<>())
        .def_readwrite("timeStep", &SimParameters::timeStep)
        .def_readwrite("NewtonMaxIters", &SimParameters::NewtonMaxIters)
        .def_readwrite("NewtonTolerance", &SimParameters::NewtonTolerance)
        .def_readwrite("gravityEnabled", &SimParameters::gravityEnabled)
        .def_readwrite("gravityG", &SimParameters::gravityG)
        .def_readwrite("penaltyEnabled", &SimParameters::penaltyEnabled)
        .def_readwrite("penaltyStiffness", &SimParameters::penaltyStiffness)
        .def_readwrite("impulsesEnabled", &SimParameters::impulsesEnabled)
    ;

    py::class_<Collision>(m, "Collision", py::module_local())
        .def_readwrite("body1", &Collision::body1)
        .def_readwrite("body2", &Collision::body2)
        .def_readwrite("collidingVertex", &Collision::collidingVertex)
        .def_readwrite("collidingTet", &Collision::collidingTet)
    ;

    m.def("detect_collision", &bird2::collisionDetection);

    py::class_<RigidBodyTemplate, std::shared_ptr<RigidBodyTemplate>>(m, "RigidBodyTemplate", py::module_local())
        .def(py::init<Eigen::Ref<Eigen::MatrixX3d>, Eigen::Ref<Eigen::MatrixX3i>, double>())
        .def_property_readonly("volume", &RigidBodyTemplate::getVolume)
        .def_property_readonly("com", &RigidBodyTemplate::getCenterOfMass)
        .def_property_readonly("MI", &RigidBodyTemplate::getInertiaTensor)
        .def_property_readonly("Darray", &RigidBodyTemplate::getDistances)
        .def("getVerts", &RigidBodyTemplate::getVerts) // Makes a copy, so use them with care!
        .def("getFaces", &RigidBodyTemplate::getFaces) // Also makes a copy
        .def("getTets", &RigidBodyTemplate::getTets)   // Also (ry
        .def("distance", &RigidBodyTemplate::distance)
        .def("getJacobian", &RigidBodyTemplate::Ddistance)
        ;

    py::class_<RigidBodyInstance, std::shared_ptr<RigidBodyInstance>>(m, "RigidBodyInstance", py::module_local())
        .def_readwrite("c", &RigidBodyInstance::c)
        .def_readwrite("theta", &RigidBodyInstance::theta)
        .def_readwrite("cvel", &RigidBodyInstance::cvel)
        .def_readwrite("w", &RigidBodyInstance::w)
        .def_readwrite("density", &RigidBodyInstance::density)
        .def_readwrite("bid", &RigidBodyInstance::bid);

    py::class_<BirdsCore, PhysicsCore, std::shared_ptr<BirdsCore>>(m, "BirdsCore", py::module_local())
        .def(py::init<>())
        .def("clear_scene", &BirdsCore::clearScene)
        .def("add_mesh", &BirdsCore::addMesh)
        .def("add_single_instance", &BirdsCore::addSingleInstance)
        .def("add_instances", &BirdsCore::addInstances)
        .def("query_rigid_body_instance", &BirdsCore::queryRigidBodyInstance)
        .def("list_rigid_body_instances", &BirdsCore::listRigidBodyInstances)
        .def("get_current_mesh", &BirdsCore::getCurrentMesh)
        .def("reference_sim_parameters", &BirdsCore::getPointerToSimParameters)
        .def("compute_penalty_collision_forces", &BirdsCore::computePenaltyCollisionForces)
        .def("apply_collision_impulses", &BirdsCore::applyCollisionImpulses)
    ;
#if PSIM_ENABLE_VISUALIZER
    py::class_<BirdsVisualizer, IglVisualizer, PyBirdsVisualizer>(m, "BirdsVisualizer", py::module_local())
        .def(py::init<>())
        .def_property_readonly("_core", &BirdsVisualizer::getCore)
        .def("load_scene", &BirdsVisualizer::load_scene)
        .def("init_scene", &BirdsVisualizer::initScene);
#endif
}

};
