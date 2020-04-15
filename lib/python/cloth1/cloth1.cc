#include "cloth1.h"
#include <core/bird1/helper.h>
#include <core/cloth1/ClothCore.h>
#include <core/cloth1/SimParameters.h>

#if PSIM_ENABLE_VISUALIZER
#include <vis/cloth1/ClothHook.h>
#include <vis/cloth1/ClothVisualizer.h>
#endif

namespace cloth1 {

#define DEF_RW(NAME) def_readwrite(#NAME, &SimParameters::NAME)

void define_module(py::module& m)
{
    m.def("loadOBJ", &bird1::loadOBJ);
    py::class_<SimParameters, std::shared_ptr<SimParameters>> SimP(m, "SimParameters", py::module_local());
    SimP.def(py::init<>())
        .DEF_RW(dt)
        .DEF_RW(constraintIters)
        .DEF_RW(gravityEnabled)
        .DEF_RW(gravityG)
        .DEF_RW(pinEnabled)
        .DEF_RW(pinWeight)
        .DEF_RW(stretchEnabled)
        .DEF_RW(stretchWeight)
        .DEF_RW(bendingEnabled)
        .DEF_RW(bendingWeight)
        .DEF_RW(pullingEnabled)
        .DEF_RW(pullingWeight);

    py::class_<ClothCore, PhysicsCore, std::shared_ptr<ClothCore>>(m, "ClothCore", py::module_local())
        .def(py::init<>())
        .def("attach_mesh", &ClothCore::attachMesh,
             py::arg("V"),
             py::arg("F"),
             py::arg("scale") = 50.0)
        .def("get_current_mesh", &ClothCore::getCurrentMesh)
        .def("reference_sim_parameters", &ClothCore::getPointerToSimParameters)
        .def("hold", &ClothCore::hold)
        .def("update_hold", &ClothCore::updateHold)
        .def("release_hold", &ClothCore::releaseHold);
#if PSIM_ENABLE_VISUALIZER
    py::class_<ClothVisualizer, IglVisualizer>(m, "ClothVisualizer", py::module_local())
        .def(py::init<>())
        .def("attach_mesh", &ClothVisualizer::attachMesh,
             py::arg("V"),
             py::arg("F"),
             py::arg("scale") = 50.0)
        .def_property_readonly("_core", &ClothVisualizer::getCore);
#endif
}

};
