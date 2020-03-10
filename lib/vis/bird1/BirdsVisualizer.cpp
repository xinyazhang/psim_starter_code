#include "BirdsVisualizer.h"
#include "BirdsHook.h"
#include <core/bird1/BirdsCore.h>

namespace bird1 {

BirdsVisualizer::BirdsVisualizer()
{
    core_.reset(new BirdsCore);
    hook_.reset(new BirdsHook(core_.get()));
    init(core_.get(), hook_.get());
}

BirdsVisualizer::~BirdsVisualizer()
{
    // Must kill simulation thread before deleting core_
    hook_.reset(); // called killSimThread in the destructor
    core_.reset(); // No necessary but make it explicit.
}

void BirdsVisualizer::setupViewer()
{
    viewer_.data().set_face_based(true);
    viewer_.core().is_animating = true;
    viewer_.data().show_lines = false;
}

bool BirdsVisualizer::keyCallback(igl::opengl::glfw::Viewer& viewer,
                                  unsigned int key,
                                  int modifiers)
{
    if (key == ' ') {
        toggleSimulation();
        return true;
    }
    Eigen::Vector4f look4 = viewer.core().view.inverse() * Eigen::Vector4f(0, 0, 1.0, 0.0);
    Eigen::Vector4f left4 = viewer.core().view.inverse() * Eigen::Vector4f(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3f look(look4[0], look4[1], look4[2]);
    Eigen::Vector3f left(left4[0], left4[1], left4[2]);
    if (key == 'w') {
        viewer.core().camera_base_translation += look;
    }
    if (key == 's') {
        viewer.core().camera_base_translation -= look;
    }
    if (key == 'a') {
        viewer.core().camera_base_translation += left;
    }
    if (key == 'd') {
        viewer.core().camera_base_translation -= left;
    }
    return false;
}

bool BirdsVisualizer::mouseCallback(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
    return false;
}

bool BirdsVisualizer::drawGUI()
{
    if (ImGui::Button("Load Scene")) {
        auto fn = igl::file_dialog_open();
        if (!fn.empty()) {
            hook_->beginLoadScene();
            load_scene(fn);
            hook_->endLoadScene();
            initScene();
        }
    }
    return IglVisualizer::drawGUI();
}

void BirdsVisualizer::initScene()
{
    hook_->reset();
}

}
