#include "ClothVisualizer.h"
#include "ClothHook.h"
#include <core/cloth1/ClothCore.h>
#include <core/cloth1/SimParameters.h>
#include <igl/unproject_onto_mesh.h>

namespace cloth1 {

ClothVisualizer::ClothVisualizer()
{
    core_.reset(new ClothCore);
    hook_.reset(new ClothHook(core_.get()));
    init(core_.get(), hook_.get());
}

ClothVisualizer::~ClothVisualizer()
{
    // Must kill simulation thread before deleting core_
    hook_.reset(); // called killSimThread in the destructor
    core_.reset(); // No necessary but make it explicit.
}

void ClothVisualizer::setupViewer()
{
    viewer_.data().set_face_based(true);
    viewer_.core().is_animating = true;

    viewer_.callback_mouse_up = [=](Viewer& viewer, int button, int mod) {
        return this->mouseReleased(viewer, button, mod);
    };
    viewer_.callback_mouse_move = [=](Viewer& viewer, int button, int mod) {
        return this->mouseMoved(viewer, button, mod);
    };
}

bool ClothVisualizer::mouseCallback(Viewer& viewer, int button, int modifier)
{
    if (button != static_cast<int>(Viewer::MouseButton::Left))
        return false;

    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    int fid;
    Eigen::Vector3f bc;
    MouseEvent me;
    bool ret = true;

    hook_->lockRenderer(); // Protect the result of hook_->getV()
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y),
                                 viewer.core().view,
                                 viewer.core().proj,
                                 viewer.core().viewport,
                                 hook_->getV(), hook_->getF(),
                                 fid, bc)) {
        int bestvert = -1;
        double bestcoord = 2.0;
        for (int j = 0; j < 3; j++) {
            if (bc[j] < bestcoord) {
                bestcoord = bc[j];
                bestvert = j;
            }
        }
        me.type = MouseEvent::ME_CLICKED;
        me.vertex = hook_->getF()(fid, bestvert);

        Eigen::Vector3f proj;
        Eigen::Vector3f pt = hook_->getV().row(me.vertex).cast<float>();
        Eigen::Matrix4f modelview = viewer.core().view;
        proj = igl::project(pt, modelview, viewer.core().proj, viewer.core().viewport);

        clickedz_ = proj[2];
        Eigen::Vector3f pos;
        pos << x, y, clickedz_;
        Eigen::Vector3f unproj = igl::unproject(pos,
                                                modelview,
                                                viewer.core().proj,
                                                viewer.core().viewport);
        me.pos = unproj.cast<double>();
        ret = true;
    } else {
        me.type = MouseEvent::ME_RELEASED;
        ret = false;
    }
    hook_->unlockRenderer();

    hook_->addMouseEvent(me);
    return ret;
}

bool ClothVisualizer::mouseReleased(Viewer& viewer, int button, int mod)
{
    MouseEvent me;
    me.type = MouseEvent::ME_RELEASED;
    hook_->addMouseEvent(me);
    return false;
}

bool ClothVisualizer::mouseMoved(Viewer& viewer, int button, int mod)
{
    MouseEvent me;
    me.type = MouseEvent::ME_DRAGGED;
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    Eigen::Vector3d pos(x, y, clickedz_);
    igl::unproject(pos, viewer.core().view, viewer.core().proj, viewer.core().viewport, me.pos);
    hook_->addMouseEvent(me);
    return false;
}

bool ClothVisualizer::drawGUI()
{
    auto param = core_->getPointerToSimParameters();
    if (ImGui::CollapsingHeader("Simulation Options", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputFloat("Timestep", &param->dt, 0, 0, 3);
        ImGui::InputInt("Constraint Iters", &param->constraintIters);
    }
    if (ImGui::CollapsingHeader("Forces", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Gravity Enabled", &param->gravityEnabled);
        ImGui::InputFloat("Gravity G", &param->gravityG, 0, 0, 3);
        ImGui::Checkbox("Pins Enabled", &param->pinEnabled);
        ImGui::InputFloat("Pin Weight", &param->pinWeight, 0, 0, 3);
        ImGui::Checkbox("Stretching Enabled", &param->stretchEnabled);
        ImGui::InputFloat("Stretching Weight", &param->stretchWeight, 0, 0, 3);
        ImGui::Checkbox("Bending Enabled", &param->bendingEnabled);
        ImGui::InputFloat("Bending Weight", &param->bendingWeight, 0, 0, 3);
        ImGui::Checkbox("Pulling Enabled", &param->pullingEnabled);
        ImGui::InputFloat("Pulling Weight", &param->pullingWeight, 0, 0, 3);
    }
    return IglVisualizer::drawGUI();
}

void ClothVisualizer::attachMesh(Eigen::Ref<Eigen::MatrixX3d> V,
                                 Eigen::Ref<Eigen::MatrixX3i> F,
                                 double scale)
{
    core_->attachMesh(V, F, scale);
    resetSimulation();
}

}
