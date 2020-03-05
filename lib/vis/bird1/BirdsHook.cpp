#include "BirdsHook.h"
#include <core/bird1/BirdsCore.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/file_dialog_open.h>

using namespace Eigen;

namespace bird1 {
BirdsHook::BirdsHook(BirdsCore *core)
        : PhysicsHook(core), params_(*core->getPointerToSimParameters())
{
}
    

void BirdsHook::drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu)
{
    if (ImGui::CollapsingHeader("Simulation Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputFloat("Timestep", &params_.timeStep, 0, 0, 3);
        ImGui::DragFloat("Newton Tolerance", &params_.NewtonTolerance, 0.01, 1e-16, 1e-1, "%.3e", 10);
        ImGui::InputInt("Newton Max Iters", &params_.NewtonMaxIters);
    }
    if (ImGui::CollapsingHeader("Forces", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Gravity Enabled", &params_.gravityEnabled);
        ImGui::InputFloat("Gravity G", &params_.gravityG, 0, 0, 3);
    }
}

}
