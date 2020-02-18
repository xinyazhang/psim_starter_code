#include "GooHook.h"
#include <core/goo2/GooCore.h>

using namespace Eigen;

namespace goo2 {

GooHook::GooHook(GooCore* core)
    : core_(core), PhysicsHook(core), params_(*core->getPointerToSimParameters())
{
}

GooHook::~GooHook()
{
}

void
GooHook::mouseClicked(double x, double y, int button)
{
    message_mutex.lock();
    {
        MouseClick mc;
        mc.x = x;
        mc.y = y;
        mc.mode = params_.clickMode;
        mouseClicks_.push_back(mc);
    }
    message_mutex.unlock();
}

void
GooHook::drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu)
{
    if (ImGui::CollapsingHeader("UI Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Combo("Click Adds", (int *)&params_.clickMode, "Particles\0Saws\0\0");
        ImGui::Combo("Connector Type", (int *)&params_.connectorType, "Springs\0Rigid Rods\0Flexible Rods\0\0");
    }
    if (ImGui::CollapsingHeader("Simulation Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Combo("Constraint Handling", (int *)&params_.constraintHandling, "Penalty Method\0Step and Project\0Lagrange Multipliers\0\0");

        ImGui::InputDouble("Timestep",  &params_.timeStep);
        ImGui::InputDouble("Newton Tolerance", &params_.NewtonTolerance);
        ImGui::InputInt("Newton Max Iters", &params_.NewtonMaxIters);
        ImGui::InputDouble("Penalty Stiffness", &params_.penaltyStiffness);
    }
    if (ImGui::CollapsingHeader("Forces", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Gravity Enabled", &params_.gravityEnabled);
        ImGui::InputDouble("  Gravity g", &params_.gravityG);
        ImGui::Checkbox("Springs Enabled", &params_.springsEnabled);
        ImGui::InputDouble("  Max Strain", &params_.maxSpringStrain);
        ImGui::Checkbox("Damping Enabled", &params_.dampingEnabled);
        ImGui::InputDouble("  Viscosity", &params_.dampingStiffness);
        ImGui::Checkbox("Floor Enabled", &params_.floorEnabled);
        ImGui::Checkbox("Bending Enabled", &params_.bendingEnabled);
        //viewer.imgui->addWindow(Eigen::Vector2i(1000, 0), "New Objects");
    }


    if (ImGui::CollapsingHeader("New Particles", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Is Fixed", &params_.particleFixed);
        ImGui::InputDouble("Mass", &params_.particleMass);
    }

    if (ImGui::CollapsingHeader("New Saws", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputDouble("Radius", &params_.sawRadius);
    }

    if (ImGui::CollapsingHeader("New Springs", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputDouble("Max Spring Dist", &params_.maxSpringDist);
        ImGui::InputDouble("Base Stiffness", &params_.springStiffness);
    }


    if (ImGui::CollapsingHeader("New Rods", ImGuiTreeNodeFlags_DefaultOpen))
    {
      ImGui::InputInt("Num Segments", &params_.rodSegments);
      ImGui::InputDouble("Density", &params_.rodDensity);
      ImGui::InputDouble("Stretching Stiffness", &params_.rodStretchingStiffness);
      ImGui::InputDouble("Bending Stiffness", &params_.rodBendingStiffness);
    }

}


void GooHook::tick()
{
    message_mutex.lock();
    {
        while (!mouseClicks_.empty())
        {
            MouseClick mc = mouseClicks_.front();
            mouseClicks_.pop_front();
            switch (mc.mode)
            {
                case SimParameters::ClickMode::CM_ADDPARTICLE:
                    {
                        core_->addParticle(mc.x, mc.y);
                        break;
                    }
                case SimParameters::ClickMode::CM_ADDSAW:
                    {
                        core_->addSaw(mc.x, mc.y);
                        break;
                    }
            }
        }
    }
    message_mutex.unlock();
}

}
