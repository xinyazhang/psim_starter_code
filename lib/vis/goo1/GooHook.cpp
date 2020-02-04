#include "GooHook.h"
#include <core/goo1/GooCore.h>

using namespace Eigen;

namespace goo1 {

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
    }
    if (ImGui::CollapsingHeader("Simulation Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputDouble("Timestep",  &params_.timeStep);
        ImGui::Combo("Integrator", (int *)&params_.integrator, "Explicit Euler\0Implicit Euler\0Implicit Midpoint\0Velocity Verlet\0\0");
        ImGui::InputDouble("Newton Tolerance", &params_.NewtonTolerance);
        ImGui::InputInt("Newton Max Iters", &params_.NewtonMaxIters);
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
