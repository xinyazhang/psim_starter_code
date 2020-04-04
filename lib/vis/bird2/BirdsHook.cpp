#include "BirdsHook.h"
#include <core/bird2/BirdsCore.h>
#include <core/bird2/RigidBodyTemplate.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/file_dialog_open.h>
#include "furious_bird.inc"

using namespace Eigen;

namespace bird2 {
BirdsHook::BirdsHook(BirdsCore *core)
        : PhysicsHook(core), core_(core), params_(*core->getPointerToSimParameters())
{
    std::cerr << ">>> Initializing the Bird" << std::endl;
    bird_ = std::make_shared<RigidBodyTemplate>(furious_bird_v, furious_bird_f, 1.0);
    std::cerr << ">>> Bird Initialized" << std::endl;
}

BirdsHook::~BirdsHook()
{
}
    
void BirdsHook::tick()
{
    launch_mutex_.lock();
    render_mutex.lock();

    double launch_vel = 100.0;

    if (launch_) {
        Eigen::Vector3d theta(0, 0, 0);
        Eigen::Vector3d w(0, 0, 0);
        core_->addSingleInstance(bird_, 1.0, launch_pos_, theta, launch_vel * launch_dir_, w);
        launch_ = false;
    }

    render_mutex.unlock();
    launch_mutex_.unlock();
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
        ImGui::Checkbox("Penalty Forces Enabled", &params_.penaltyEnabled);
        ImGui::InputFloat("Penalty Stiffness", &params_.penaltyStiffness, 0, 0, 3);
        ImGui::Checkbox("Impulses Enabled", &params_.impulsesEnabled);
        ImGui::InputFloat("CoR", &params_.CoR, 0, 0, 3);
    }
}

void BirdsHook::launchBird(const Eigen::Vector3d& launch_pos, const Eigen::Vector3d& launch_dir)
{
	launch_mutex_.lock();
	launch_ = true;
	launch_pos_ = launch_pos;
	launch_dir_ = launch_dir;
	launch_mutex_.unlock();
}

}
