#ifndef PSIM_VIS_IGL_VISUALIZER_H
#define PSIM_VIS_IGL_VISUALIZER_H

#include <core/PhysicsCore.h>
#include <vis/PhysicsHook.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

using Viewer = igl::opengl::glfw::Viewer;
using ImGuiMenu = igl::opengl::glfw::imgui::ImGuiMenu;

class IglVisualizer {
protected:
    Viewer viewer_;
    ImGuiMenu menu_;

    PhysicsCore *core_;
    PhysicsHook *hook_;
public:
    IglVisualizer()
    {
    }

    void init(PhysicsCore* core, PhysicsHook* hook)
    {
        core_ = core;
        hook_ = hook;

	setupViewer();

        viewer_.callback_key_pressed = [=](Viewer& viewer, unsigned int key, int mod) {
            return this->keyCallback(viewer, key, mod);
        };
        viewer_.callback_pre_draw = [=](Viewer& viewer) {
            return this->drawCallback(viewer);
        };
        viewer_.callback_mouse_down = [=](Viewer& viewer, int button, int mod) {
            return this->mouseCallback(viewer, button, mod);
        };
        viewer_.callback_mouse_scroll = [=](Viewer& viewer, float delta) {
            return this->mouseScrollCallback(viewer, delta);
        };

        viewer_.plugins.push_back(&menu_);
        menu_.callback_draw_viewer_menu = [=]() {
            this->drawGUI();
        };
    }

    virtual void setupViewer()
    {
        viewer_.core().orthographic = true;
        viewer_.core().camera_zoom = 4.0;
        viewer_.data().show_lines = false;
        viewer_.data().set_face_based(false);
        viewer_.core().is_animating = true;
    }

    virtual ~IglVisualizer() {};

    virtual bool drawCallback(Viewer &viewer)
    {
        if (!hook_)
            return false;
        hook_->render(viewer);
        return false;
    }

    virtual void renderGeometry(Eigen::MatrixXd renderQ,
				Eigen::MatrixXi renderF,
				Eigen::MatrixXd renderC)
    {
        hook_->useGeometry(renderQ, renderF, renderC);
        hook_->render(viewer_);
    }

    virtual bool keyCallback(Viewer& viewer, unsigned int key, int modifiers)
    {
        if (key == ' ')
        {
            toggleSimulation();
            return true;
        }
        return false;
    }

    virtual bool mouseCallback(Viewer& viewer, int button, int modifier)
    {
        if (!hook_)
            return false;

        Eigen::Vector3f pos(viewer.down_mouse_x, viewer.down_mouse_y, 0);
        Eigen::Matrix4f model = viewer.core().view;
        Eigen::Vector3f unproj = igl::unproject(pos, model, viewer.core().proj, viewer.core().viewport);
        hook_->mouseClicked(unproj[0], -unproj[1], button);
        return true;
    }

    virtual bool mouseScrollCallback(Viewer& viewer, float delta)
    {
        return true;
    }

    virtual bool drawGUI()
    {
        if (ImGui::CollapsingHeader("Simulation Control", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Run/Pause Sim", ImVec2(-1, 0)))
            {
                toggleSimulation();
            }
            if (ImGui::Button("Reset Sim", ImVec2(-1, 0)))
            {
                resetSimulation();
            }
        }
        hook_->drawGUI(menu_);
        return false;
    }

    void run()
    {
        viewer_.launch();
    }

protected:
    void toggleSimulation()
    {
        if (!hook_)
            return;

        if (hook_->isPaused())
            hook_->run();
        else
            hook_->pause();
    }

    void resetSimulation()
    {
        if (!hook_)
            return;

        hook_->reset();
    }
};

#endif
