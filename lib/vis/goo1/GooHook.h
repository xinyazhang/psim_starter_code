#ifndef PSYM_VIS_GOO1_GOOHOOK_H
#define PSYM_VIS_GOO1_GOOHOOK_H

#include "../PhysicsHook.h"
#include <core/goo1/SimParameters.h>

#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <deque>

namespace goo1 {

struct MouseClick
{
    double x;
    double y;
    SimParameters::ClickMode mode;
};

class GooCore;

class GooHook : public PhysicsHook
{
public:
    GooHook(GooCore* core);
    ~GooHook();

    virtual void drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu) override;

    virtual void mouseClicked(double x, double y, int button) override;

    virtual void tick();

private:
    GooCore* core_;
    SimParameters& params_;
    double time_;

    std::mutex message_mutex;
    std::deque<MouseClick> mouseClicks_;
};

}

#endif
