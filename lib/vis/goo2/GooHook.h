#ifndef PSYM_VIS_GOO2_GOOHOOK_H
#define PSYM_VIS_GOO2_GOOHOOK_H

#include "../PhysicsHook.h"
#include <core/goo2/SimParameters.h>
#include <deque>

namespace goo2 {

class GooCore;

struct MouseClick
{
    double x;
    double y;
    SimParameters::ClickMode mode;
};

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
