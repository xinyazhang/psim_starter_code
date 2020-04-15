#include "ClothHook.h"
#include <core/cloth1/ClothCore.h>
#include <core/cloth1/SimParameters.h>
#include <igl/file_dialog_open.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

using namespace Eigen;

namespace cloth1 {

ClothHook::ClothHook(ClothCore* core)
    : PhysicsHook(core)
    , core_(core)
    , params_(*core->getPointerToSimParameters())
{
}

ClothHook::~ClothHook()
{
}

void ClothHook::tick()
{
    mouse_mutex_.lock();
    for (MouseEvent me : mouse_events_) {
        if (me.type == MouseEvent::ME_CLICKED) {
            core_->hold(me.vertex, me.pos);
        }
        if (me.type == MouseEvent::ME_RELEASED) {
            core_->releaseHold();
        }
        if (me.type == MouseEvent::ME_DRAGGED) {
            core_->updateHold(me.pos);
        }
    }
    mouse_events_.clear();
    mouse_mutex_.unlock();
}

void ClothHook::drawGUI(igl::opengl::glfw::imgui::ImGuiMenu& menu)
{
}

void ClothHook::addMouseEvent(const MouseEvent& me)
{
    mouse_mutex_.lock();
    mouse_events_.emplace_back(me);
    mouse_mutex_.unlock();
}

}
