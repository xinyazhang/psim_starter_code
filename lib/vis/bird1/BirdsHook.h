#include "../PhysicsHook.h"
#include <igl/opengl/glfw/Viewer.h>
#include <deque>

namespace bird1 {

class SimParameters;
class BirdsCore;

class BirdsHook : public PhysicsHook
{
public:
    BirdsHook(BirdsCore *core);

    virtual void drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu) override;
    
private:
    SimParameters& params_;
};

}
