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
    
    void beginLoadScene() { render_mutex.lock(); }
    void endLoadScene() { render_mutex.unlock(); }
private:
    SimParameters& params_;
};

}
