#include "../PhysicsHook.h"
#include <igl/opengl/glfw/Viewer.h>
#include <deque>

namespace bird2 {

class SimParameters;
class BirdsCore;
class RigidBodyTemplate;

class BirdsHook : public PhysicsHook
{
public:
    BirdsHook(BirdsCore *core);
    ~BirdsHook();

    virtual void drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu) override;
    virtual void tick() override;

    void beginLoadScene() { render_mutex.lock(); }
    void endLoadScene() { render_mutex.unlock(); }

    void launchBird(const Eigen::Vector3d& launchPos, const Eigen::Vector3d& launchDir);
private:
    BirdsCore* core_;
    SimParameters& params_;

    bool launch_ = false;
    Eigen::Vector3d launch_pos_;
    Eigen::Vector3d launch_dir_;
    std::shared_ptr<RigidBodyTemplate> bird_;

    std::mutex launch_mutex_;
};

}
