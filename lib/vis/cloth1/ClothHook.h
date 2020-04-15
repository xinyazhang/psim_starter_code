#include "../PhysicsHook.h"
#include <deque>
#include <igl/opengl/glfw/Viewer.h>

namespace cloth1 {

struct SimParameters;
class ClothCore;

struct MouseEvent {
    enum METype {
        ME_CLICKED,
        ME_RELEASED,
        ME_DRAGGED
    };

    METype type;
    int vertex;
    Eigen::Vector3d pos;
};

class ClothHook : public PhysicsHook {
public:
    ClothHook(ClothCore* core);
    ~ClothHook();

    virtual void drawGUI(igl::opengl::glfw::imgui::ImGuiMenu& menu) override;
    virtual void tick() override;

    void lockRenderer() { render_mutex.lock(); }
    void unlockRenderer() { render_mutex.unlock(); }

    const Eigen::MatrixXd& getV() const { return renderQ_; }
    const Eigen::MatrixXi& getF() const { return renderF_; }

    void addMouseEvent(const MouseEvent& me);

private:
    ClothCore* core_;
    SimParameters& params_;

    bool launch_ = false;
    Eigen::Vector3d launch_pos_;
    Eigen::Vector3d launch_dir_;

    std::mutex mouse_mutex_;
    std::vector<MouseEvent> mouse_events_;
};

}
