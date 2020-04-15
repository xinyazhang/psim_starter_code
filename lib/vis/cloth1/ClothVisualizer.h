#include "../IglVisualizer.h"
#include <memory>
#include <thread>

namespace cloth1 {

class ClothCore;
class ClothHook;

class ClothVisualizer : public IglVisualizer {
protected:
    std::shared_ptr<ClothHook> hook_;
    std::shared_ptr<ClothCore> core_;

public:
    ClothVisualizer();
    ~ClothVisualizer();

    virtual void setupViewer() override;

    virtual bool mouseCallback(Viewer& viewer, int button, int mod) override;

    virtual bool mouseReleased(Viewer& viewer, int button, int mod);
    virtual bool mouseMoved(Viewer& viewer, int button, int mod);

    virtual bool drawGUI() override;

    std::shared_ptr<ClothCore> getCore() { return core_; }

    void attachMesh(Eigen::Ref<Eigen::MatrixX3d> V,
                    Eigen::Ref<Eigen::MatrixX3i> F,
                    double scale = 50.0);

protected:
    double clickedz_;
};

}
