#include "../IglVisualizer.h"
#include <memory>
#include <thread>

namespace bird1 {

class BirdsCore;
class BirdsHook;

class BirdsVisualizer : public IglVisualizer {
protected:
    std::shared_ptr<BirdsHook> hook_;
    std::shared_ptr<BirdsCore> core_;

public:
    BirdsVisualizer();
    ~BirdsVisualizer();

    virtual void setupViewer() override;
    virtual bool keyCallback(Viewer& viewer, unsigned int key, int modifiers) override;
    virtual bool mouseCallback(Viewer& viewer, int button, int modifier) override;
    virtual bool mouseScrollCallback(Viewer& viewer, float delta)
    {
        return false;
    }

    // Named in snake converntion for python implementation
    virtual void load_scene(const std::string& file_name) = 0;

    // Should be called after load_scene otherwise nothing appears.
    void initScene();

    virtual bool drawGUI() override;

    std::shared_ptr<BirdsCore> getCore() { return core_; }
};

}
