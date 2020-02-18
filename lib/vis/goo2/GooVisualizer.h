#include "../IglVisualizer.h"
#include <thread>
#include <memory>

static PhysicsHook *hook = NULL;

namespace goo2 {

class GooCore;
class GooHook;

class GooVisualizer : public IglVisualizer {
    std::unique_ptr<GooCore> core_;
    std::unique_ptr<GooHook> hook_;
public:
    GooVisualizer();
    ~GooVisualizer();
};

}
