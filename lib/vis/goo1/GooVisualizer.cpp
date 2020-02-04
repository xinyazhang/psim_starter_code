#include "GooHook.h"
#include "GooVisualizer.h"
#include <core/goo1/GooCore.h>

namespace goo1 {

GooVisualizer::GooVisualizer()
{
    core_.reset(new GooCore);
    hook_.reset(new GooHook(core_.get()));
    hook_->reset();
    init(core_.get(), hook_.get());
}

GooVisualizer::~GooVisualizer()
{
}

}
