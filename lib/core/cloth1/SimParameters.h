#ifndef PSIM_CORE_CLOTH1_SIMPARAMETERS_H
#define PSIM_CORE_CLOTH1_SIMPARAMETERS_H

namespace cloth1 {

struct SimParameters {
    float dt = 1e-3;
    int constraintIters = 5;

    bool gravityEnabled = true;
    float gravityG = -9.8;

    bool pinEnabled = true;
    float pinWeight = 1.0;

    bool stretchEnabled = true;
    float stretchWeight = 0.5;

    bool bendingEnabled = true;
    float bendingWeight = 0.5;

    bool pullingEnabled = true;
    float pullingWeight = 0.5;
};

}

#endif
