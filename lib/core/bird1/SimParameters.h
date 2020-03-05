#ifndef PSIM_CORE_BIRD1_SIMPARAMETERS_H
#define PSIM_CORE_BIRD1_SIMPARAMETERS_H

namespace bird1 {

struct SimParameters
{
    SimParameters()
    {
        timeStep = 0.001;
        NewtonMaxIters = 20;
        NewtonTolerance = 1e-8;
        
        gravityEnabled = true;
        gravityG = 1.0;                
    }

    float timeStep;
    float NewtonTolerance;
    int NewtonMaxIters;
    
    bool gravityEnabled;
    float gravityG;    
};

}

#endif
