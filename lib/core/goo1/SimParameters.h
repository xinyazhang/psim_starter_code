#ifndef PSYM_CORE_GOO1_SIMPARAMETERS_H
#define PSYM_CORE_GOO1_SIMPARAMETERS_H

namespace goo1 {

struct SimParameters
{
    SimParameters()
    {
        timeStep = 0.001;
        NewtonMaxIters = 20;
        NewtonTolerance = 1e-8;

        gravityEnabled = true;
        gravityG = -9.8;
        springsEnabled = true;
        springStiffness = 100;
        maxSpringStrain = 0.2;
        dampingEnabled = true;
        dampingStiffness = 1.0;
        floorEnabled = true;

        particleMass = 1.0;
        maxSpringDist = 0.25;
        particleFixed = false;

        sawRadius= 0.1;

        clickMode = CM_ADDPARTICLE;
        integrator = TI_EXPLICIT_EULER;
    }

    enum ClickMode {CM_ADDPARTICLE, CM_ADDSAW};
    enum TimeIntegrator {TI_EXPLICIT_EULER, TI_IMPLICIT_EULER, TI_IMPLICIT_MIDPOINT, TI_VELOCITY_VERLET};

    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;

    bool gravityEnabled;
    double gravityG;
    bool springsEnabled;
    double springStiffness;
    double maxSpringStrain;
    bool floorEnabled;
    bool dampingEnabled;
    double dampingStiffness;

    double particleMass;
    double maxSpringDist;
    bool particleFixed;
    double sawRadius;

    TimeIntegrator integrator;
    ClickMode clickMode;
};

}

#endif
