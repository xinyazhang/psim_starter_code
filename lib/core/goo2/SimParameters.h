#ifndef PSYM_CORE_GOO2_SIMPARAMETERS_H
#define PSYM_CORE_GOO2_SIMPARAMETERS_H

namespace goo2 {

struct SimParameters {
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

        sawRadius = 0.1;

        clickMode = CM_ADDPARTICLE;

        // New parameters
        constraintHandling = CH_PENALTY;
        connectorType = CT_SPRING;
        penaltyStiffness = 1e5;
        bendingEnabled = true;

        rodDensity = 2;
        rodStretchingStiffness = 100;
        rodBendingStiffness = 0.05;
        rodSegments = 5;
    }

    enum ClickMode { CM_ADDPARTICLE,
                     CM_ADDSAW };

    enum ConstraintHandling { CH_PENALTY,
                              CH_STEPPROJECT,
                              CH_LAGRANGEMULT };

    enum ConnectorType { CT_SPRING,
                         CT_RIGIDROD,
                         CT_FLEXROD };

    ConstraintHandling constraintHandling;
    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;
    double penaltyStiffness;

    bool gravityEnabled;
    double gravityG;
    bool springsEnabled;
    bool bendingEnabled;
    double springStiffness;
    double maxSpringStrain;
    bool floorEnabled;
    bool dampingEnabled;
    double dampingStiffness;

    ClickMode clickMode;
    ConnectorType connectorType;
    double particleMass;
    double maxSpringDist;
    bool particleFixed;
    double sawRadius;

    double rodDensity;
    double rodBendingStiffness;
    double rodStretchingStiffness;
    int rodSegments;
};

}

#endif
