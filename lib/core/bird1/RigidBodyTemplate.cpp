#include "RigidBodyTemplate.h"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <Eigen/Sparse>

using namespace std;
using namespace Eigen;

namespace bird1 {

RigidBodyTemplate::RigidBodyTemplate(Eigen::Ref<Eigen::MatrixX3d> V,
                                     Eigen::Ref<Eigen::MatrixX3i> F,
                                     double scale) : volume_(0), radius_(0)
{
    inertiaTensor_.setZero();
    this->V = V * scale;
    this->F = F;

    initialize();
}

RigidBodyTemplate::~RigidBodyTemplate()
{    
}

void RigidBodyTemplate::initialize()
{
    volume_ = computeVolume();
    com_ = computeCenterOfMass();
    // TODO: Translate center of mass to origin
    inertiaTensor_ = computeInertiaTensor();    
}


double RigidBodyTemplate::computeVolume()
{
    double volume = 0;
    // TODO : Compute Volume with Stokes Theorem
    return volume;
}

Vector3d RigidBodyTemplate::computeCenterOfMass()
{
    Vector3d cm(0, 0, 0);
    // TODO: computer center of mass

    return cm;
}

Eigen::Matrix3d
RigidBodyTemplate::computeInertiaTensor()
{
    Eigen::Matrix3d inertiaTensor;    
    inertiaTensor.setZero();
    // TODO: Computer Inertia Tensor
    return inertiaTensor;
}

}
