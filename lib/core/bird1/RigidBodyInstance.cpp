#include "RigidBodyInstance.h"
#include "VectorMath.h"
#include "RigidBodyTemplate.h"
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;
using namespace std;

namespace bird1 {

RigidBodyInstance::RigidBodyInstance(const RigidBodyTemplate &rbtemplate,
    const Eigen::Vector3d &c, const Eigen::Vector3d &theta,
    const Eigen::Vector3d &cvel, const Eigen::Vector3d &w,
    double density)
    : c(c), theta(theta), cvel(cvel), w(w), density(density), rbtemplate_(rbtemplate)
{
}

RigidBodyInstance::~RigidBodyInstance()
{    
}

}
