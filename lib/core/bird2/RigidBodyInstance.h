#ifndef PSIM_CORE_BIRD2_RIGIDBODYINSTANCE_H
#define PSIM_CORE_BIRD2_RIGIDBODYINSTANCE_H

#include <Eigen/Core>
#include <list>
#include <vector>

namespace bird2 {

class RigidBodyTemplate;
struct AABBNode;

class RigidBodyInstance
{
public:
    RigidBodyInstance(const RigidBodyTemplate &rbtemplate, const Eigen::Vector3d &c, const Eigen::Vector3d &theta, const Eigen::Vector3d &cvel, const Eigen::Vector3d &w, double density);
    ~RigidBodyInstance();

    Eigen::Vector3d c;
    Eigen::Vector3d theta;

    Eigen::Vector3d cvel;
    Eigen::Vector3d w;

    double density;

    AABBNode *AABB;
    
    const RigidBodyTemplate &getTemplate() const {return rbtemplate_;}
    
    int32_t bid;
private:
    const RigidBodyTemplate &rbtemplate_;
};

}

#endif // RIGIDBODYINSTANCE_H
