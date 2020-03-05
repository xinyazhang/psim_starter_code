#ifndef PSIM_CORE_BIRD1_RIGIDBODYTEMPLATE_H
#define PSIM_CORE_BIRD1_RIGIDBODYTEMPLATE_H

#include <string>
#include <Eigen/Core>
#include <set>

namespace bird1 {

class SignedDistanceField;

class RigidBodyTemplate
{
public:
    RigidBodyTemplate(Eigen::Ref<Eigen::MatrixX3d> V,
                      Eigen::Ref<Eigen::MatrixX3i> F,
                      double scale);
    ~RigidBodyTemplate();

    double getVolume() const {return volume_;}
    Eigen::Vector3d getCenterOfMass() { return com_; }
    const Eigen::Matrix3d getInertiaTensor() const {return inertiaTensor_;}

    double getBoundingRadius() const {return radius_;}
    const Eigen::MatrixX3d &getVerts() const {return V;}
    const Eigen::MatrixX3i &getFaces() const {return F;}

private:
    RigidBodyTemplate(const RigidBodyTemplate &other) = delete;
    RigidBodyTemplate &operator=(const RigidBodyTemplate &other) = delete;

    void initialize();

    // Not included in starter code
    void computeFaces();
    double computeVolume();
    Eigen::Vector3d computeCenterOfMass();
    Eigen::Matrix3d computeInertiaTensor();

    Eigen::MatrixX3d V;
    Eigen::MatrixX3i F;

    double volume_;
    double radius_;
    Eigen::Vector3d com_;  // Only used once, but kept for testing
    Eigen::Matrix3d inertiaTensor_;
};

}

#endif // RIGIDBODYTEMPLATE_H
