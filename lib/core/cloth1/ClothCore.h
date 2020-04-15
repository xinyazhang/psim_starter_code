#ifndef PSIM_CORE_CLOTH1_CLOTHCORE_H
#define PSIM_CORE_CLOTH1_CLOTHCORE_H

#include "../PhysicsCore.h"
#include <memory>
#include <vector>

namespace cloth1 {

struct SimParameters;

class ClothCore : public PhysicsCore {
public:
    ClothCore();
    ~ClothCore();

    void attachMesh(Eigen::Ref<Eigen::MatrixX3d> V,
                    Eigen::Ref<Eigen::MatrixX3i> F,
                    double scale = 50.0);

    virtual void initSimulation() override;

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
    getCurrentMesh() const override;

    std::shared_ptr<SimParameters> getPointerToSimParameters()
    {
        return params_;
    }

    virtual bool simulateOneStep() override;

    void hold(int vertex_id, const Eigen::Vector3d& position);
    void updateHold(const Eigen::Vector3d& position);
    void releaseHold();

private:
    void computeHinges();
    void applyConstraints();

    std::shared_ptr<SimParameters> params_;

    Eigen::MatrixXd origQ_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd Qdot_;
    Eigen::MatrixXi F_;

    Eigen::MatrixXi H_;

    std::vector<int> pinnedVerts_;

    int clickedVertex_;
    Eigen::Vector3d curPos_;
};

}

#endif
