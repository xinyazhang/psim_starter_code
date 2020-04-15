#include "ClothCore.h"
#include "SimParameters.h"
#include <Eigen/SVD>
#include <iostream>
#include <limits>
#include <map>
#include <set>

namespace cloth1 {

ClothCore::ClothCore()
{
    params_ = std::make_shared<SimParameters>();
    clickedVertex_ = -1;
}

ClothCore::~ClothCore()
{
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
ClothCore::getCurrentMesh() const
{
    Eigen::MatrixXd C;
    return std::make_tuple(Q_, F_, C);
}

void ClothCore::attachMesh(Eigen::Ref<Eigen::MatrixX3d> V,
                           Eigen::Ref<Eigen::MatrixX3i> F,
                           double scale)
{
    F_ = F;
    origQ_ = V * scale;
    initSimulation();
}

void ClothCore::initSimulation()
{
    Q_ = origQ_;
    Qdot_.resize(Q_.rows(), 3);
    Qdot_.setZero();
    pinnedVerts_.clear();

    int nverts = Q_.rows();
    int topleft = -1;
    int topright = -1;
    double topleftdist = -std::numeric_limits<double>::infinity();
    double toprightdist = -std::numeric_limits<double>::infinity();
    Eigen::Vector3d tr(1, 1, 0);
    Eigen::Vector3d tl(-1, 1, 0);
    for (int i = 0; i < nverts; i++) {
        double disttr = tr.dot(Q_.row(i));
        if (disttr > toprightdist) {
            toprightdist = disttr;
            topright = i;
        }
        double disttl = tl.dot(Q_.row(i));
        if (disttl > topleftdist) {
            topleftdist = disttl;
            topleft = i;
        }
    }
    pinnedVerts_.push_back(topleft);
    pinnedVerts_.push_back(topright);

    computeHinges();
}

bool ClothCore::simulateOneStep()
{
    int nverts = Q_.rows();

    Eigen::MatrixXd oldQ = Q_;
    Q_ += params_->dt * Qdot_;

    // apply constraints
    for (int i = 0; i < params_->constraintIters; i++)
        applyConstraints();

    Qdot_ = (Q_ - oldQ) / params_->dt;

    if (params_->gravityEnabled) {
        Qdot_.col(1) = Qdot_.col(1).array() + params_->dt * params_->gravityG;
    }
    return false;
}

void ClothCore::hold(int vertex_id, const Eigen::Vector3d& position)
{
    // std::cerr << "hold vertex " << vertex_id << " at " << position.transpose() << std::endl;
    clickedVertex_ = vertex_id;
    curPos_ = position;
}

void ClothCore::updateHold(const Eigen::Vector3d& position)
{
    if (clickedVertex_ < 0)
        return;
    // std::cerr << "update hold vertex " << " to " << position.transpose() << std::endl;
    curPos_ = position;
}

void ClothCore::releaseHold()
{
    clickedVertex_ = -1;
}

void ClothCore::computeHinges()
{
    // TODO: Compute Hinges and store them to H_
}

void ClothCore::applyConstraints()
{
    if (params_->pinEnabled) {
    // TODO: Pinned vertices
    }

    if (params_->stretchEnabled) {
    // TODO: Stretching
    }

    if (params_->bendingEnabled) {
    // TODO: Bending
    }

    if (clickedVertex_ != -1 && params_->pullingEnabled) {
    // TODO: Pulling an vertex
    }
}

}
