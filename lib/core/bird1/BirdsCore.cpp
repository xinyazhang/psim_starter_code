#include "BirdsCore.h"
#include "helper.h"
#include "RigidBodyTemplate.h"
#include "RigidBodyInstance.h"
#include "VectorMath.h"
#include <iostream>
#include <Eigen/LU> // Required for .inverse()

using namespace Eigen;

namespace bird1 {

BirdsCore::BirdsCore()
{
    params_ = std::make_shared<SimParameters>();
    initSimulation();
}

BirdsCore::~BirdsCore()
{
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
BirdsCore::getCurrentMesh() const
{
    int totverts = 0;
    int totfaces = 0;
    for (const auto& rbi : bodies_)
    {
        totverts += rbi->getTemplate().getVerts().rows();
        totfaces += rbi->getTemplate().getFaces().rows();
    }

    Eigen::MatrixXd renderQ;
    Eigen::MatrixXi renderF;
    Eigen::MatrixXd renderC;

    renderQ.resize(totverts, 3);
    renderF.resize(totfaces, 3);
    int voffset = 0;
    int foffset = 0;
    for (const auto& rbi : bodies_)
    {
        int nverts = rbi->getTemplate().getVerts().rows();
        for (int i = 0; i < nverts; i++)
            renderQ.row(voffset + i) = (rbi->c + VectorMath::rotationMatrix(rbi->theta)*rbi->getTemplate().getVerts().row(i).transpose()).transpose();
        int nfaces = rbi->getTemplate().getFaces().rows();
        for (int i = 0; i < nfaces; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                renderF(foffset + i, j) = rbi->getTemplate().getFaces()(i, j) + voffset;
            }
        }
        voffset += nverts;
        foffset += nfaces;
    }
    // std::cerr << __func__ << " Nbodies " << bodies_.size() << std::endl;
    return std::make_tuple(renderQ, renderF, renderC);
}


void BirdsCore::initSimulation()
{
    rigid_body_id_ = 0;
    time_ = 0;
}

void BirdsCore::computeForces(VectorXd &Fc, VectorXd &Ftheta)
{
    // TODO: Compute Forces here
}

bool BirdsCore::simulateOneStep()
{
    time_ += params_->timeStep;
    int nbodies = (int)bodies_.size();

    // TODO: Implement Time Integrator here

    return false;
}

void
BirdsCore::clearScene()
{
    bodies_.clear();
    templates_.clear();
    init_configurations_.clear();
}

Eigen::VectorXi
BirdsCore::addMesh(const std::string& file_name,
                   double scale,
                   const Eigen::MatrixXd& Qs)
{
    auto tup = loadOBJ(file_name);
    templates_.emplace_back(new RigidBodyTemplate(std::get<0>(tup), std::get<1>(tup), scale));

    auto rbt = templates_.back();
    init_configurations_.emplace_back(Qs);
    return addInstances(rbt, Qs);
}

std::shared_ptr<RigidBodyInstance>
BirdsCore::queryRigidBodyInstance(int32_t bid)
{
    for (auto& b : bodies_)
        if (b->bid == bid)
            return b;
    return std::shared_ptr<RigidBodyInstance>(nullptr);
}

int32_t
BirdsCore::addSingleInstance(std::shared_ptr<RigidBodyTemplate> rbt,
                             double density,
                             const Eigen::Vector3d &c,
                             const Eigen::Vector3d &theta,
                             const Eigen::Vector3d &cvel,
                             const Eigen::Vector3d &w)
{
    bodies_.emplace_back(new RigidBodyInstance(*rbt, c, theta, cvel, w, density));
    bodies_.back()->bid = rigid_body_id_++;
    return bodies_.back()->bid;
}

Eigen::VectorXi
BirdsCore::addInstances(std::shared_ptr<RigidBodyTemplate> rbt,
                        const Eigen::MatrixXd& Qs)
{
    Eigen::VectorXi ret;
    ret.resize(Qs.rows());
    for (int i = 0; i < Qs.rows(); i++) {
        double density;
        Eigen::Vector3d c, cvel, theta, w;
        density = Qs(i, 0);
        int base = 1;
        c << Qs(i, base + 0), Qs(i, base + 1), Qs(i, base + 2);
        base += 3;
        cvel << Qs(i, base + 0), Qs(i, base + 1), Qs(i, base + 2);
        base += 3;
        theta << Qs(i, base + 0), Qs(i, base + 1), Qs(i, base + 2);
        base += 3;
        w << Qs(i, base + 0), Qs(i, base + 1), Qs(i, base + 2);
        ret(i) = addSingleInstance(rbt, density, c, theta, cvel, w);
    }
    return ret;
}

}
