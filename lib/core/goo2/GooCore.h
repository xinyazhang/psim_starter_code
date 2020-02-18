#ifndef PSYM_CORE_GOO2_GOOCORE_H
#define PSYM_CORE_GOO2_GOOCORE_H

#include "../PhysicsCore.h"
#include "SceneObjects.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/StdVector>

#include <memory>
#include <stdint.h>

namespace goo2 {

class SimParameters;

class GooCore : public PhysicsCore {
public:
    GooCore();
    ~GooCore();

    virtual void initSimulation() override;

    virtual bool simulateOneStep() override;

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
    getCurrentMesh() const override;

    std::shared_ptr<SimParameters> getPointerToSimParameters()
    {
        return params_;
    }

    /*
     * addParticle: add a particle at (x, y)
     *
     * Returns: a list of non-negative IDs which uniquely identify  the newly
     *          added particles during the lifetime of the simulation (i.e.
     *          until initSimulation() is called)
     *
     * Note: A flex rod returns mutliple particles.
     */
    Eigen::VectorXi addParticle(double x, double y);

    /*
     * addSaw: add a saw at (x, y)
     */
    void addSaw(double x, double y);

    /*
     * queryParticle: locate the particle with specific UID
     *
     * Returns: Tuple (particle, valid).
     *          valid must be false if the particle with given uid cannot be
     *          found.
     */
    std::tuple<Particle, bool>
    queryParticle(int32_t uid) const
    {
        for (const auto& p : particles_)
            if (p.uid == uid)
                return std::make_tuple(p, true);
        return std::make_tuple(Particle(Eigen::Vector2d(0.0, 0.0), -1, false, false), false);
    }


    /*
     * queryConnectivity: Detect whether from(i) is connected with to(i).
     *
     * Inputs: VectorXi with particle UIDs
     * Returns: Eigen::VectorXi. Each element ret(i) represents if Particle
     *          from(i) connected with Particle to(i).
     *
     *          Any none zero value means connected.
     */
    Eigen::VectorXi
    queryConnectivity(Eigen::VectorXi from,
                      Eigen::VectorXi to);

private:
    int32_t particle_unique_id_;
    std::shared_ptr<SimParameters> params_;
    double time_;
    std::vector<Particle, Eigen::aligned_allocator<Particle>> particles_;
    std::vector<Connector*> connectors_;
    std::vector<Saw> saws_;
    std::vector<BendingStencil> bendingStencils_;

    double getTotalParticleMass(int idx) const;
    int getNumRigidRods() const;

    void buildConfiguration(Eigen::VectorXd& q, Eigen::VectorXd& lambda, Eigen::VectorXd& v);
    void unbuildConfiguration(const Eigen::VectorXd& q, const Eigen::VectorXd& lambda, const Eigen::VectorXd& v);

    void computeMassInverse(Eigen::SparseMatrix<double>& Minv);
    void numericalIntegration(Eigen::VectorXd& q, Eigen::VectorXd& lambda, Eigen::VectorXd& v);

    void computeForceAndHessian(const Eigen::VectorXd& q,
                                const Eigen::VectorXd& qprev,
                                Eigen::VectorXd& F,
                                Eigen::SparseMatrix<double>& H);
    void processGravityForce(Eigen::VectorXd& F);
    void processBendingForce(const Eigen::VectorXd& q, Eigen::VectorXd& F);
    void processSpringForce(const Eigen::VectorXd& q, Eigen::VectorXd& F,
                            std::vector<Eigen::Triplet<double>>& H);
    void processDampingForce(const Eigen::VectorXd& q,
                             const Eigen::VectorXd& qprev, Eigen::VectorXd& F,
                             std::vector<Eigen::Triplet<double>>& H);
    void processFloorForce(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& qprev, Eigen::VectorXd& F,
                           std::vector<Eigen::Triplet<double>>& H);

    double ptSegmentDist(const Eigen::Vector2d& p, const Eigen::Vector2d& q1,
                         const Eigen::Vector2d& q2);
    void detectSawedConnectors(std::set<int>& connectorsToDelete);
    void detectSawedParticles(std::set<int>& particlesToDelete);
    void deleteSawedObjects();
    void pruneOverstrainedSprings();

    // TODO: Additional member functions
};

}

#endif
