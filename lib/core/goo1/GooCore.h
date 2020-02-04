#ifndef PSYM_CORE_GOO1_GOOCORE_H
#define PSYM_CORE_GOO1_GOOCORE_H

#include "../PhysicsCore.h"
#include "SceneObjects.h"
#include "SimParameters.h"
#include <Eigen/StdVector>
#include <Eigen/SparseCore>
#include <memory>

namespace goo1 {

class GooCore : public PhysicsCore
{
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

    SimParameters getSimParameters() const
    {
            return *params_;
    }

    void setSimParameters(const SimParameters& nsimp)
    {
            *params_ = nsimp;
    }

    /*
     * addParticle: add a particle at (x, y)
     *
     * Returns: a ID which uniquely identifies a particle during the lifetime
     *          of the simulation (i.e. until initSimulation() is called)
     */
    uint32_t addParticle(double x, double y);

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
    queryParticle(uint32_t uid) const {
        for(const auto& p : particles_)
            if (p.uid == uid)
                return std::make_tuple(p, true);
        return std::make_tuple(Particle(Eigen::Vector2d(0.0,0.0), -1, false, false, -1), false);
    }

private:
    uint32_t particle_unique_id_;
    std::shared_ptr<SimParameters> params_;
    double time_;
    std::vector<Particle, Eigen::aligned_allocator<Particle> > particles_;
    std::vector<Connector *> connectors_;
    std::vector<Saw> saws_;

    double getTotalParticleMass(int idx) const;

    // TODO: Additional member functions
};

}

#endif
