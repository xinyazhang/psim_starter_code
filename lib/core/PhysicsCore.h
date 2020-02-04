#ifndef PHYSICS_CORE_H
#define PHYSICS_CORE_H

#include <tuple>
#include <Eigen/Core>

/*
 * PhysicsCore
 * 
 * The Interface class of core functions for physical simulation.
 * Different projects will extend this class accordingly.
 * 
 * The automatic testing will only test against the subclass of this interface class.
 */
class PhysicsCore
{
public:
    PhysicsCore();

    virtual ~PhysicsCore();

    /*
     * Runs once when the simulation is initialized, and again each time the user resets the simulation.
     */
    virtual void initSimulation() = 0;

    /*
     * Takes one simulation "step." You can do whatever you want here, but the granularity of your computation should
     * be small enough that the user can view/pause/kill the simulation at interactive rates.
     * This method *must* be thread-safe with respect to renderRenderGeometry() (easiest is to not touch any rendering
     * data structures at all).
     */
    virtual bool simulateOneStep() = 0;

    /*
     * Return the mesh vertices, mesh faces, and mesh vertex color.
     * 
     * Mesh color can be empty, in this case the visualizer should use default
     * color.
     */
    virtual
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
    getCurrentMesh() const = 0;
};

#endif
