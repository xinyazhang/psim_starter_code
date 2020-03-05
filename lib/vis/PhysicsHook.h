#ifndef PSIM_LIB_VIS_PHYSICS_HOOK_H
#define PSIM_LIB_VIS_PHYSICS_HOOK_H

#include <mutex>
#include <thread>
#include <memory>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

class PhysicsCore;

/*
 * PhysicsHook
 *
 * The Interface of shim classes between igl::Viewer and the PhysicsCore
 */
class PhysicsHook
{
public:
    PhysicsHook(PhysicsCore* core);

    virtual ~PhysicsHook();

    /*
     * Runs once when the program starts; can be used to add GUI elements for simulation parameters, etc.
     */
    virtual void drawGUI(igl::opengl::glfw::imgui::ImGuiMenu &menu) = 0;

    /*
     * Runs once when the simulation is initialized, and again each time the user resets the simulation.
     */
    virtual void initSimulation();

    /*
     * Called every once in a while (and always before every simulation step) even if the simulation is paused.
     * Use this to update the visualization in response to user input etc.
     */
    virtual void tick();

    /*
     * Update the rendering data structures here. This method will be called in alternation with simulateOneStep().
     * This method blocks rendering in the viewer, so do *not* do extensive computation here (leave it to
     * simulateOneStep()).
     */
    virtual void updateRenderGeometry();

    /*
     * Perform any actual rendering here. This method *must* be thread-safe with respect to simulateOneStep().
     * This method runs in the same thread as the viewer and blocks user IO, so there really should not be any
     * extensive computation here or the UI will lag/become unresponsive (the whole reason the simulation itself
     * is in its own thread.)
     */
    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer);

    /*
     * Called when the user clicks on the simulation panel.
     * This method is called in the *rendering thread*. If you need to make changes to the simulation state, you
     * should stash the mouse click in a message queue and deal with it in the simulation thread.
     */
    virtual void mouseClicked(double x, double y, int button) {}

    /*
     * Runs the simulation, if it has been paused (or never started).
     */
    void run();

    /*
     * Resets the simulation (and leaves it in a paused state; call run() to start it).
     */
    void reset();

    /*
     * Pause a running simulation. The simulation will pause at the end of its current "step"; this method will not
     * interrupt simulateOneStep mid-processing.
     */
    void pause();

    bool isPaused();

    void render(igl::opengl::glfw::Viewer &viewer);

    void useGeometry(Eigen::MatrixXd renderQ, Eigen::MatrixXi renderF, Eigen::MatrixXd renderC);

protected:
    PhysicsCore* core_;

    void runSimThread();
    void killSimThread();

    std::unique_ptr<std::thread> sim_thread;
    bool please_pause;
    bool please_die;
    bool running;
    std::mutex render_mutex;
    std::mutex status_mutex;
    std::mutex message_mutex;

    Eigen::MatrixXd renderQ_;
    Eigen::MatrixXi renderF_;
    Eigen::MatrixXd renderC_;
    bool render_data_dirty_ = true;
};

#endif
