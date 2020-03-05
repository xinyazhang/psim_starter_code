#include "PhysicsHook.h"
#include <core/PhysicsCore.h>
#include <iostream>

PhysicsHook::PhysicsHook(PhysicsCore* core)
	: core_(core)
{
}

PhysicsHook::~PhysicsHook()
{
	killSimThread();
}

void
PhysicsHook::initSimulation()
{
	core_->initSimulation();
}

void
PhysicsHook::tick()
{
}

void
PhysicsHook::updateRenderGeometry()
{
	std::tie(renderQ_, renderF_, renderC_) = core_->getCurrentMesh();
	render_data_dirty_ = true;
}

void
PhysicsHook::renderRenderGeometry(igl::opengl::glfw::Viewer &viewer)
{
	viewer.data().clear();
    if (renderQ_.rows() > 0 && renderF_.rows() > 0) {
        viewer.data().set_mesh(renderQ_, renderF_);
    }
	if (renderC_.size() > 0)
		viewer.data().set_colors(renderC_);
	render_data_dirty_ = false;
}

/*
 * Runs the simulation, if it has been paused (or never started).
 */
void
PhysicsHook::run()
{
	status_mutex.lock();
	please_pause = false;
	status_mutex.unlock();
}

/*
 * Resets the simulation (and leaves it in a paused state; call run() to start it).
 */
void
PhysicsHook::reset()
{
	killSimThread();
	please_die = running = false;
	please_pause = true;
	core_->initSimulation();
	updateRenderGeometry();
	sim_thread.reset(new std::thread(&PhysicsHook::runSimThread, this));
}

/*
 * Pause a running simulation. The simulation will pause at the end of its current "step"; this method will not
 * interrupt simulateOneStep mid-processing.
 */
void
PhysicsHook::pause()
{
	status_mutex.lock();
	please_pause = true;
	status_mutex.unlock();
}

bool
PhysicsHook::isPaused()
{
	bool ret = false;
	status_mutex.lock();
	if(running && please_pause)
		ret = true;
	status_mutex.unlock();
	return ret;
}

void
PhysicsHook::useGeometry(Eigen::MatrixXd renderQ, Eigen::MatrixXi renderF, Eigen::MatrixXd renderC)
{
	renderQ_ = std::move(renderQ);
	renderF_ = std::move(renderF);
	renderC_ = std::move(renderC);
}

void
PhysicsHook::render(igl::opengl::glfw::Viewer &viewer)
{
	render_mutex.lock();
	renderRenderGeometry(viewer);
	render_mutex.unlock();
}

void
PhysicsHook::runSimThread()
{
	status_mutex.lock();
	running = true;
	status_mutex.unlock();

	bool done = false;
	while (!done) {
		tick();

		status_mutex.lock();
		bool pausenow = please_pause;
		status_mutex.unlock();
		if (pausenow) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		} else {
			done = core_->simulateOneStep();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		render_mutex.lock();
		updateRenderGeometry();
		render_mutex.unlock();
		status_mutex.lock();
		if (please_die)
			done = true;
		status_mutex.unlock();
	}

	status_mutex.lock();
	running = false;
	status_mutex.unlock();
}

void
PhysicsHook::killSimThread()
{
	if (sim_thread) {
		status_mutex.lock();
		please_die = true;
		status_mutex.unlock();
		sim_thread->join();
		sim_thread.reset();
	}
}
