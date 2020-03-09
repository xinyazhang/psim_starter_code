#include "GooCore.h"
#include <Eigen/Sparse>
#include <algorithm>
#include <iostream>

using namespace Eigen;

namespace goo1 {

GooCore::GooCore()
{
	params_ = std::make_shared<SimParameters>();
}

GooCore::~GooCore()
{
}

void GooCore::initSimulation()
{
    particle_unique_id_ = 0;
    time_ = 0;
    particles_.clear();
    for (auto c: connectors_)
        delete c;
    connectors_.clear();
    saws_.clear();
    // TODO: other initializatios
}

bool GooCore::simulateOneStep()
{
    // TODO: simulate
    return false;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
GooCore::getCurrentMesh() const
{
    Eigen::MatrixXd renderQ;
    Eigen::MatrixXi renderF;
    Eigen::MatrixXd renderC;

    double baseradius = 0.02;
    double pulsefactor = 0.1;
    double pulsespeed = 50.0;

    int sawteeth = 20;
    double sawdepth = 0.1;
    double sawangspeed = 10.0;

    double baselinewidth = 0.005;

    int numcirclewedges = 20;

    // this is terrible. But, easiest to get up and running

    std::vector<Eigen::Vector3d> verts;
    std::vector<Eigen::Vector3d> vertexColors;
    std::vector<Eigen::Vector3i> faces;

    int idx = 0;

    double eps = 1e-4;


    if(params_->floorEnabled) {
        for (int i = 0; i < 6; i++) {
            vertexColors.emplace_back(0.3, 1.0, 0.3);
        }

        verts.emplace_back(-1, -0.5, eps);
        verts.emplace_back(1, -0.5, eps);
        verts.emplace_back(-1, -1, eps);

        faces.emplace_back(idx, idx + 1, idx + 2);

        verts.emplace_back(-1, -1, eps);
        verts.emplace_back(1, -0.5, eps);
        verts.emplace_back(1, -1, eps);
        faces.emplace_back(idx + 3, idx + 4, idx + 5);
        idx += 6;
    }

    for (auto c : connectors_)
    {
        Eigen::Vector3d color;
        if (c->associatedBendingStencils.empty())
            color << 0.0, 0.0, 1.0;
        else
            color << 0.75, 0.5, 0.75;
        Vector2d sourcepos = particles_[c->p1].pos;
        Vector2d destpos = particles_[c->p2].pos;

        Vector2d vec = destpos - sourcepos;
        Vector2d perp(-vec[1], vec[0]);
        perp /= perp.norm();

        double dist = (sourcepos - destpos).norm();

        double width = baselinewidth / (1.0 + 20.0 * dist * dist);

        for (int i = 0; i < 4; i++)
            vertexColors.emplace_back(color);

        verts.emplace_back(sourcepos[0] + width * perp[0], sourcepos[1] + width * perp[1], -eps);
        verts.emplace_back(sourcepos[0] - width * perp[0], sourcepos[1] - width * perp[1], -eps);
        verts.emplace_back(destpos[0] + width * perp[0], destpos[1] + width * perp[1], -eps);
        verts.emplace_back(destpos[0] - width * perp[0], destpos[1] - width * perp[1], -eps);

        faces.emplace_back(idx, idx + 1, idx + 2);
        faces.emplace_back(idx + 2, idx + 1, idx + 3);
        idx += 4;
    }

    int nparticles = particles_.size();

    for(int i=0; i<nparticles; i++)
    {
        double radius = baseradius*sqrt(getTotalParticleMass(i));
        radius *= (1.0 + pulsefactor*sin(pulsespeed*time_));

        Eigen::Vector3d color(0,0,0);

        if(particles_[i].fixed) {
            radius = baseradius;
            color << 1.0, 0, 0;
        }

        for (int j = 0; j < numcirclewedges + 2; j++) {
            vertexColors.push_back(color);
        }


        verts.push_back(Eigen::Vector3d(particles_[i].pos[0], particles_[i].pos[1], 0));

        const double PI = 3.1415926535898;
        for (int j = 0; j <= numcirclewedges; j++) {
            verts.emplace_back(particles_[i].pos[0] + radius * cos(2 * PI*j / numcirclewedges),
                               particles_[i].pos[1] + radius * sin(2 * PI*j / numcirclewedges),
			       0);
        }

        for (int j = 0; j <= numcirclewedges; j++) {
            faces.emplace_back(idx, idx + j + 1, idx + 1 + ((j + 1) % (numcirclewedges + 1)));
        }

        idx += numcirclewedges + 2;
    }

    for(const auto& saw : saws_) {
        double outerradius = saw.radius;
        double innerradius = (1.0-sawdepth)*outerradius;

        Eigen::Vector3d color(0.5,0.5,0.5);

        int spokes = 2*sawteeth;
        for (int j = 0; j < spokes + 2; j++)
        {
            vertexColors.push_back(color);
        }

        verts.emplace_back(saw.pos[0], saw.pos[1], 0);

        const double PI = 3.1415926535898;
        for (int i = 0; i <= spokes; i++) {
            double radius = (i % 2 == 0) ? innerradius : outerradius;
            verts.emplace_back(saw.pos[0] + radius * cos(2 * PI*i / spokes + sawangspeed*time_),
                               saw.pos[1] + radius * sin(2 * PI*i / spokes + sawangspeed*time_),
                               0.0);
        }

        for (int j = 0; j <= spokes; j++) {
            faces.emplace_back(idx, idx + j + 1, idx + 1 + ((j + 1) % (spokes + 1)));
        }

        idx += spokes + 2;
    }

    renderQ.resize(verts.size(),3);
    renderC.resize(vertexColors.size(), 3);
    for (int i = 0; i < verts.size(); i++) {
        renderQ.row(i) = verts[i];
        renderC.row(i) = vertexColors[i];
    }
    renderF.resize(faces.size(), 3);
    for (int i = 0; i < faces.size(); i++)
        renderF.row(i) = faces[i];
    return std::make_tuple(renderQ, renderF, renderC);
}


uint32_t GooCore::addParticle(double x, double y)
{
    Vector2d newpos(x,y);
    double mass = params_->particleMass;
    if(params_->particleFixed)
        mass = std::numeric_limits<double>::infinity();

    int newid = particles_.size();
    int ret = particle_unique_id_;
    particles_.emplace_back(newpos, mass, params_->particleFixed, false, ret);
    particle_unique_id_++;

    // TODO: Add connectors
    return ret;
}

void GooCore::addSaw(double x, double y)
{
    // TODO
}

double GooCore::getTotalParticleMass(int idx) const
{
    double mass = particles_[idx].mass;
    for(const auto c : connectors_)
    {
        if(c->p1 == idx || c->p2 == idx)
            mass += 0.5 * c->mass;
    }
    return mass;
}

// TODO: Additional member functions

}
