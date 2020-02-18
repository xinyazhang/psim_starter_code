#include "GooCore.h"
#include "SimParameters.h"
#include <Eigen/Geometry>
#include <Eigen/SparseQR>
#include <algorithm>
#include <unordered_map>

using namespace Eigen;

namespace goo2 {

GooCore::GooCore()
{
    params_ = std::make_shared<SimParameters>();
}

GooCore::~GooCore()
{
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
GooCore::getCurrentMesh() const
{
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

    if (params_->floorEnabled) {
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

    for (auto c : connectors_) {
        Vector2d sourcepos = particles_[c->p1].pos;
        Vector2d destpos = particles_[c->p2].pos;
        Vector2d vec = destpos - sourcepos;
        Vector2d perp(-vec[1], vec[0]);
        perp /= perp.norm();

        double dist = (sourcepos - destpos).norm();

        Eigen::Vector3d color;
        double width;
        switch (c->getType()) {
        case SimParameters::CT_SPRING: {
            if (c->associatedBendingStencils.empty())
                color << 0.0, 0.0, 1.0;
            else
                color << 0.75, 0.5, 0.75;
            width = baselinewidth / (1.0 + 20.0 * dist * dist);

            break;
        }
        case SimParameters::CT_RIGIDROD: {
            Eigen::Vector3d color;
            if (c->associatedBendingStencils.empty())
                color << 1.0, 0.0, 1.0;
            else
                color << 1.0, 1.0, 0.3;
            width = baselinewidth;
            break;
        }
        default:
            break;
        }

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

    for (int i = 0; i < nparticles; i++) {
        double radius = baseradius * sqrt(getTotalParticleMass(i));
        radius *= (1.0 + pulsefactor * sin(pulsespeed * time_));

        Eigen::Vector3d color(0, 0, 0);

        if (particles_[i].fixed) {
            radius = baseradius;
            color << 1.0, 0, 0;
        }

        for (int j = 0; j < numcirclewedges + 2; j++) {
            vertexColors.push_back(color);
        }

        verts.push_back(Eigen::Vector3d(particles_[i].pos[0], particles_[i].pos[1], 0));

        const double PI = 3.1415926535898;
        for (int j = 0; j <= numcirclewedges; j++) {
            verts.emplace_back(particles_[i].pos[0] + radius * cos(2 * PI * j / numcirclewedges),
                               particles_[i].pos[1] + radius * sin(2 * PI * j / numcirclewedges),
                               0);
        }

        for (int j = 0; j <= numcirclewedges; j++) {
            faces.emplace_back(idx, idx + j + 1, idx + 1 + ((j + 1) % (numcirclewedges + 1)));
        }

        idx += numcirclewedges + 2;
    }

    for (const auto& saw : saws_) {
        double outerradius = saw.radius;
        double innerradius = (1.0 - sawdepth) * outerradius;

        Eigen::Vector3d color(0.5, 0.5, 0.5);

        int spokes = 2 * sawteeth;
        for (int j = 0; j < spokes + 2; j++) {
            vertexColors.push_back(color);
        }

        verts.emplace_back(saw.pos[0], saw.pos[1], 0);

        const double PI = 3.1415926535898;
        for (int i = 0; i <= spokes; i++) {
            double radius = (i % 2 == 0) ? innerradius : outerradius;
            verts.emplace_back(saw.pos[0] + radius * cos(2 * PI * i / spokes + sawangspeed * time_),
                               saw.pos[1] + radius * sin(2 * PI * i / spokes + sawangspeed * time_),
                               0.0);
        }

        for (int j = 0; j <= spokes; j++) {
            faces.emplace_back(idx, idx + j + 1, idx + 1 + ((j + 1) % (spokes + 1)));
        }

        idx += spokes + 2;
    }

    Eigen::MatrixXd renderQ;
    Eigen::MatrixXi renderF;
    Eigen::MatrixXd renderC;

    renderQ.resize(verts.size(), 3);
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

void GooCore::initSimulation()
{
    particle_unique_id_ = 0;
    time_ = 0;
    particles_.clear();
    for (std::vector<Connector*>::iterator it = connectors_.begin(); it != connectors_.end(); ++it)
        delete *it;
    connectors_.clear();
    saws_.clear();
    bendingStencils_.clear();
    // TODO: other initializatios
}

bool GooCore::simulateOneStep()
{
    VectorXd q, lambda, v;
    buildConfiguration(q, lambda, v);
    numericalIntegration(q, lambda, v);
    unbuildConfiguration(q, lambda, v);

    pruneOverstrainedSprings();
    deleteSawedObjects();
    time_ += params_->timeStep;
    return false;
}

Eigen::VectorXi GooCore::addParticle(double x, double y)
{
    Eigen::VectorXi ret;
    std::vector<int> ids;
    Vector2d newpos(x, y);
    double mass = params_->particleMass;
    if (params_->particleFixed)
        mass = std::numeric_limits<double>::infinity();

    int newid = particles_.size();
    particles_.emplace_back(newpos, mass, params_->particleFixed, false);
    particles_.back().uid = particle_unique_id_++;
    ids.emplace_back(particles_.back().uid);

    int numparticles = particles_.size() - 1;

    for (int i = 0; i < numparticles; i++) {
        if (particles_[i].inert)
            continue;
        Vector2d pos = particles_[i].pos;
        double dist = (pos - newpos).norm();
        if (dist <= params_->maxSpringDist) {
            switch (params_->connectorType) {
            case SimParameters::CT_SPRING: {
                connectors_.push_back(new Spring(newid, i, 0, params_->springStiffness / dist, dist, true));
                break;
            }

	    // TODO: Handle Rigid rods and flexible rods.
            case SimParameters::CT_RIGIDROD:
                break;
            case SimParameters::CT_FLEXROD:
                break;
            default:
                break;
            }
        }
    }
    ret.resize(ids.size());
    for (int i = 0; i < ret.size(); i++)
        ret(i) = ids[i];
    return ret;
}

void GooCore::addSaw(double x, double y)
{
    saws_.emplace_back(Vector2d(x, y), params_->sawRadius);
}

Eigen::VectorXi
GooCore::queryConnectivity(Eigen::VectorXi from,
                           Eigen::VectorXi to)
{
    int N = from.rows();
    Eigen::VectorXi ret(from.rows());
    std::unordered_map<int, int> uid2index;
    for (size_t i = 0; i < particles_.size(); i++) {
        uid2index[particles_[i].uid] = static_cast<int>(i);
    }
    for (int i = 0; i < N; i++) {
        from(i) = uid2index[from(i)];
        to(i) = uid2index[to(i)];

        ret(i) = 0;
        for (const auto& c : connectors_) {
            if (c->p1 == from(i) && c->p2 == to(i)) {
                ret(i) = 1;
                break;
            }
            if (c->p2 == from(i) && c->p1 == to(i)) {
                ret(i) = 1;
                break;
            }
        }
    }
    return ret;
}

double GooCore::getTotalParticleMass(int idx) const
{
    double mass = particles_[idx].mass;
    for (auto it = connectors_.cbegin(); it != connectors_.cend(); ++it) {
        if ((*it)->p1 == idx || (*it)->p2 == idx)
            mass += 0.5 * (*it)->mass;
    }
    return mass;
}

int GooCore::getNumRigidRods() const
{
    int nrods = 0;
    for (const auto& c : connectors_) {
        if (c->getType() == SimParameters::CT_RIGIDROD)
            nrods++;
    }
    return nrods;
}

void GooCore::buildConfiguration(VectorXd& q, VectorXd& lambda, VectorXd& v)
{
    int ndofs = 2 * particles_.size();
    q.resize(ndofs);
    v.resize(ndofs);

    int nrods = getNumRigidRods();

    for (int i = 0; i < (int)particles_.size(); i++) {
        q.segment<2>(2 * i) = particles_[i].pos;
        v.segment<2>(2 * i) = particles_[i].vel;
    }

    // TODO: Fill up the initial guessing of lambda
}

void GooCore::unbuildConfiguration(const VectorXd& q, const VectorXd& lambda, const VectorXd& v)
{
    int ndofs = q.size();
    assert(ndofs == int(2 * particles_.size()));
    int nrods = lambda.size();
    assert(nrods == getNumRigidRods());

    for (int i = 0; i < ndofs / 2; i++) {
        particles_[i].pos = q.segment<2>(2 * i);
        particles_[i].vel = v.segment<2>(2 * i);
    }

    // TODO: Save current lambda as the initial guessing of the next
    //       time step.
}

void GooCore::numericalIntegration(VectorXd& q, VectorXd& lambda, VectorXd& v)
{
    VectorXd F;
    SparseMatrix<double> H;
    SparseMatrix<double> Minv;

    computeMassInverse(Minv);

    VectorXd oldq = q;

    switch (params_->constraintHandling) {
    // TODO: Handling constraints
    case SimParameters::CH_PENALTY:
        q += params_->timeStep * v;
        computeForceAndHessian(q, oldq, F, H);
        // TODO: Compute Penalty Forces
        // computePenaltyForces(q, F);
        v += params_->timeStep * Minv * F;
        break;
    case SimParameters::CH_STEPPROJECT:
        break;
    case SimParameters::CH_LAGRANGEMULT:
        break;
    }
}

void GooCore::computeMassInverse(Eigen::SparseMatrix<double>& Minv)
{
    int ndofs = 2 * int(particles_.size());

    Minv.resize(ndofs, ndofs);
    Minv.setZero();

    std::vector<Eigen::Triplet<double>> Minvcoeffs;
    for (int i = 0; i < ndofs / 2; i++) {
        Minvcoeffs.emplace_back(2 * i, 2 * i, 1.0 / getTotalParticleMass(i));
        Minvcoeffs.emplace_back(2 * i + 1, 2 * i + 1, 1.0 / getTotalParticleMass(i));
    }

    Minv.setFromTriplets(Minvcoeffs.begin(), Minvcoeffs.end());
}

void
GooCore::computeForceAndHessian(const VectorXd& q,
                                const VectorXd& qprev,
                                Eigen::VectorXd& F,
                                SparseMatrix<double>& H)
{
    F.resize(q.size());
    F.setZero();
    H.resize(q.size(), q.size());
    H.setZero();

    std::vector<Eigen::Triplet<double>> Hcoeffs;
    if (params_->gravityEnabled)
        processGravityForce(F);
    if (params_->springsEnabled)
        processSpringForce(q, F, Hcoeffs);
    if (params_->dampingEnabled)
        processDampingForce(q, qprev, F, Hcoeffs);
    if (params_->floorEnabled)
        processFloorForce(q, qprev, F, Hcoeffs);
    if (params_->bendingEnabled)
        processBendingForce(q, F);

    H.setFromTriplets(Hcoeffs.begin(), Hcoeffs.end());
}

void GooCore::processGravityForce(VectorXd& F)
{
    int nparticles = (int)particles_.size();
    for (int i = 0; i < nparticles; i++) {
        if (!particles_[i].fixed) {
            F[2 * i + 1] += params_->gravityG * getTotalParticleMass(i);
        }
    }
}

void GooCore::processBendingForce(const VectorXd& q, VectorXd& F)
{
    // TODO: Bending energy
}

void GooCore::processSpringForce(const Eigen::VectorXd& q,
                                 Eigen::VectorXd& F,
                                 std::vector<Eigen::Triplet<double>>& H)
{
    int nsprings = (int)connectors_.size();

    for (int i = 0; i < nsprings; i++) {
        if (connectors_[i]->getType() != SimParameters::CT_SPRING)
            continue;
        Spring& s = *(Spring*)connectors_[i];
        Vector2d p1 = q.segment<2>(2 * s.p1);
        Vector2d p2 = q.segment<2>(2 * s.p2);
        double dist = (p2 - p1).norm();
        Vector2d localF = s.stiffness * (dist - s.restlen) / dist * (p2 - p1);
        F.segment<2>(2 * s.p1) += localF;
        F.segment<2>(2 * s.p2) -= localF;

        Matrix2d I;
        I << 1, 0, 0, 1;
        Matrix2d localH = s.stiffness * (1.0 - s.restlen / dist) * I;
        localH += s.stiffness * s.restlen * (p2 - p1) * (p2 - p1).transpose() / dist / dist / dist;

        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++) {
                H.emplace_back(2 * s.p1 + j, 2 * s.p1 + k, localH.coeff(j, k));
                H.emplace_back(2 * s.p2 + j, 2 * s.p2 + k, localH.coeff(j, k));
                H.emplace_back(2 * s.p1 + j, 2 * s.p2 + k, -localH.coeff(j, k));
                H.emplace_back(2 * s.p2 + j, 2 * s.p1 + k, -localH.coeff(j, k));
            }
    }
}

void GooCore::processDampingForce(const VectorXd& q,
                                  const VectorXd& qprev,
                                  VectorXd& F,
                                  std::vector<Eigen::Triplet<double>>& H)
{
    int nsprings = (int)connectors_.size();

    for (int i = 0; i < nsprings; i++) {
        if (connectors_[i]->getType() != SimParameters::CT_SPRING)
            continue;
        Spring& s = *(Spring*)connectors_[i];
        Vector2d p1 = q.segment<2>(2 * s.p1);
        Vector2d p2 = q.segment<2>(2 * s.p2);
        Vector2d p1prev = qprev.segment<2>(2 * s.p1);
        Vector2d p2prev = qprev.segment<2>(2 * s.p2);

        Vector2d relvel = (p2 - p2prev) / params_->timeStep - (p1 - p1prev) / params_->timeStep;
        Vector2d localF = params_->dampingStiffness * relvel;
        F.segment<2>(2 * s.p1) += localF;
        F.segment<2>(2 * s.p2) -= localF;

        Matrix2d I;
        I << 1, 0, 0, 1;
        Matrix2d localH = params_->dampingStiffness * I / params_->timeStep;

        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++) {
                H.emplace_back(2 * s.p1 + j, 2 * s.p1 + k, localH.coeff(j, k));
                H.emplace_back(2 * s.p2 + j, 2 * s.p2 + k, localH.coeff(j, k));
                H.emplace_back(2 * s.p1 + j, 2 * s.p2 + k, -localH.coeff(j, k));
                H.emplace_back(2 * s.p2 + j, 2 * s.p1 + k, -localH.coeff(j, k));
            }
    }
}

void GooCore::processFloorForce(const VectorXd& q,
                                const VectorXd& qprev,
                                VectorXd& F,
                                std::vector<Eigen::Triplet<double>>& H)
{
    int nparticles = particles_.size();

    double basestiffness = 10000;
    double basedrag = 1000.0;

    for (int i = 0; i < nparticles; i++) {
        if (q[2 * i + 1] < -0.5 && !particles_[i].fixed) {
            double vel = (q[2 * i + 1] - qprev[2 * i + 1]) / params_->timeStep;
            double dist = -0.5 - q[2 * i + 1];

            F[2 * i + 1] += basestiffness * dist - basedrag * dist * vel;

            H.emplace_back(2 * i + 1, 2 * i + 1, basestiffness - 0.5 * basedrag / params_->timeStep + basedrag * qprev[2 * i + 1] / params_->timeStep - 2.0 * basedrag * q[2 * i + 1] / params_->timeStep);
        }
    }
}

double
GooCore::ptSegmentDist(const Vector2d& p, const Vector2d& q1, const Vector2d& q2)
{
    double t = (p - q1).dot(q2 - q1) / (q2 - q1).dot(q2 - q1);
    t = std::max(0.0, std::min(t, 1.0));
    double linedistsq = (q1 + t * (q2 - q1) - p).squaredNorm();
    return sqrt(linedistsq);
}

void GooCore::detectSawedConnectors(std::set<int>& connectorsToDelete)
{
    for (int i = 0; i < (int)connectors_.size(); i++) {
        Vector2d pos1 = particles_[connectors_[i]->p1].pos;
        Vector2d pos2 = particles_[connectors_[i]->p2].pos;
        double maxx = std::max(pos1[0], pos2[0]);
        double minx = std::min(pos1[0], pos2[0]);
        double maxy = std::max(pos1[1], pos2[1]);
        double miny = std::min(pos1[1], pos2[1]);
        for (std::vector<Saw>::iterator saw = saws_.begin(); saw != saws_.end(); ++saw) {
            Vector2d sawpos = saw->pos;
            double sawr = saw->radius;

            if (sawpos[0] - sawr > maxx || sawpos[0] + sawr < minx || sawpos[1] - sawr > maxy || sawpos[1] + sawr < miny)
                continue;

            double sawspringdist = ptSegmentDist(sawpos, pos1, pos2);
            if (sawspringdist <= sawr) {
                connectorsToDelete.insert(i);
                break;
            }
        }
    }
}

void GooCore::detectSawedParticles(std::set<int>& particlesToDelete)
{
    for (int i = 0; i < (int)particles_.size(); i++) {
        Vector2d partpos = particles_[i].pos;

        if (!(fabs(partpos[0]) < 2 && fabs(partpos[1]) < 2)) {
            particlesToDelete.insert(i);
            continue;
        }

        for (std::vector<Saw>::iterator it = saws_.begin(); it != saws_.end(); ++it) {
            Vector2d sawpos = it->pos;
            double sqdist = (sawpos - partpos).squaredNorm();
            if (sqdist < it->radius * it->radius) {
                particlesToDelete.insert(i);
                break;
            }
        }
    }
}

void GooCore::deleteSawedObjects()
{
    std::set<int> particlestodelete;
    std::set<int> connectorstodelete;
    std::set<int> bendingtodelete;
    detectSawedParticles(particlestodelete);
    detectSawedConnectors(connectorstodelete);

    std::vector<Particle, Eigen::aligned_allocator<Particle>> newparticles;
    std::vector<Connector*> newconnectors;
    std::vector<BendingStencil> newbending;
    std::vector<int> remainingparticlemap;
    std::vector<int> remainingbendingmap;

    if (!particlestodelete.empty()) {
        for (int i = 0; i < (int)connectors_.size(); i++) {
            if (particlestodelete.count(connectors_[i]->p1) || particlestodelete.count(connectors_[i]->p2))
                connectorstodelete.insert(i);
        }

        for (int i = 0; i < (int)particles_.size(); i++) {
            if (particlestodelete.count(i) == 0) {
                remainingparticlemap.push_back(newparticles.size());
                newparticles.push_back(particles_[i]);
            } else
                remainingparticlemap.push_back(-1);
        }
    }
    if (!connectorstodelete.empty()) {
        for (std::set<int>::iterator it = connectorstodelete.begin(); it != connectorstodelete.end(); ++it) {
            for (std::set<int>::iterator bit = connectors_[*it]->associatedBendingStencils.begin(); bit != connectors_[*it]->associatedBendingStencils.end(); ++bit) {
                bendingtodelete.insert(*bit);
            }
        }
        for (int i = 0; i < (int)connectors_.size(); i++) {
            if (connectorstodelete.count(i) == 0) {
                newconnectors.push_back(connectors_[i]);
            } else
                delete connectors_[i];
        }
    }
    if (!bendingtodelete.empty()) {
        int newidx = 0;
        for (int i = 0; i < (int)bendingStencils_.size(); i++) {
            if (bendingtodelete.count(i) == 0) {
                newbending.push_back(bendingStencils_[i]);
                remainingbendingmap.push_back(newidx++);
            } else
                remainingbendingmap.push_back(-1);
        }
    }

    if (!connectorstodelete.empty() || !particlestodelete.empty()) {
        if (!connectorstodelete.empty())
            connectors_ = newconnectors;
        if (!bendingtodelete.empty()) {
            bendingStencils_ = newbending;
            for (std::vector<Connector*>::iterator it = connectors_.begin(); it != connectors_.end(); ++it) {
                std::set<int> newass;
                for (std::set<int>::iterator sit = (*it)->associatedBendingStencils.begin(); sit != (*it)->associatedBendingStencils.end(); ++sit) {
                    if (bendingtodelete.count(*sit) == 0)
                        newass.insert(remainingbendingmap[*sit]);
                }
                (*it)->associatedBendingStencils = newass;
            }
        }
        if (!particlestodelete.empty()) {
            particles_ = newparticles;
            for (std::vector<Connector*>::iterator it = connectors_.begin(); it != connectors_.end(); ++it) {
                (*it)->p1 = remainingparticlemap[(*it)->p1];
                (*it)->p2 = remainingparticlemap[(*it)->p2];
            }
            for (std::vector<BendingStencil>::iterator it = bendingStencils_.begin(); it != bendingStencils_.end(); ++it) {
                it->p1 = remainingparticlemap[it->p1];
                it->p2 = remainingparticlemap[it->p2];
                it->p3 = remainingparticlemap[it->p3];
            }
        }
    }
}

void GooCore::pruneOverstrainedSprings()
{
    int nsprings = connectors_.size();

    std::vector<int> toremove;
    for (int i = 0; i < nsprings; i++) {
        if (connectors_[i]->getType() != SimParameters::CT_SPRING)
            continue;

        Spring& s = *(Spring*)connectors_[i];
        if (s.canSnap) {
            Vector2d srcpos = particles_[s.p1].pos;
            Vector2d dstpos = particles_[s.p2].pos;
            double dist = (dstpos - srcpos).norm();

            double strain = (dist - s.restlen) / s.restlen;
            if (strain > params_->maxSpringStrain)
                toremove.push_back(i);
        }
    }

    for (std::vector<int>::reverse_iterator it = toremove.rbegin(); it != toremove.rend(); ++it) {
        assert(connectors_[*it]->associatedBendingStencils.empty());
        delete connectors_[*it];
        connectors_.erase(connectors_.begin() + *it);
    }
}

// TODO: Implmentation of additional member functions

}
