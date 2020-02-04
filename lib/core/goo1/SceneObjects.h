#ifndef PSYM_CORE_GOO1_SCENEOBJECTS_H
#define PSYM_CORE_GOO1_SCENEOBJECTS_H

#include <stdint.h>
#include "SimParameters.h"
#include <Eigen/Core>
#include <set>

namespace goo1 {

struct Particle
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Particle(Eigen::Vector2d pos, double mass, bool isFixed, bool isInert, uint32_t uid)
	: pos(pos), mass(mass), fixed(isFixed), inert(isInert), uid(uid)
    {
        vel.setZero();
        prevpos = pos;
    }

    Eigen::Vector2d pos;
    Eigen::Vector2d prevpos;
    Eigen::Vector2d vel;
    double mass;
    bool fixed;
    bool inert;

    uint32_t uid;
};

struct Connector
{
public:
    Connector(int p1,int p2, double mass) : p1(p1), p2(p2), mass(mass) {}
    virtual ~Connector() {}

    int p1;
    int p2;
    double mass;

    std::set<int> associatedBendingStencils;
};

struct Spring : public Connector
{
public:
    Spring(int p1, int p2, double mass, double stiffness, double restlen, bool canSnap) : Connector(p1, p2, mass), stiffness(stiffness), restlen(restlen), canSnap(canSnap) {}

    double stiffness;
    double restlen;
    bool canSnap;
};

struct Saw
{
public:
    Saw(Eigen::Vector2d pos, double radius) : pos(pos), radius(radius) {}

    Eigen::Vector2d pos;
    double radius;

    uint32_t uid;
};

struct BendingStencil
{
public:
    BendingStencil(int p1, int p2, int p3, double kb) : p1(p1), p2(p2), p3(p3), kb(kb) {}

    int p1, p2, p3;
    double kb;
};

}

#endif
