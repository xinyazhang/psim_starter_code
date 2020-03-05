#!/usr/bin/env python3

import os
import sys
sys.path.append(os.getcwd())
import random
import math
import numpy as np

import pytest
from pypsim.bird1 import BirdsCore, RigidBodyTemplate, RigidBodyInstance, SimParameters, loadOBJ

MONKEY_FN = '../assets/birds/monkey.obj'

def _particle_energy(p, g):
    return 0.5 * np.inner(p.vel, p.vel) - g * p.pos[1]

def _create_core():
    c = BirdsCore()
    c.init_simulation()
    return c

@pytest.fixture
def core():
    return _create_core()

@pytest.fixture
def monkey():
    return loadOBJ(MONKEY_FN)

"""
Section 4.1.1
"""
def test_template(monkey):
    t = RigidBodyTemplate(monkey[0], monkey[1] , 1.0)
    assert np.allclose(t.com, [-1.2691001442924821e-17, 0.22542446101571903, 0.047411434236999776])
    assert np.allclose([t.volume], [2.2553726340937801])
    assert np.allclose(t.MI, [[0.64630015126328,-7.1557343384043293e-18,3.0357660829594124e-18],
                              [-7.1557343384043293e-18,0.72859571129126921,0.081725183328729745],
                              [3.0357660829594124e-18,0.081725183328729745,0.65482533269102317]])

def test_body(core, monkey):
    t_monkey = RigidBodyTemplate(monkey[0], monkey[1] , 1.0)
    density = 3.0
    c = np.array([1.0, 2.0, 3.0])
    theta = np.array([0.0, 1.0, 0.0])
    cvel = np.array([0.0, 5.0, 0.0])
    w = np.array([0.0, 0.0, 2.0])
    upd_c = np.array([4.0, 5.0, 6.0])
    bid = core.add_single_instance(t_monkey, density, c, theta, cvel, w)
    assert bid >= 0
    body = core.query_rigid_body_instance(bid)
    assert body.density == density
    assert np.allclose(body.c, c)
    assert np.allclose(body.theta, theta)
    assert np.allclose(body.cvel, cvel)
    assert np.allclose(body.w, w)
    body.c = upd_c
    core.simulate_one_step()
    body2 = core.query_rigid_body_instance(bid)
    assert np.allclose(body2.c, upd_c + core.reference_sim_parameters().timeStep * cvel)

def test_add_meshes(core):
    Qs = np.random.uniform(low = -10.0, high = 10.0, size=(3,13))
    Qs[:,0] = np.random.uniform(low = 1.0, high = 100.0, size=(3)) # No negative density
    Qs[:,0] = np.random.uniform(low = 1.0, high = 100.0, size=(3)) # No negative density
    bids = core.add_mesh(MONKEY_FN, 3.0, Qs)
    def compare_body_with_conf(body, conf):
        density = conf[0]
        c, cvel, theta, w = conf[1:].reshape((-1, 3), order='C')
        assert body.density == density
        assert np.allclose(body.c, c)
        assert np.allclose(body.theta, theta)
        assert np.allclose(body.cvel, cvel)
        assert np.allclose(body.w, w)
    for i in range(Qs.shape[0]):
        compare_body_with_conf(core.query_rigid_body_instance(bids[i]), Qs[i])
