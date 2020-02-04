#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.getcwd())

try:
    from reference.pypsim.goo1 import *
except:
    from pypsim.goo1 import *

INTEGRATORS = [SimParameters.TI_EXPLICIT_EULER, SimParameters.TI_IMPLICIT_EULER, SimParameters.TI_IMPLICIT_MIDPOINT, SimParameters.TI_VELOCITY_VERLET]

def main():
    core = GooCore()
    for it in INTEGRATORS:
        core.init_simulation()
        simp = core.reference_sim_parameters()
        '''
        Moving particle
        '''
        simp.particleFixed = False
        simp.gravityG = 100.0
        simp.timeStep = 0.01
        simp.particleMass = 3.14159
        simp.integrator = it
        pid = core.add_particle(1.0, 1.0)
        '''
        Distant particle
        '''
        pid2 = core.add_particle(10000.0, 10000.0)
        '''
        Fixed particle
        '''
        simp.maxSpringStrain = 100.0
        simp.particleFixed = True
        pid3 = core.add_particle(0.9, 1.0)

        p, valid = core.query_particle(pid)
        print(f"Current Integrator: {it}")
        for i in range(10):
            core.simulate_one_step()
            p, valid = core.query_particle(pid)
            assert valid, f"pid {pid} must be valid"
            print(f"ITER {i:04}: Particle {pid} valid {valid} pos {p.pos} vel {p.vel} mass {p.mass} fixed {p.fixed}")
            p2, valid = core.query_particle(pid2)
            assert not valid, "Distant particles should be removed"

if __name__ == '__main__':
    main()
