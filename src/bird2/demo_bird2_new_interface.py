#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.getcwd())
import numpy as np
import pathlib
import argparse

from pypsim.bird2 import *

def main():
    core = BirdsCore()
    V, F = loadOBJ('../assets/birds/bird2.obj')
    birdT = RigidBodyTemplate(V, F , 1.0)
    bid1 = core.add_single_instance(birdT, 1.0,
            np.array([5.0, 0.5, 5.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0]))
    bid2 = core.add_single_instance(birdT, 1.0,
            np.array([9.0, 0.5, 5.0]),
            np.array([0.0, 0.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0]))
    s = core.list_rigid_body_instances()
    print(s)
    col = detect_collision(s)
    for c in col:
        print(f'{c.body1} {c.body2} {c.collidingVertex} {c.collidingTet}')
    F = np.zeros((6))
    Fc = np.zeros((6))
    core.compute_penalty_collision_forces(col, F, Fc)
    print(F)
    print(Fc)

if __name__ == '__main__':
    main()
