#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.getcwd())
import numpy as np
import pathlib
import argparse

from pypsim.bird1 import *

class Birds(BirdsVisualizer):
    '''
    load_scene:
    override the implementation of BirdsVisualier.load_scene.

    The logic is implemented in Python instead of C++ to exploit the standard
    python library and numpy.
    '''
    def load_scene(self, fn):
        '''
        BirdsCore
        '''
        core = self._core
        print(core)
        if not fn.endswith('.scn'):
            print(f'{fn} is not a valid scene file')
            return
        print(f"Loading scene {fn}")
        p = pathlib.Path(fn)
        pd = p.parent
        with open(fn, 'r') as f:
            nobj = int(next(f).split()[0])
            for i in range(nobj):
                ln = next(f)
                seq = ln.split()
                print(seq)
                print(len(seq))
                relobj = seq[0]
                scale = float(seq[1])
                conf = np.array([float(e) for e in seq[2:]])
                conf = conf.reshape((1, 13))
                print(conf.shape)
                '''
                We blindly create a new RigidBodyTemplate for each line.

                Extra credit: introduce some mechanism to avoid creating
                              duplicated RigidBodyTemplate

                Hint        : using (file name, scale) as the key of dict as
                              the directory of RigidBodyTemplate objects.

                              (V,F) can be loaded with bird1.loadOBJ(fn)

                              The configuration can be extracted with the following code
                                c, cvel, theta, w = conf[1:].reshape((-1, 3), order='C')
                '''
                ret = core.add_mesh(str(pd.joinpath(relobj)), scale, conf)
                print(f"add_mesh returns {ret}")
                body = core.query_rigid_body_instance(ret[0])
                print(f"{body.c} {body.cvel} {body.theta} {body.w}")

def main():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--scene', help='Load scene file', default='')
    args = p.parse_args()

    vis = Birds()
    if args.scene:
        vis.load_scene(args.scene)
        vis.init_scene()
    vis.run()

if __name__ == '__main__':
    main()
