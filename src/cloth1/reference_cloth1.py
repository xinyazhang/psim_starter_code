#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.getcwd())
import numpy as np
import pathlib
import argparse

from reference.pypsim.cloth1 import *

def main():
    if 'LD_PRELOAD' in os.environ:
        del os.environ['LD_PRELOAD']
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--obj', help='Load obj file as cloth', default='../assets/cloth/rect-coarse.obj')
    args = p.parse_args()

    vis = ClothVisualizer()
    V, F = loadOBJ(args.obj)
    vis._core.attach_mesh(V, F)
    vis.run()

if __name__ == '__main__':
    main()
