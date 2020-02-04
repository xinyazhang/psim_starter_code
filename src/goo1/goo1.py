#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.getcwd())

from pypsim.goo1 import *

def main():
    vis = GooVisualizer()
    vis.run()

if __name__ == '__main__':
    main()
