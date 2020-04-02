#!/usr/bin/env python3

import sys
import os
import argparse
import importlib
import numpy as np
import json

def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('paths', help='Directories of pysim modules', nargs='+')
    p.add_argument('--case', help='Grading cases', choices=['goo1', 'goo2'], default='goo1')
    p.add_argument('--store', help='Store the grading results to the same directory of the pysim module', action='store_true')
    p.add_argument('--target_module_name', default='pypsim')
    p.add_argument('-k', help='Select specific test', default='')
    p.add_argument('--verbose', help='Show the details of partial credited items', action='store_true')

    return p.parse_args()

def main():
    args = parse_args()
    init_syspath = list(sys.path)
    grading_module = importlib.import_module(args.case)

    for p in args.paths:
        sys.path = [p] + init_syspath
        try:
            submission_module = importlib.import_module(args.target_module_name)
        except ModuleNotFoundError as e:
            print(e)
            print(f"Cannot find module {args.target_module_name} from {p}. The path should point to bin/ directory")
            continue

        sys.path = init_syspath + [os.getcwd()]
        sub_module = getattr(submission_module, args.case)
        dic, ttl_dic = grading_module.grade(args, sub_module)
        print(dic)
        total_pts = np.sum([v for k,v in dic.items()])
        print(f"Overall {total_pts}/100")
        if args.store:
            sav = { 'Points': dic, 'TotalPoints': ttl_dic, 'OVERALL': total_pts }
            with open(f'{p}/grading.json', 'w') as f:
                json.dump(sav, fp=f)
        del(submission_module)

if __name__ == '__main__':
    main()
