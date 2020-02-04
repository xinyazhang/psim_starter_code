# Introduction to Physical Simulation Project Starter Code

## Preamble 

We are going to use the same monolithic git repository to development all your
physical simulation code, and the following instructions applies to all your
projects.

## Build Instruction

This is a standard cmake project, with a few bundled git modules.
To build this project, run:
```
git clone --recursive https://github.com/xinyazhang/psim_starter_code psim
cd psim
mkdir build
cd build
cmake ..
make -j `nproc`
```
These commands will clone the git repo into psim/ directory, and build the
code inside `psim/build/` directory. The compiled the code can be found at
`psim/bin`.

Afterwards, you can run `./goo1.py` under the `psim/bin` directory.
Note: this script must be run at `psim/bin`, otherwise it cannot find the
compiled module.

## Test your build

change directory to `psim/bin/`, and run `./goo1.py` for your implementation.
and `./reference_goo1.py` for the reference implementation.

`bin/reference` contains the reference solution.

## Quick start to write your simulation code

The starter code contains all essential code for visualization. All you need
to do is to fill up the `// TODO ` sections inside `lib/core/goo1/GooCore.cpp`
and its header file `lib/core/goo1/GooCore.cpp`.

## Brief introduction to the code structure

This repository organizes the source code in the following manner:
* `lib/core/<project name>`: core physical simulation code
* `lib/vis/<project name>`: visualization of the current physical system, with
    [libigl](https://libigl.github.io/)
* `lib/python/<project name>`: python binding code with pybind11
* `src/<project name>`: Python code to invoke the built module
* `third-party/`: git submodules
* `bundle/`: bundled projects with does not have a git repo.

## A Few Grading Criteria

* Unless you are claiming extra credits for improvements of your GUI,
  we are NOT going to evaluate your GUI.
* During the evaluation, we are not going to build the GUI code inside the python module.
  In other words, your simulation code should build and run well with `cmake -DPSIM_ENABLE_VISUALIZER=OFF`.
  + This option can be toggled with `(cd build; ccmake .)`
