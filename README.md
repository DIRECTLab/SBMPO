# SBMPO

Sampling Based Model Predictive Control (SBMPO), a novel nonlinear MPC (NMPC) approach that enables motion planning with dynamic models as well as the solution of more traditional MPC problems.

The focus of our motion planning research is to develop algorithms that will enable both mobile robots and manipulators to operate intelligently in extreme environments or for extreme tasks. For mobile robots such as autonomous ground vehicles (AGVs), autonomous air vehicles (AAVs), or autonomous underwater vehicles (AUVs) an example of an extreme environment is a cluttered environment. This problem has been addressed using both reactive and deliberative planning algorithms. For AGVs extreme environments also include difficult terrains such as sand, ice and mud, and highly undulating terrains. For manipulators an extreme task is lifting objects that are so heavy that they cannot be lifted quasi-statically. A unifying feature of each of these latter problems is that they benefit from using a dynamic model in the planning process. Hence, a major focus of this research is the development and refinement of SBMPO.

For any more details on the project, please visit our [website](http://www.ciscor.org/motion-planning/).

# Graph Search Vizualization
A webpage that Gwen has provided that has very helpful vizualizations for different graph search methods.
Includes A* method used in our SBMPO implementation:

[Visualizing Graph Search Methods](https://glfmn.github.io/ISC4221/)

# Quick start

To run SBMPO with the default example and profile, simply invoke

```
$ make run
```

# Dependencies

## Ubuntu 14.04 and lower

SBMPO needs `g++` version 4.9, which you can get by running

```
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
$ sudo apt-get update && sudo apt-get install g++-4.9
$ sudo update-alternatives --install /bin/usr/g++ g++ /bin/usr/g++-4.9 50
```

This adds the ubuntu toolchain PPA which provides access to newer versions of `g++`, and updates the `g++` command to use `g++-4.9` as desired.

# Make rules

## all

Compile all object files, static library, and **SBMPO** binary, but will not run or perform any action.

## run

Run the current target with the default example.

## lib

Compile the shared library (.so) and static library (.a) variants of SBMPO fit for inclusion in other code.

## target

Makefile supports three build targets:

- `dev`
- `debug`
- `release`

The `dev` target compiles in optimized mode with debug symbols for `gdb` enabled.  The optimized compilation ensures that we catch bugs that will appear in the final release mode, and enabling debug symbols for GDB ensures we can find simple bugs in SBMPO.

The `debug` target is much the same as `dev` except that it compiles with the `-DDEBUG_BUILD` flag passed to the compiler which enables debug logging in SBMPO; this allows deeper inspection of what SBMPO is doing at any given time by searching through the debug log.

The `release` target compiles the fastest possible binary with no extra symbols or features to enable debugging.

To specify a target, pass a command-line argument `target=[deg, debug, release]`.  For example, to compile and run in the `release` target, invoke make with

```
$ make run target=release
```

To see list of targets, invoke:

```
$ make targets
```

## feature

We support some features to be conditionally complied, usually only used for special debugging purposes.  Features are complied using:

```
$ make clean
$ make run feature=-D<feature_flag>
$ make clean
```

> Make clean is **absolutely** necessary, as it is the **only** way to ensure that every object file in the desired target is compiled with the desired feature-flag enabled.  Furthermore, it is **crucially** important to run make clean once you no longer want to compile with the feature, otherwise we run the risk of mixing object files where some have a feature enabled and some don't, which can cause nasty bugs and linker errors.

### visualizer

The visualizer will output the state of the priority queue, and the optimal trajectory from the start to the current node at _every_ iteration of **SBMPO**.  Because this incurs some serious runtime costs (where the visualizer has `n^2` space complexity storing the preivous states in memory), we do not compile this except in special cases where trajectories are relatively short.

```
$ make clean target=<target> && make run target=<target> feature=-DVISUALIZER
```

## example

When running **SBMPO**, we support several different example implementations for testing purposes.  To select an example, pass a command-line argument to make `example=[1D, kinematic, dotcder]` which specifies which example to run.  1D is the default example.

```
$ make run example=kinematic
```

Will run **SBMPO** with the kinematic model of a skid-steered robot.

To see list of targets, invoke:

```
$ make examples
```

## make log

If **SBMPO** was run as debug target, then `make log` will draw the log onto the screen.  For most effective use, combine with `grep` and `less` or some other searching and paging utilities.

To show all nodes expanded from the priority queue, invoke:

```
$ make log | grep Q | less
```

To see the calculated heuristic values, try:

```
$ make log | grep heuristic | less
```

To see every time a model's methods are called, try:

```
$ make log | grep MODEL | less
```

## make clean

Will clean binaries and result .json and .txt files.  The clean rule is sensitive to target, so make sure to specifically clean the files for the desired target.

```
$ make clean target=release # clean ONLY release objects
$ make clean target=dev # clean ONLY dev objects
$ make clean target=debug # clean ONLY debug objects
```

# Directory Structure

## Docs

Documentations are located in ./docs folder. For html, please open index.html file located in ./docs/html folder.

## Resuts

Location of files generated by running **SBMPO** during testing.

## Lib

The SBMPO itself resides in lib, where it is a library meant for consumption by
other libraries or applications.

- src:
    - `.cpp` source files
- include:
    - `.h` header files

## App

An example main file and binary implementation that consumes SBMPO as a library,
used for day-to-day testing of **SBMPO** to ensure things are working.

## Resources:

Supporting files such as samples and torque values on asphalt surface etc.

## bin:

Contains the object and executables generated after compiling the code
