# SBMPO

Sampling Based Model Predictive Control (SBMPO), a novel nonlinear MPC (NMPC) approach that enables motion planning with dynamic models as well as the solution of more traditional MPC problems.

The focus of our motion planning research is to develop algorithms that will enable both mobile robots and manipulators to operate intelligently in extreme environments or for extreme tasks. For mobile robots such as autonomous ground vehicles (AGVs), autonomous air vehicles (AAVs), or autonomous underwater vehicles (AUVs) an example of an extreme environment is a cluttered environment. This problem has been addressed using both reactive and deliberative planning algorithms. For AGVs extreme environments also include difficult terrains such as sand, ice and mud, and highly undulating terrains. For manipulators an extreme task is lifting objects that are so heavy that they cannot be lifted quasi-statically. A unifying feature of each of these latter problems is that they benefit from using a dynamic model in the planning process. Hence, a major focus of this research is the development and refinement of SBMPO.
<!-- 
For any more details on the project, please visit our [website](http://www.ciscor.org/motion-planning/). -->


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

To specify a target, pass a command-line argument `target=[dev, debug, release]`.  For example, to compile and run in the `release` target, invoke make with

```
$ make run target=release
```

To see list of targets, invoke:

```
$ make targets
```

Specific example cases can be seen in the Quickstart.md file located in this same directory.

# Demo Implementations

When running **SBMPO**, we support several different example implementations for testing and development.  To select an example, pass a command-line argument to make `example=[1D, kinematic]` which specifies which example to run.  

Example 1D: 
``` 
make run example=1D
```

1D is the default example where a moving ball is controlled by forces exerted in a 1D world. There are options for moving in a time optimal manner.

Example kinematic:
```
$ make run example=kinematic
```

The kinematic example will run **SBMPO** with the kinematic model of a skid-steered robot. Optimization for this robot can use a distance minimizing or energy minimizing function. (Energy models for the skid-steered robot is provided.)

To see the list of targets available, invoke:

```
$ make examples
```

## 1D Example

A configuration file is provided for this example in 
```
/resources/1D/config.json
```

**Start** and **Goal** vectors contain (in order): position, velocity, and force.

There are three different heuristic functions preprogrammed for this example and can be used by changing the associated config.json file's heuristic argument (line 17)

**distance optimal** will optimize to minimize total distance travelled by looking at the simple difference in desired position from current position.

**velocity aware** will optimize to minimize total distance while being aware of acceleration constraints.

**time optimal** will optimize to minimize time used to get to the goal location.

## Kinematic Example

A configuration file is provided for this example in 
```
/resources/kinematic/config.json
```

**Start** and **Goal** vectors contain (in order): x coordinate, y coordinate, heading angle, angular velocity x, angular velocity y, turn radius 

While this example contains many changeable parameters, two parameters impact the foundational problem solved by the model -> **energy planning** and model -> **obstacle file** . Setting **energy planning** to true will change the optimization to energy minimizing (rather than minimizing distance).

**obstacle file** points to the location of the obstacle file, this is a .txt file that contains three columns corresponding to location and size of obstacle. The first column denotes the x coordinate (in meters), the second column denotes the y coordinate (in meters). The third column denotes the radius of the obstacle (in meters).

# Interpretting Results

Resulting trajectories are generated in 

```
/results/results.json
```

This json file is organized with a control and state vector at each time interval. The control elements correspond to what the system is directly able to control, and the state elements denote what elements correspond to measured states of the system at that time. This state and control is in a vector form and initialized in by the user in a config file (details below).

## Results of 1D Example

The results.json file can be parsed manually by control or state. The dictionary of resulting information is provided below:

* control[0] - force
* state[0] - position
* state[1] - velocity
* state[2] - force (same as control)

This resulting information then informs the user the time-series of commanded forces which resulted in the completion of the objective (get to a specific position).

## Results of Kinematic Example

The results.json file can be similarly parsed by control or state. The dictionary of resulting information is provided below:

* control[0] - left wheel velocity
* control[0] - right wheel velocity
* state[0] - x coordinate
* state[1] - y coordinate
* state[2] - heading angle
* state[3] - angular velocity x
* state[4] - angular velocity y
* state[5] - turn radius (1000 is a straight line)

To better visualize the output of a skid-steered robot, a helpful trajectory reading matlab script has been created for this example and can be found in  
```
/resources/scripts/trajread.m
```
In this same folder, PlotFigs.m will return the trajectory and motion path plots in a typical robot 2D map format. 


# Additional Configurations

This software requires certain configuration parameters to be set in the config.json . Some of these features includes:

* *Start* - Define the start state of the system
* *Goal* - Define the goal state of the system
* *Max Iteration* - Allowed limits to how long the software is allowed to iterate and explore the search space
* *Samples* - The type and size of sampling used, generally defined by the number of samples per expansion and if there are preset samples or randomly drawn samples
* *Goal Threshold* - How close the end result and goal needs to be in order to constitute a successful completion
* *Sampling Time* - Integration time or time between samples 
* *Grid Active* - Designates which (if any) of the state features should be gridded when checking for uniqueness in nodes. This refers to the "implicit" grid that is part of the SBMPO framework. The implicit grid finds and prunes nodes that have higher costs within the same grid cell. Example: two nodes occupy the same grid cell defined by (x_position, y_position, heading_angle), the node with higher cost amongst these two similar nodes will be pruned. If heading angle was not a concern, this example would only choose to grid two parameters (x_position, y_position) resulting in more pruning
* *Grid Resolution* - The resolution that the implicit grid (above) is set to, larger grid means more pruning
* *Model* - used to define any model-specific components. This can include things such as vehicle geometry, obstacle locations, sensor data for the kinematic example
* *Other Config Params* - can be defined per specific model but are not required

### Configuration Example: Kinematic

* "start": [0, 0, 0, 0, 0, 1000], starting configuration is at x=0, y=0, heading angle = 0, angular velocity x = 0, angular velocity y = 0, turn radius = 0
* "goal": [5, 5, 0, 0, 0, 0], go to point (5,5)
* "max iterations": 50000, allowed iterations are relatively low but the default example only covers a short-distance of travel
* "branchout": 13, allowed number of samples per exploration
* "samples": the number of samples (determined by the branchout) are hard coded to reflect vehicle realities. The first sample refers to the turn radius (smaller is a tighter turn) and the second corresponds to the forward velocity (this example assumes that we maintain constant velocity of 0.6 m/s).
    * [0.6415, 0.6],
    * [0.9812, 0.6],
    * [2.7800, 0.6],
    * [4.7260, 0.6],
    * [8.3400, 0.6],
    * [33.0800, 0.6],
    * [1000.0000, 0.6],
    * [-0.6415, 0.6],
    * [-0.9812, 0.6],
    * [-2.7800, 0.6],
    * [-4.7260, 0.6],
    * [-8.3400, 0.6],
    * [-33.0800, 0.6]
* "goal threshold": 1, the algorithm completes successfully if a trajectory is found within 1 m radius around the goal 
* "sampling time": 0.5, expansions happen every 0.5 seconds of the robot's motion
* "constraints": [[0.5, 1000.0], [-1, 1]], the robot is limited in this case to only being able to sample between turn radii of 0.5 to 1000.0 m, and max velocity between -1m/s to 1m/s. This parameter is used only for randomly sampling, any hard-coded or preset samples (as in the above) will override the constraints. It is then important that hard-coded samples be within the constraints of the problem that is being solved
* "grid active":[true, true, false, false, false, false], the implicit grid only considers x_position and y_position
* "grid resolution":[0.05, 0.05, 0, 0, 0, 0], pruning of the implicit grid will happen in cells that are sized at 0.05m
* "model": { these parameters are specific for this scenario
    * "type": "energy", sets the problem up as an energy-optimal planner
    * "energy planning": false, toggles the cost/heuristic to consider energy or just distance
    * "control dof": 2, we are allowing 2 control parameters (velocity left wheel, velocity right wheel)
    * "model dof": 2, model considers two parameters as well (turn radius, forward velocity)
    * "model type": 1, a bit superfulous as we have only defined one skid-steer vehicle model. If others were defined, this parameter would change to another model that uses similar inputs (ie: ackerman steer, tank/treaded vehicle, ect.)
    * "expansion factor": 2.0193, this is a terrain penalty introduced to acount for the degradation of turning in real-world environments. This terrain factor corresponds to experimentally derived values on asphalt
    * "surface": "asphalt", helpful for us to remember what the expansion factor above is correlated with. Can be used in the code to label terrain but is not currently utilized
    * "wheel radius": 0.165, physical characteristic, impacts the distance traversed
    * "vehicle width": 0.556, physical characteristic that impacts the effective turn radius and energy consumption
    * "payload": 0, additional weight put on the vehicle
    * "slope": 0, adds a slope to the ground in the forward direction of the robot only
    * "slip x": [ 1, 0, 0 ], adds tuning for any slip in the x direction
    * "slip y": [ 0, 1, 0 ], adds tuning for any slip in the y direction
    * "slip z": [ 0, 0, 1 ], adds tuning for any slip in the z direction
    * "obstacle file": "resources/kinematic/fewobstacles.txt", location of the obstacle file that will be loaded into this scenario. This file is comprised of circular obstacles defined by three parameters (x, y, radius)
    * "obstacle grid resolution": 0.1, how close the vehicle can approach the obstacle before collision is determined.
    * "ignore close obstacles": true, in case a random start location or obstacle field accidently starts the vehicle inside of an obstacle, any obstacle is such cases is ignored
    * "ignore radius": 0.3, the radius of ignoring in the close obstacles case, note it is smaller than the bounding box of the vehicle
    * "bounding box": {"length": 0.5, "width": 0.5}, the effective size of the vehicle

## Expected Outcomes

Generally one of three outcomes occur after excecuting the software. 

* Queue is empty - No solution could be found, this is often due to complex obstacle configurations, model failure, or other situations where solutions are not possible to attain. 
* Iteration limit exceeded - The solution requires more compute iterations, rerun with the config file altered to increase iterations
* Solution found - this will be immediately apparent by the set of control and state information returned to the console 

The output is stored in the results.json file within the results/ directory. The computation time, cost (model specific), control, and state information are parseable through json. 

### Outputs of 1D Example
The default parameters (velocity aware) in the config file for the 1D actuated ball will find a solution that takes 69 time-steps to solve. The final state of the system will be (9.99750232696533,0.0750010162591934,-0.5), corresponding to position, velocity, and force). 

Running the 1D case in a time optimal paradigm resutls in a final state of (10.0050039291382,0.0500010475516319,-1.0) which required only 68 time steps to accomplish. This situation can be run by going into the 1D case config.json file and editing the "heuristic" element (under model) to "time optimal." 

Finally, setting the heuristic to "distance optimal" sets the final state to (9.95001983642578,-0.0499982386827469,0.75) and takes 146 time steps. This is due to the planner exploring far longer to find a trajectory which allows the system to come as close to exactly 9.95m (goal condition is at 10m but there is a 0.05m goal threshold) as possible. This is the minimum distance as can be compared to the other two cases above.


### Outputs of Kinematic Example
When running the software for the kinematic case under the default configurations (distance optimal), the results.json file will contain 22 time-steps of data, with the final position of the vehicle at (4.73392152786255,4.31180143356323). This can be seen from the final state vector's first two elements.

Rerunning this case with the config.json 'energy planning'=true (line 28) changes the planner from a distance optimal to an energy optimal paradigm. This planner now takes 31 time-steps to complete (a roughly 50% increase in distance covered) and ends at position (4.92237424850464,4.16017389297485). 

## Additional Features

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

<!-- # Examples

When running **SBMPO**, we support several different example implementations for testing purposes.  To select an example, pass a command-line argument to make `example=[1D, kinematic, dotcder]` which specifies which example to run.  1D is the default example.

```
$ make run example=kinematic
```

Will run **SBMPO** with the kinematic model of a skid-steered robot.

To see list of targets, invoke:

```
$ make examples
``` -->

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

## Results

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


