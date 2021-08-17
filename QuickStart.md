# Simple Run

To run **SBMPO** with the default example (1D kinematic object) 

```
$ make run
```

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

There are three different heuristic functions preprogrammed for this example

**distance optimal** will optimize to minimize total distance travelled by looking at the simple difference in desired position from current position.

**velocity aware** will optimize to minimize total distance while being aware of acceleration constraints.

**time optimal** will optimize to minimize time used to get to the goal location.

## Kinematic Example

A configuration file is provided for this example in 
```
/resources/kinematic/config.json
```

**Start** and **Goal** vectors contain (in order): x coordinate, y coordinate, heading angle, angular velocity x, angular velocity y, turn radius 

While this example contains many changeable parameters, we will focus on 2: model -> **energy planning** and model -> **obstacle file** (please see the README for more details on other paramters). Setting **energy planning** to true will change the optimization to energy minimizing, false will keep the default distance optimization. 

**obstacle file** points to the location of the obstacle file, this is a .txt file that contains three columns corresponding to location and size of obstacle. The first column denotes the x coordinate (in meters), the second column denotes the y coordinate Iin meters). The third column denotes the radius of the obstacle.

# Interpretting Results

Resulting trajectories are generated in 

```
/results/results.json
```

This json file is organized with a control and state vector at each time interval. 

## Results of 1D Example

The results.json file can be parsed manually by control or state. The dictionary is provided below:

control[0] - force
state[0] - position
state[1] - velocity
state[2] - force (same as control)

## Results of Kinematic Example

A helpful trajectory reading matlab script has been created and can be found in  
```
/resources/scripts/trajread.m
```

The results.json file can be parsed manually by control or state. The dictionary is provided below:

control[0] - left wheel velocity
control[0] - right wheel velocity
state[0] - x coordinate
state[1] - y coordinate
state[2] - heading angle
state[3] - angular velocity x
state[4] angular velocity y
state[5] - turn radius (1000 is a straight line)