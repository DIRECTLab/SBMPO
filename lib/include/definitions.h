#pragma once

#include <vector>
#include <list>
#include <cfloat>
#include <unordered_map>
#include <array>

#include "json.h"

using nlohmann::json;

using namespace std;

#define VICON_PORT 3491
#define CUBEROOT(x) ( (x) < 0 ? -pow(-1*(x),1.0/3) : pow((x),1.0/3) )
#define CUBEOF(x) ( (x) < 0 ? -pow(-1*(x), 3) : pow((x),3) )

// ENERGY REALTED SECTION - TALK TO EITHER NIKHIL OR CAMILO IF YOU WANT TO CHANGE THE VALUES

#define PI 3.14159265
#define Mass 86.35
#define gr 9.8
#define Kt 0.044487
#define gRatio 78.71
#define eta 0.70
#define Re 0.5
#define WHEEL_RADIUS 0.165

// SECTION ENDS

/// defines the state of a robot
typedef vector<float> State;
/// defines the key-value stored in a priority queue
typedef pair<float, float> Key;
/// defines index used in the code
typedef int Index;
/// defines the control value of a robot
typedef vector<float> Control;
/// defines the grid representation of the graph
typedef vector<Index> GridCell;
/// define list of indexes which are not used
typedef list<Index> IndexList;

namespace std {
	template<>
	class hash<GridCell> {
	public:
		std::size_t operator()(GridCell const& gridcell) const {
			std::size_t seed = gridcell.size();
			for (auto& i : gridcell)
				seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);

			return seed;
		}
	};
}

/// hashmap of the graph to quickly query index wrt position of the robot
typedef std::unordered_map<GridCell, Index> Grid;
typedef std::unordered_map<GridCell, IndexList> MultiGrid;

/// Representation of the Obstacle as a vector of floats.
typedef vector<float> Obstacle;

/// Representation of the State as a vector of floats.
typedef vector<float> State;

/// @brief Representation of a neighbor in the graph.
///
/// Representation of the connections between a vertex and a neighbor vertex.
struct Neighbor {
	Index neighbor_id;
	float cost;
	Control control_fwd; // dont throw away the control part
	State state_fwd;
};

/// @brief Representation of a vertex in the graph.
///
/// This is used in a directed graph, so neighbors listed in pred vector
/// represent an edge starting at the neighbor and ending and the current
/// vertex.  Neighbors listed in the succ vector represent an edge starting and
/// the current vertex and ending at the neighbor.
struct Vertex {

	Vertex() {}

	Vertex(Index _id, int _p, float _h, State &_s, Control & _c) :
		id(_id), state(_s), control(_c), parent_id(_p),
		queue_id(-1), rhs(FLT_MAX), g(FLT_MAX), h(_h)
	{}
	/// id of the vertex
	Index id;
	/// parent id of the vertex
	Index parent_id;
	/// its index in the queue
	int queue_id;
	/// its key value stored in the queue
	Key key;
	/// distance from the start
	float g;
	/// heuristic estimate towards the goal
	float h;
	/// better estimate of the distance from the start node
	float rhs;
	/// state of the vertex
	State state;
	/// control required to reach that state from the parent state
	Control control;
	/// List of nodes which merges on the current node
	vector<Neighbor> pred;
	/// List of nodes which branch-out from the current node
	vector<Neighbor> succ;
};

/// @brief final trajectory
struct Trajectory {
	double cost;
	double computation_time;
	vector<Vertex> trajectory;

	Trajectory(): cost(0), computation_time(0) {}
};

/// @brief robot's configuration such as width and length
struct robot_config{
	float width;
	float length;
};

/// @brief Conversion of Trajectory to JSON
///
/// Outputs the cost and a json array of controls and states, used for results
/// output by performing implicit to conversion to json and then outputting the
/// json file.
///
/// ## Example
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// json results = sbmpo_output;
/// fout << results;
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// results in something like:
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.json}
/// { "cost": 0,
///   "computation time": 0,
///   "trajectory": [
///     { "control": [1,1,1,0], "state": [0,0,0,1000] },
///     { "control": [2,-1,1,1], "state": [0,0,0,900] }
///   ]
/// }
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
inline void to_json(json& j, const Trajectory& o) {
	// HACK: if you have defined a `to_json` or `from_json` instance in a header
	// file, it will not compile properly without an inline statement to ensure
	// that only one definition of the function exists, as opposed to multiple.
	// This is basically an arcane "feature" of the C++ compilation chain.

	json trajectory;
	for (auto v : o.trajectory) {
		// Accumulate only the states and controls in a json array from each
		// vertex, as all other information is irrelevant for the output.
		trajectory.push_back(json{{"state", v.state}, {"control", v.control}});
	}
	j = {
		{"cost", o.cost},
		{"trajectory", trajectory},
		{"computation time", o.computation_time}
	};
}
