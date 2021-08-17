/// @mainpage Sampling Based Model Predictive Optimization
///
/// Sampling Based Model Predictive Control (SBMPO), a novel nonlinear MPC
/// (NMPC) approach that enables motion planning with dynamic models as well as
/// the solution of more traditional MPC problems.
///
/// The focus of our motion planning research is to develop algorithms that will
/// enable both mobile robots and manipulators to operate intelligently in
/// extreme environments or for extreme tasks. For mobile robots such as
/// autonomous ground vehicles (AGVs), autonomous air vehicles (AAVs), or
/// autonomous underwater vehicles (AUVs) an example of an extreme environment
/// is a cluttered environment. This problem has been addressed using both
/// reactive and deliberative planning algorithms. For AGVs extreme environments
/// also include difficult terrains such as sand, ice and mud, and highly
/// undulating terrains. For manipulators an extreme task is lifting objects
/// that are so heavy that they cannot be lifted quasi-statically. A unifying
/// feature of each of these latter problems is that they benefit from using a
/// dynamic model in the planning process. Hence, a major focus of this research
/// is the development and refinement of SBMPO.
///
/// For any more details on the project, please visit our
/// [website](http://www.ciscor.org/motion-planning/)

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <iomanip>
#include <cfloat>
#include <cmath>
#include <sys/time.h>
#include <unistd.h>
#include <unordered_map>

#include "debug.h"
#include "definitions.h"
#include "controlConstraints.h"
#include "model.h"
#include "collision.h"
#include "config.h"

using namespace std;

// Forward declare Tuner so Planner can declare it as friend to grant Tuner
// access to it's own constructor.
namespace config { class Tuner; }

/// @brief implementation details of sbmpo
namespace sbmpo {

/// @brief Implementation of SBMPO algorithm.
///
/// The search algorithm actually uses LPA*.
///
/// Initialize `sbmpo` using the builder pattern:
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// Planner planner = Tuner::from_config(config_json).init(&model);
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// `Tuner::init` will create a Planner object after the Tuner has performed
/// up-front tuning and initialization for sbmpo.
class Planner {
	friend class config::Tuner;
public:
	~Planner();

	/// @brief Compute trajectory from configured start state to goal state
	Trajectory compute_trajectory();

	/// @breif Compute Trajectory from given start state to given goal state
	Trajectory compute_trajectory(const State& start, const State& end);

	/// @brief Clear all internal data
	void clear();

	/// Sampling Time, aka Step size
	float T;
	// made it public to use it to look ahead from the behaviour class of RFrame
	// framework

private:
	/// @brief private constructor called inside Tuner
	Planner(
		model::Model* model,
		State start,
		State goal,
		int branchout,
		int max_iteration_limit,
		float T,
		float goal_threshold,
		vector<Control> samples,
		ConstraintList control_constraints,
		vector<float> grid_resolution,
		vector<bool> grid_active
	);
	// -------------------- Variables section -----------------------

	model::Model* model;
	State start_global;
	Index start_id;
	Index goal_id;
	struct timeval tod1, tod2;
	State goal_global;
	vector<Control> samples;
	int max_iteration_limit;
	ConstraintList control_constraints;

	//GRID
	vector<Vertex> graph;

	Grid grid;
	int branchout;
	vector<bool> grid_active;
	vector<float> grid_resolution;
	float goal_threshold;

	//PRIORITY QUEUE
	vector<int> Q;
	vector<Vertex> rejected_list;

	//SPE section
	vector<Control> _sampled_controls_spe;

	// -------------------- Variables section ends -----------------

	/// @name Gridding
	///
	/// Used to implicitly grid controls based on their parmeters, using user
	/// specified grid resolution.
	///
	/// @{

	/// Checks whether a cell corresponding to a state already contains a state; if
	/// the corresponding cell already contains a state, the pointer to the cell is
	/// copied in the cell parameter.
	///
	/// \param state - State to be checked.
	/// \param cell - Location where the cell corresponding to the State provided
	/// will be stored.
	/// \return Returns true if the cell doesn't contain any states; false,
	/// otherwise.
	bool cell_empty(const State &state, GridCell* cell);

	/// Computes the cell that corresponds to a given state, based on the resolution
	/// set for the grid.
	///
	/// \param state - State to compute the corresponding cell.
	/// \param cell - Location where the corresponding cell will be stored.
	void calc_grid_cell(const State &state, GridCell* cell);

	/// @}

	/// @name Queue Operation
	///
	/// Implementation of SBMPO's priority queue.
	///
	/// @{

	/// Compares two keys.
	///
	/// \return Returns true if k1 is smaller than k2, false otherwise.
	bool compare_keys(Key k1, Key k2);

	void percolate_down(int hole, int vid);
	void percolate_up(int hole, int vid);
	void percolate_up_or_down(int hole, int vid);

	/// Returns the Key of the first item in the queue.  If the queue is empty, the
	/// value returned is a pair of FLT_MAX.
	Key q_top_key();

	/// Returns the first item of the queue.
	/// \return QueueItem, which consists of the Vertex id and the Key that
	/// determines the priority of the current vertex.
	int q_top();

	/// @brief Removes the first item from the priority queue.
	void q_pop();

	/// Adds a new item to the priority queue in the correct location according to
	/// the Key given to QueueItem qi.
	/// \param qi - QueueItem to be added to the priority queue.
	void q_insert(int vid);

	/// Removes all QueueItems corresponding to the vertex whose id is vid.
	/// \param vid - Id of the vertex that needs to be removed from the queue.
	void q_remove(int vid);

	/// @}

	/// @name Logging
	///
	/// @{

	/// @brief calculating time difference
	long int tod_diff(const timeval& tod1, const timeval& tod2);
	/// @brief stores trajectory
	void store_trajectory(char* filename, const Trajectory& path);
	/// @brief stores graph
	void store_graph(char* filename);
	/// @brief stores controls given in the list
	void store_controls(char* filename, vector<Control> control_list);

	/// @}

	/// @name LPA*
	///
	/// Helper methods for LPA* specific tasks
	///
	/// @{

	Key calculate_key(int u);
	void update_vertex(int u);
	void update_rhs(int u);

	//Generates children for each node
	void generate_children(Index u_id,
		int iteration,
		ConstraintList control_constraints);

	/// @}

};

/// Thrown by sbmpo::Planner in case of error
class PlanningError : public std::runtime_error {
public:
	explicit PlanningError(std::string const& error_message) :
		std::runtime_error("Planning error: " + error_message)
	{}
};

} // namespace sbmpo
