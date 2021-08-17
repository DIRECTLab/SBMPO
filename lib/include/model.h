#pragma once

#include <iostream>
#include <fstream>

#include "json.h"
#include "debug.h"
#include "definitions.h"

using nlohmann::json;

namespace model {

/// @brief Interface between SBMPO and the model to optimize
///
/// The model represents some complex system which we hope to optimize in terms
/// of one of its state parameters.  In other words, if we have some complex
/// object like a robot, how can we optimize some change in state in terms of
/// _any_ of its state parameters, like energy consumption, or traversal time.
///
/// The model has some set of controls which we can use to influence states; the
/// model class ultimately translates changes in controls to changes in state,
/// so we can use the controls available to us to determine optimal solutions to
/// complex problems.
///
/// This allows us to, rather than say determining the shortest path between
/// two nodes, find the shortest path that the robot can actually take.
class Model {
public:
	/// @brief return the degrees of freedom for the controls
	virtual int control_dof() = 0;

	/// @brief Perform any initial calculation or initialization of model
	///
	/// Called once planning starts, giving the model access to the first state
	/// and control to perform a sometimes necessary initialization step, such
	/// as:
	///
	/// - removing close obstacles of the model's internal obstacle list
	/// - calculating extra initial values for the model that depend on the
	///   values of the first state
	/// - validating the initial state and controls for more robust error
	/// - determination
	///
	/// This method is essentially used in place of a constructor, ensuring that
	/// the model has an opportunity to do an initialization step that json
	/// serialization does not allow, as it requires default construction.
	virtual void init(const State& state, const Control& control) = 0;

	/// @brief Generate a state from a control to apply to a previous state
	///
	/// Because we want to optimize states in terms of the factors we can
	/// actually control, we must have a mapping from states to controls.
	/// However, since previous states influence later states, it's important to
	/// have some sort of method to propogate the effects of past controls from
	/// earlier in the graph search, so we also take the current state.
	///
	/// \param integ_step_size - Is the sampling time
	/// \param state - current state
	/// \param control - Input appplied to determine the next state
	/// \return New generated state
	virtual State next_state(const State& state,
		const Control& control,
		float integ_step_size) = 0;

	/// @brief Determine the cost between two states
	///
	/// Given a current state and future state, to find the optimal path in
	/// terms of a particular state parameter or state parameters, the cost
	/// function provides a method to quantify and _compare_ different paths,
	/// chosing the path that results in the lowest overall cost in terms of
	/// this cost function.
	///
	/// The canonical cost function for many path-finding applications is simply
	/// the euclidian distance (or some other type of distance calculation),
	/// which results in a distance optimizing path.  However, SBMPO exposes
	/// methods to do optimization over arbitrary state parameters, such as:
	///
	/// * energy consumption
	/// * traversal time
	/// * elevation change
	/// * dollars spent
	///
	/// It's important to note that while the cost value is a single value, the
	/// cost function can generate this value after taking into account
	/// _multiple_ state parameters.  For example, if we want to minimize the
	/// cost in dollars when operating some machinery, the fuel consumptoin can
	/// result from _many_ different state parametrs, allowing SBMPO to perform
	/// much more complicated optimizations than normal graph search algorithms.
	///
	/// \param current - the state to traverse from
	/// \param next - the state to traverse to
	/// \param control -
	virtual float cost(const State& current,
		const State& next,
		const Control& control) = 0;

	/// @brief Estimate of future costs from the current state
	///
	/// Given the current state and the goal state, what can we estimate the
	/// future costs will be?  The heuristic determines where the most fertile
	/// paths to search exist, assuming that continuing along a direct path to
	/// the goal will result in the most efficient overall solution.  This
	/// ensures that paths which take a less direct route are explored last.
	///
	/// The canonical heuristic function is often also the euclidian distance
	/// from the current state to the goal state.
	///
	/// The heuristic works best when its units are the same--or at least in the
	/// same order of magnitude--as the cost.
	///
	/// \param current - the state to traverse from
	/// \param goal - the overall goal node to estimate the future costs from
	/// \param control -
	virtual float heuristic(const State& current,
		const State& goal,
		const Control& control) = 0;

	/// @brief Determine when the algorithm has reached the goal
	///
	/// Compare the current state with the goal state to determine when to exit
	/// the algorithm, using the threshold set in the SBMPO config.
	virtual bool converge(const State& current,
		const State& goal,
		float threshold) = 0;

	/// @brief Check to ensure state is valid for the model
	///
	/// Used to ensure that the current state and controls do not meet some
	/// breaking conditions, allowing for collision detection or constraint
	///  enforcement, or other, more complicated, sanity and validity checks.
	///
	/// The canonical example would be colision detection: given some oriented
	/// bounding box and a map of obstacles in the world, can we actually
	/// occupy a position without colliding?  `is_valid` could do the collision
	/// check to tell SBMPO not to expand that particular node.
	///
	/// Another example could be constraining controls: given some maximum or
	/// minimum value for our controls, we can stop a node from expanding if
	/// a control parameter falls outside of the acceptable range.
	///
	/// \param current - the state to check for validity
	virtual bool is_valid(const State& current, const Control& control) = 0;

	/// @brief conversion to JSON representation
	///
	/// Provides an interface to support _serialization_ and deserialization to
	/// and from JSON format, and defines the JSON config file format.  Treat
	/// this as the model constructor, using `Model::init(...)` to do _only_ the
	/// initialization step that requires access to the initial `State` and
	/// `Control`.
	///
	/// Defines structure of JSON config, allowing for full flexibility and
	/// arbitrary config formats.
	///
	/// Allows for implicit conversions like:
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
	/// Model m;
	/// json j = m;
	/// cout << j;
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/// which will result in json output something like:
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{json}
	/// {"dof": 3}
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	///
	/// # Example Implementation
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
	/// to_json(json& j, const Model& model) {
	///     j = json{"dof", model.dof};
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	///
	/// # References
	/// Uses nlohmann's [json library](https://github.com/nlohmann/json).
	friend void to_json(json& j, const Model& model);

	/// @brief conversion from JSON representation
	///
	/// Provides an interface to support serialization and _deserialization_ to
	/// and from JSON format.
	///
	/// Defines structure of JSON config, allowing for full flexibility and
	/// arbitrary config formats.
	///
	/// Allows for implicit conversions from a json file like
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{json}
	/// {"dof": 3}
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/// to a `Model` object through simple assignment like
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
	/// json j;
	/// std::cin >> j; // read in example json from stdin
	/// Model model = j;
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	///
	/// # Example Implementation
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
	/// from_json(const json& j, Model& model) {
	///     model.dof = j.at("dof").get<int>();
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	///
	/// # References
	/// Uses nlohmann's [json library](https://github.com/nlohmann/json).
	friend void from_json(const json& j, Model& model);
};

} // namespace model
