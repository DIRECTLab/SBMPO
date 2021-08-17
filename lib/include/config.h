#pragma once

#include <ostream>
#include <string>
#include <stdexcept>
#include <vector>

#include "controlConstraints.h"
#include "definitions.h"
#include "model.h"
#include "sampling.h"
#include "sbmpo.h"

// Forward declare Planner so Tuner can call its constructor.
namespace sbmpo { class Planner; }

/// @brief Configuration types for SBMPO
namespace config {

// Bring json into the config namespace, allowing use of json as "config::json"
// after importing the "config.h" header
using nlohmann::json;

/// @brief Builder type for sbmpo
///
/// Handles complex sanity checking and construction of the planning algorithm,
/// performing simple step-by-step initialization with method chaining.
///
/// Some methods may throw `config::ConfigError` as an exception.
///
/// # Examples
///
/// ## Basic Usage
///
/// We can set multiple parameters using `Tuner::from_config`. In most cases, we
/// can use this static named constructor for simple cases:
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// sbmpo::Planner p = config::Tuner::from_config(config_json).init(&model);
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// More complicated initialization cases become relatively simple as well,
/// using `Tuner::from_config` as a base and performing more complicated
/// conditional logic after the fact.
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// using config::Tuner;
/// using sbmpo::Planner;
///
/// Tuner tuner = Tuner::from_config(config_json);
///
/// // Conditionally override branchout
/// if (model.dof == 3) { tuner.branchout(6); }
///
/// Planner planner = tuner.init(&model);
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///
/// ## Outputting debug information and warnings
///
/// Optionally, the Tuner can inform the user when unsafe values are overriden,
/// or when values are adjusted.
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// sbmpo::Planner planner = config::Tuner()
///     .verbose(true) // flag tuner to output information and warnings
///     .load_config(config)
///     .init(&model);
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// We can also initialize a Tuner with a reference to an ostream other than
/// `std::cerr`, such as an output file.  When creating a tuner with a custom
/// `std::ostream`, the Tuner will output information by default, so there's
/// no need to set verbose to true.
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// ofstream config_log("results/config.log");
///
/// Tuner config = Tuner::with_output(config_log); // verbose by default
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///
/// ## Mandatory Configuration parameters
///
/// The `Tuner` will perform a runtime check to make sure that all mandatory
/// parameters have been set; any call to `Tuner::init` with unset mandatory
/// parameters throws a `ConfigError`.
///
/// Mandatory parameters include:
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// State start;
/// State goal;
/// int branchout;
/// float goal_threshold;
/// float sampling_time;
/// vector<Control> samples
/// controlConstraints control_constraints;
/// vector<blool> grid_active;
/// vector<float> grid_resolution;
/// int max_iteration_limit;
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///
/// \sa start - Starting state of the model to optimize, the initial point of
/// the trajectory to  produce.
///
/// \sa goal - Goal state of the model to optimize, the end point of the
/// trajectory to produce.
///
/// \sa samples - The fixed sample set used to branchout controls and explore
/// the input space.
///
/// \sa branchout - the number of branches to produce, must match sample size
///
/// \sa goal_threshold - convergence criterion for calculating the trajectory
///
/// \sa sampling_time - integration step-size
///
/// \sa control_constraints - constraints on control values
///
/// \sa implicit_grid_params - flags for parameters to grid and respective grid
/// resolutions.
///
/// \sa max_iterations - maximum number of iterations before early-exit
class Tuner {
	// The builder pattern uses one type to handle the complex cases for the
	// initialization of another, by using the builder-type's methods to set
	// the values for the buit type, rather than creating many complicated
	// constructors for every possible case, which can, in extreme cases, result
	// in a combinatorial explosion of constructor definitions.
	//
	// Another advantage is the ability to handle complicated initialization
	// cases where we need to defer setting parameters after checking conditions
	//
	// Furthermore, by hiding the planner's constructor we obfuscate sbmpo
	// and provide a clean external interface, where we can change how sbmpo
	// works internally without breaking old code, so long as our changes do not
	// change the constructor too much.
public:
	Tuner();
	~Tuner();

	/// @brief initialize and return the constructed sbmpo object with the model
	///
	/// Final sanity checking before Planner initialization.
	sbmpo::Planner init(model::Model*);

	/// @name Setting Tuner Values from JSON
	///
	/// @{

	/// @brief Create a tuner from a config, alias of Tuner::load_config
	///
	/// Allows for one-liners like so, where we initialize a planner fully from
	/// a config file and then call init.
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
	/// sbmpo::Planner planner = Tuner::from_config(config).init(&model);
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	///
	/// # Configuration parameters
	///
	/// TODO: Fully document configuration parameters
	///
	static Tuner from_config(const json&);

	/// @brief Set multiple configuration values from a json config
	///
	/// # Sample Config
	///
	/// Using fixed samples:
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.json}
	/// {
	///     "verbose config": true,
	///     "start": [0, 0, 0, 0, 0, 1000],
	///     "goal": [5, 0, 0, 0, 0, 0],
	///     "max iterations": 50000,
	///     "branchout": 13,
	///     "samples": [
	///        [0.6415, 0.6],
	///        [0.9812, 0.6],
	///        [2.7800, 0.6],
	///        [4.7260, 0.6],
	///        [8.3400, 0.6],
	///        [33.0800, 0.6],
	///        [1000.0000, 0.6],
	///        [-0.6415, 0.6],
	///        [-0.9812, 0.6],
	///        [-2.7800, 0.6],
	///        [-4.7260, 0.6],
	///        [-8.3400, 0.6],
	///        [-33.0800, 0.6]
	///    ],
	///     "goal threshold": 0.5,
	///     "replanning frequency": 1,
	///     "look ahead index": 1,
	///     "sampling time": 0.5,
	///     "constraints": [[0.0, 0.2], [-1, 1]],
	///     "grid active":[true, true, false, false, false, false],
	///     "grid resolution":[0.05, 0.05, 0, 0, 0, 0],
	///     "model": { "type": "energy" }
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	Tuner& load_config(const json&);

	/// @}

	/// @name Builder methods
	///
	/// Used to build the Planner, performing sanity checks on individual
	/// parameters when necessary.
	/// @{

	/// @brief Set the start state
	Tuner& start(State);

	/// @brief Set the goal state
	Tuner& goal(State);

	/// @brief load fixed samples directly as a vector of controls
	Tuner& samples(std::vector<Control>);

	/// @brief Set the number of child nodes expanded from each node.
	///
	/// If the branchout argument is 0, method will default to 8.  Must match
	/// sample size.
	Tuner& branchout(unsigned int);

	/// @brief Set the goal threshold used to determine algorithm convergence
	Tuner& goal_threshold(float);

	/// @brief Set the timestep, or sample rate
	Tuner& sampling_time(float);

	/// @brief Set constraints on control values
	Tuner& control_constraints(ConstraintList);

	/// @brief Set the active grid states and provide their resolutions
	///
	/// The implicit grid parameters are probably the **most** important tuning
	/// parameters.  They determine how--and weather--states are implicitly
	/// gridded to save memory, preventing the expansion of similar nodes.
	///
	/// \param active - a vector of flags to determine which states to
	/// implicitly grid by.  Usually, this would be something analagous to the
	/// x and y components of the state.
	/// \param resolution - the non-zero gridding resolution used to determine
	/// how close the gridded parameters must fall to collide.
	Tuner& implicit_grid_params(std::vector<bool>, std::vector<float>);

	/// @brief Set the maximum number of iterations for early exit
	Tuner& max_iterations(int);

	/// @}

	/// @name Tuner configuration methods
	///
	/// Methods specifically configuring Tuner's behaviour, such as setting the
	/// source for debug information.
	/// @{

	/// @brief Set Tuner to output information and warnings
	///
	/// Will output to the specified `std::ostream`, which is `std::cerr` by
	/// defaut.
	Tuner& verbose(bool);

	/// @brief Set Tuner's diagnostic output destination
	///
	/// Reference to any ostream, such as a file or
	static Tuner with_output(std::ostream&);

	/// @}
private:
	/// @brief helper method to support alternate syntax for loading samples
	/// from config file
	///
	/// Rather than passing an array of values to the config, we can specify
	/// a method, branchout, and constraints as part of the sample config key:
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.json}
	/// {
	///     "samples": {
	///         "type": "halton",
	///         "branchout": 13,
	///         "constraints": [[0.0, 0.2], [-1, 1]]
	///     }
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	Tuner& auto_samples(std::string, int, ConstraintList);

	void to_json(json&, const Tuner&);
	Tuner(std::ostream&);

	// Values passed to Planner

	State start_;
	State goal_;
	int branchout_;
	float goal_threshold_;
	float sampling_time_;
	std::vector<Control> pre_sample;
	ConstraintList control_constraints_;
	std::vector<bool> grid_active_;
	std::vector<float> grid_resolution_;
	int max_iterations_;

	// Tuner internal values

	// Keeps track of weather or not to output debug information to the console
	// during the configuration step.
	bool is_verbose;
	// The desired output stream for debug information
	std::ostream& out;

	// Struct which describes mandatory fields of the builder, essentially an
	// alias for a set of bitwise shifts that we can perform on an unsigned
	// int to keep track of set and unset fields.
	//
	// If you add a new mandatory field, just add it to the enum following the
	// pattern, and add a line like so to register that it has been set:
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
	// this->set = this->set | Mandatory::SomeParameter;
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// To make sure that it is checked in the `Tuner::init` method, add it to
	// the check at the start of the method following the pattern.
	struct Mandatory {
		enum {
			Start = (1 << 0),
			Goal = (1 << 1),
			Branchout = (1 << 2),
			GoalThreshold = (1 << 3),
			SamplingTime = (1 << 4),
			Samples = (1 << 5),
			Constraints = (1 << 6),
			Grid = (1 << 7),
			MaxIt = (1 << 8)
		};
	};

	/// Bitfield which describes which mandatory paramteres have been set
	unsigned set;
};

/// @brief error thrown by `Tuner` in case of invalid or missing configs
///
/// # Examples
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// throw ConfigError("implicit grid parameters size mismatch");
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class ConfigError : public std::runtime_error {
	// Make sure to pass a phrase as error message that sounds natural placed
	// in a sentence, making it easier for the exception catcher to provide a
	// user friendly error message.
public:
	explicit ConfigError(std::string const& error_message) :
		std::runtime_error(error_message)
	{}
};

} // namespace Config
