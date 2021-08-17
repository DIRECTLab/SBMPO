#include "config.h"

namespace config {

// Set default values for diagnostic output behaviour
Tuner::Tuner(): is_verbose(false), out(std::cerr) {}
Tuner::Tuner(std::ostream& o): is_verbose(true), out(o) {}
Tuner::~Tuner() {}

Tuner Tuner::from_config(const json& j) {
	return Tuner().load_config(j);
}

Tuner& Tuner::load_config(const json& config) {

	// If a parameter exists, load it, delegating the details of validation
	// to the respective setter method.
	if (config.find("verbose config") != config.end())
		this->verbose(config.at("verbose config").get<bool>());

	if (config.find("start") != config.end())
		this->start(config.at("start").get<State>());

	if (config.find("goal") != config.end())
		this->goal(config.at("goal").get<State>());

	// read fixed samples directly from config file
	if (config.find("samples") != config.end()
		&& config.at("samples").is_array()) {

		this->samples(
			config.at("samples").get<std::vector<std::vector<float>>>()
		);
	// use auto_samples helper method to generate pre samples
	} else if (config.find("samples") != config.end()) {

		auto vals = config.at("samples")
			.at("constraints")
			.get<std::vector<std::vector<float>>>();

		ConstraintList cl;
		for (auto el : vals)
			cl.add(el[0], el[1]);

		int b = config.at("samples").at("branchout");
		std::string type = config.at("samples").at("type");

		this->auto_samples(type, b, cl);
	}

	if (config.find("sampling time") != config.end())
		this->sampling_time(config.at("sampling time").get<float>());

	if (config.find("goal threshold") != config.end())
		this->goal_threshold(config.at("goal threshold").get<float>());

	if (config.find("branchout") != config.end())
		this->branchout(config.at("branchout").get<int>());

	if (config.find("max iterations") != config.end())
		this->max_iterations(config.at("max iterations").get<int>());

	if ((config.find("grid active") != config.end())
		&& config.find("grid resolution") != config.end()) {

		this->implicit_grid_params(
			config.at("grid active").get<std::vector<bool>>(),
			config.at("grid resolution").get<std::vector<float>>()
		);
	}

	if (config.find("constraints") != config.end()) {
		auto vals = config
			.at("constraints")
			.get<std::vector<std::vector<float>>>();

		ConstraintList cl;
		for (auto el : vals)
			cl.add(el[0], el[1]);

		this->control_constraints(cl);
	}

	return *this;
}


sbmpo::Planner Tuner::init(model::Model* model) {
	// Check to make sure all of the mandatory parameters have been set
	if (
		!((set & ( Mandatory::Start
			| Mandatory::Goal
			| Mandatory::Branchout
			| Mandatory::GoalThreshold
			| Mandatory::SamplingTime
			| Mandatory::Samples
			| Mandatory::Constraints
			| Mandatory::Grid
			| Mandatory::MaxIt))
		==
		( Mandatory::Start
			| Mandatory::Goal
			| Mandatory::Branchout
			| Mandatory::GoalThreshold
			| Mandatory::SamplingTime
			| Mandatory::Samples
			| Mandatory::Constraints
			| Mandatory::Grid
			| Mandatory::MaxIt))
		)
	{
		throw ConfigError("Unset mandatory parameters");
	}

	// Ensure size coherence among branchout factor, pre-sample, and constraint
	// list.
	if (pre_sample.size() != branchout_) {
		throw ConfigError("sample size does not match branchout");
	}

	if (pre_sample[0].size() != control_constraints_.size()) {
		throw ConfigError("constraint list does not match control size");
	}

	if (pre_sample[0].size() != model->control_dof()) {
		DEBUG("CONFIG => init, "
			<< "control_dof => "
			<< model->control_dof() << ", "
			<< "pre_sample[0].size() => "
			<< pre_sample[0].size()
		);
		throw ConfigError("model control and sample size mismatch");
	}

	if (this->is_verbose) {
		json debug;
		to_json(debug, *this);
		this->out << "Called init with values: "
			<< setw(4)
			<< debug
			<< std::endl;
	}

	// pass the model and internally set values into the constructor
	return sbmpo::Planner(
		model,
		this->start_,
		this->goal_,
		this->branchout_,
		this->max_iterations_,
		this->sampling_time_,
		this->goal_threshold_,
		this->pre_sample,
		this->control_constraints_,
		this->grid_resolution_,
		this->grid_active_
	);
}


Tuner& Tuner::start(State start) {
	this->start_ = start;

	this->set = this->set | Mandatory::Start;

	return *this;
}


Tuner& Tuner::goal(State goal) {
	this->goal_ = goal;

	this->set = this->set | Mandatory::Goal;

	return *this;
}


Tuner& Tuner::samples(std::vector<Control> pre_sample) {
	this->pre_sample = pre_sample;

	this->set = this->set | Mandatory::Samples;

	return *this;
}

Tuner& Tuner::auto_samples(std::string t, int b, ConstraintList cl) {

	this->branchout(b);
	this->control_constraints(cl);

	if (t == "halton") {
		this->samples(sampling::pre_halton(branchout_, control_constraints_));
	} else if (t == "random") {
		this->samples(sampling::pre_random(branchout_, control_constraints_));
	} else if (t == "cubic") {
		this->samples(sampling::pre_cubic(branchout_, control_constraints_));
	} else {
		throw ConfigError("unrecognized sampling type");
	}

	return *this;
}


Tuner& Tuner::branchout(unsigned int branches) {
	// Since we have a sensible default of 8, when zero is passed default to 8
	// and warn the user
	if (branches > 0) {
		this->branchout_ = branches;
	}
	else {
		if (this->is_verbose) {
			this->out << "Warning: invalid branchout of "
				<< branches
				<< ", it must be positive so setting to default of 8"
				<< std::endl;
		}
		this->branchout_ = 8;
	}

	this->set = this->set | Mandatory::Branchout;

	return *this;
}


Tuner& Tuner::goal_threshold(float threshold) {
	// Since we have a sensible default of 0.5, when zero or smaller is passed
	// default to 0.5 and warn the user if verbose config is desired
	if (threshold > 0) {
		this->goal_threshold_ = threshold;
	}
	else {
		if (this->is_verbose) {
			this->out << "Warning: invalid goal threshold of "
				<< threshold
				<< ", it must be positive so setting to default of 0.5"
				<< std::endl;
		}
		this->goal_threshold_ = 0.5;
	}

	this->set = this->set | Mandatory::GoalThreshold;

	return *this;
}


Tuner& Tuner::sampling_time(float sampling) {
	if (sampling < 0.1) {
		if (this->is_verbose) {
			this->out << "Warning: small sampling time \""
				<< sampling
				<< "\" can cause memory consumption to increase dramatically."
				<< std::endl;
		}
	}
	this->sampling_time_ = sampling;

	// Update bitfield of set parameters
	this->set = this->set | Mandatory::SamplingTime;

	return *this;
}


Tuner& Tuner::control_constraints(ConstraintList constraints) {

	this->control_constraints_ = constraints;

	this->set = this->set | Mandatory::Constraints;

	return *this;
}


Tuner& Tuner::implicit_grid_params(
	std::vector<bool> active,
	std::vector<float> resolution) {

	if (active.size() != resolution.size()) {
		throw ConfigError("implicit grid vectors size mismatch");
	}

	this->grid_active_ = active;
	this->grid_resolution_ = resolution;

	this->set = this->set | Mandatory::Grid;

	return *this;
}


Tuner& Tuner::max_iterations(int max) {

	this->max_iterations_ = max;

	this->set = this->set | Mandatory::MaxIt;

	return *this;
}


Tuner& Tuner::verbose(bool v) {

	this->is_verbose = v;

	return *this;
}

Tuner Tuner::with_output(std::ostream& o) { return Tuner(o); }

void Tuner::to_json(json& j, const Tuner& t) {
	j["start"] = t.start_;
	j["goal"] = t.goal_;
	j["branchout"] = t.branchout_;
	j["max iterations"] = t.max_iterations_;
	j["samples"] = t.pre_sample;
	j["sampling time"] = t.sampling_time_;
	j["goal threshold"] = t.goal_threshold_;
	j["grid resolution"] = t.grid_resolution_;
	j["grid active"] = t.grid_active_;
	for (auto c : control_constraints_) {
		j["constraints"].push_back({c.min, c.max});
	}
}

} // namespace config
