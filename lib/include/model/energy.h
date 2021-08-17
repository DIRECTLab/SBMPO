#pragma once

#include <iostream>
#include <fstream>
#include <array>

#include "json.h"
#include "model.h"
#include "definitions.h"
#include "collision.h"

using nlohmann::json;

namespace model {

/// @brief Contains the kinodynamic model of the robot
///
/// Energy-based planning for robots of a few types, with the ability to
/// optinally optimize for the path which consumes the least energy or the path
/// that takes the shortest distance.
///
/// TODO: Finish model specific documentation of what states and contols
/// represent; get version from Nikil that explains all of the states and
/// controls
///
/// # Model States
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
/// state[0]; // x position
/// state[1]; // y position
/// state[2]; // heading angle w
/// state[3]; // wheel 1 angular velocity
/// state[4]; // wheel 2 angular velocity
/// state[5]; // turn radius
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// # Model controls
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
/// control[0]; // angular velocity of the robot
/// control[1]; // linear velocity of the robot
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class EnergyModel: public Model {
public:
	EnergyModel();
	EnergyModel(const json&);
	~EnergyModel();

	/// Control degrees of Freedom
	int control_dof();

	/// @brief Perform any initial calculation or initialization of model
	///
	/// Called once planning starts to optionally remove close obstacles if
	/// the user asks to do so.
	void init(const State&, const Control&);

	/// @brief Calculate new states from a given control and the previous state
	virtual State next_state(const State& state,
		const Control& control,
		float integ_step_size) final;

	/// @brief cost, in terms of energy or distance
	virtual float cost(const State& current,
		const State& next,
		const Control& control) final;

	/// @brief the estimated cost, in terms of energy or distance, to the goal
	virtual float heuristic(const State& current,
		const State& goal,
		const Control& control) final;

	/// @brief completion check for the optimization algorithm
	virtual bool converge(const State& current,
		const State& goal,
		float threshold) final;

	/// @brief perform collision checks on each node
	virtual bool is_valid(const State& current, const Control& control) final;

	/// @brief serialize `EnergyModel` values into json
	///
	/// # Example
	/// When we want to output a model, either for serialization or debugging
	/// purposes, the `to_json` instance allows us to seamlessly convert the
	/// model to a json file.
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
	/// using nlohmann::json;
	///
	/// Model model; // model with pre-assigned values
	/// json model_json = model; // implicit conversion
	/// std::cout << std::setw(4) << model_json;
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/// Will result in output something like so:
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{json}
	/// {
	///     "control_dof": 2,
	///     "model_dof": 3,
	///     "model type": 1,
	///     "vehicle width": 1.5,
	///     "wheel radius": 0.5,
	///     "energy planning": false,
	///     "expansion factor": 2,
	///     "slip x": [1, 0, 0],
	///     "slip y": [0, 1 ,0],
	///     "slip z": [0, 0, 1],
	///     "surface": "asphalt",
	///     "payload": 0,
	///     "slope": 4
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	///
	/// \param j - reference to the json object, should be an empty json object
	/// \param model - the model to read and convert into a json representation
	friend void to_json(json& j, const EnergyModel& model);

	/// @brief read values from json into `EnergyModel`
	///
	/// # Example
	/// Given a json configuration file with contents like so:
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{json}
	/// {
	///     "control_dof": 2,
	///     "model_dof": 3,
	///     "dof": 3,
	///     "model type": 1,
	///     "vehicle width": 1.5,
	///     "wheel radius": 0.5,
	///     "energy planning": false,
	///     "expansion factor": 2,
	///     "slip x": [1, 0, 0],
	///     "slip y": [0, 1 ,0],
	///     "slip z": [0, 0, 1],
	///     "surface": "asphalt",
	///     "payload": 0,
	///     "slope": 4
	/// }
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	/// We can read all of the associated values just by reading the file into
	/// a json object and then performing an implicit conversion like so:
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{cpp}
	/// using nlohmann::json;
	///
	/// ifstream config_file(file_path);
	/// json config_json;
	///
	/// config_file >> config_json;
	/// EnergyModel model = config_json; // implicit conversion
	/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	///
	/// \param j - json from which to read model parameters
	/// \param model - model to modify and write json values into
	friend void from_json(const json& j, EnergyModel& model);

private:
	// Model parameters, values the model depends on to calculate
	// energy consumption, travel time, distance, and collision
	// detection.

	int control_dof_;
	int model_dof;         /// Whether to do 2 or 3 dimensional planning
	int model_type;        /// Which slip model to use
	float rad;             /// Wheel radius
	float B;               /// Vehicle width
	bool energy_flag;      /// Whether to use energy-based planning
	float expF;            /// Expansion factor
	array<float,3> slip_x; /// Slip factors for slip model
	array<float,3> slip_y; /// Slip factors for slip model
	array<float,3> slip_z; /// Slip factors for slip model
	string surface;        /// Surface for slip model & expansion factors
	int slope;             /// Uniform slope of the entire surface
	int payload;           /// Mass of the payload the robot carries

	// Obstacle determination, used to store obstacle grid and check for
	// collision
	/// used for efficient testing against robot bounding box
	OBB robot;
	/// name of the obstacle file
	string obstacle_file;
	vector<Obstacle> obs_list;
	MultiGrid obs_grid;
	/// uniform grid resolution for obstacle detection
	float obs_grid_resolution;
	/// ignore obstacles withing `ignore_radius` at start
	bool ignore_close_obs;
	/// radius to use when ignore_close_obs is true
	float ignore_radius;
	/// Robot dimensions for collision detection
	float robot_width, robot_length;

	// Unconfigurable values, read in from sample files
	int Ntr;
	int Nh;
	float torque_in[15][1];
	float torque_out[15][1];
	float turn_rad[15];
	float heading[1];
	float slo;
	float tout_min;
	float tin_min;

	/// @brief euclidian 2D distance between states
	float distance(State a, State b, int dof);

	/// Read the torque input and output, turn radius, and heading angle files,
	/// where the model values are stored.
	///
	/// \param slope    - reads the slope of the surface.
	/// \param pload    - reads the payload of the vehicle.
	/// \param temp     - reads the file content.
	/// \param filename - defines the file name need to be open.
	/// \param i        - iterator variable.
	/// \param j        - iterator variable.
	/// \return VOID and fills the torque_in, torque_out, turn_rad, heading arrays.
	void readFile(string surface,int slope, int pload);

	/// Computes the distance between two 2D points.
	///
	/// \param x1 - x-coordinate of the first point.
	/// \param y1 - y-coordinate of the first point.
	/// \param x2 - x-coordinate of the second point.
	/// \param y2 - y-coordinate of the second point.
	/// \return Float representing the distance between the two points.
	float distance_2d(float x1, float y1, float x2, float y2);

	/// Computes the energy based heurstic between two 2D points.
	///
	/// \param x1    - x-coordinate of the first point.
	/// \param y1    - y-coordinate of the first point.
	/// \param x2    - x-coordinate of the second point.
	/// \param y2    - y-coordinate of the second point.
	/// \param theta - orientation of the vehicle.
	/// \param w     - angular velocity of vehicle wheels when moving straight.
	/// \param vel   - forward velocity of the vehicle.
	/// \return Float representing the energy based heuristic between two 2D points.
	float hpow_distance_2d(float x1, float x2, float y1, float y2, float theta,float w, float vel);

	/// Computes the energy based cost between two 2D points.
	///
	/// \param x1    - x-coordinate of the first point.
	/// \param y1    - y-coordinate of the first point.
	/// \param x2    - x-coordinate of the second point.
	/// \param y2    - y-coordinate of the second point.
	/// \param w_out - angular velocity of outer side wheels.
	/// \param w     - angular velocity of inner side wheels.
	/// \param t_rad - turn radius of the vehicle.
	/// \param vel   - forward velocity of the vehicle.
	/// \return Float representing the energy based cost between two 2D points.
	float pow_distance_2d(float x1, float y1, float x2, float y2, float w_out, float w_in, float t_rad, float vel);

	/// The time it takes to traverse between the two points at a given velocity.
	///
	/// \param x1  - x-coordinate of the first point.
	/// \param y1  - y-coordinate of the first point.
	/// \param x2  - x-coordinate of the second point.
	/// \param y2  - y-coordinate of the second point.
	/// \param vel - the velocity of the robot
	/// \return Float representing the traversal time
	float compute_time(float x1, float y1, float x2, float y2, float vel);

	/// @brief Import obstactles from text file
	void importObstacles(string filename);

	/// Computes the cell that corresponds to a given state, based on the resolution
	/// set for the grid.
	/// \param state - State to compute the corresponding cell.
	/// \param cell - Location where the corresponding cell will be stored.
	void calc_obs_grid_cell(const State &state, GridCell * cell);

	/// Fills the obstacle grid in init step, optionally removing very close
	/// obstacles.
	void fill_obs_grid(const State&);

	void import_obstacles(const string& filename);
};

} // namespace model
