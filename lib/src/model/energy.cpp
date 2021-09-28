#pragma once
#define SBMPO_DEBUG 1

#include "model/energy.h"

//unused default constructor might take out, rule of zero
model::EnergyModel::EnergyModel() {}

//non-default constuctor that uses initializes with a json object
//or I would think but there is an init function
//seems to be a pre-set up for the init
model::EnergyModel::EnergyModel(const json& j) {
	from_json(j, *this);
	DEBUG("MODEL => "
		<< "constructor " << this << ", "
		<< "json => "
		<< j
	);
}

//unused destructor might take out, rule of 0
model::EnergyModel::~EnergyModel(){}

//getter for control dof #need more background on what that means
int model::EnergyModel::control_dof() { return control_dof_; }


//set up variables from configs
void model::EnergyModel::init(const State& state, const Control& control) {
	// Import obstacles
	this->import_obstacles(this->obstacle_file);
	// Read sample file model parameters
	this->readFile(this->surface.c_str(), this->slope, this->payload);
	// Resize collision volume
	this->robot.resize(this->robot_length, this->robot_width);
	// Fill initial obstacle grid
	this->fill_obs_grid(state);

	DEBUG("MODEL => init, "
		<< "number of obstacles => "
		<< this->obs_list.size()
	);
	DEBUG("MODEL => init, "
		<< "energy planning => "
		<< this->energy_flag
	);
	DEBUG("MODEL => init, "
		<< "surface => "
		<< this->surface
	);
}


//energy flag is set in from_json
//which is started in non-default constructor
float model::EnergyModel::cost(const State& current,
	const State& next,
	const Control& control) {

	float cost;

	if(energy_flag){
		cost = this->pow_distance_2d(next[0], next[1], current[0], current[1], current[3], current[4],current[5], control[1]);
        if (current[0] == 10 && current[1] == 10) {
            cost = cost * 5;
        }
    }
	else
		cost = this->distance(current, next, model_dof);


	DEBUG("MODEL => Model::cost, "
		<< "current state => " << current << ", "
		<< "next state => " << next << ", "
		<< "cost => " << cost
	);

	return cost;
}

float model::EnergyModel::heuristic(const State& current,
	const State& goal,
	const Control& control) {

	float h;

	if (energy_flag)
		h = this->hpow_distance_2d(current[0], goal[0], current[1], goal[1], current[2], (control[1] / WHEEL_RADIUS), control[1]);
	else
		h = this->distance(current, goal, model_dof);

	DEBUG("MODEL => Model::heuristic, "
		<< "current state => " << current << ", "
		<< "goal state => " << goal << ", "
		<< "control => " << control << ", "
		<< "heuristic => " << h
	);

	return h;
}

State model::EnergyModel::next_state(const State& state,
	const Control& control,
	float integ_step_size) {

	float v_x, v_y, phi, w, v, radius, robot_vx, robot_vy, robot_w;

	State next(state.size(), 0);

	w = control[0];
	v = control[1];

	if (model_type == 1) {
		robot_vx = v;
		robot_vy = 0;
		robot_w = w/expF;
	} else {
		/*                                                    *
		 *           New Slip Model as from Kelly work        *
		 *                                                    */
		/* Robot Kinematic Model */
		/*
		 *   |  V_x  |     | x1  x2  x3 |   |   lin_v     |   *
		 *   |  V_y  |  =  | y1  y2  y3 | . |   ang_v     |   *
		 *   | omega |     | z1  z2  z3 |   | lin_v.ang_v |   *
		 */
		robot_vx = (slip_x[0]*v) + (slip_x[1]*w) + (slip_x[2]*v*w);
		robot_vy = (slip_y[0]*v) + (slip_y[1]*w) + (slip_y[2]*v*w);
		robot_w  = (slip_z[0]*v) + (slip_z[1]*w) + (slip_z[2]*v*w);
	}

	phi = state[2];
	next[2] = phi + robot_w*integ_step_size;

	// patch to fix round-off errors
	float sin_theta = sin(next[2]), cos_theta = cos(next[2]);

	if (abs(sin_theta) < 0.001) sin_theta = 0.0;
	if (abs(cos_theta) < 0.001) cos_theta = 0.0;

	v_x = (robot_vx * cos_theta) - (robot_vy * sin_theta);
	v_y = (robot_vx * sin_theta) + (robot_vy * cos_theta);

	next[0] = state[ 0 ] + (v_x) * integ_step_size;
	next[1] = state[ 1 ] + (v_y) * integ_step_size;
	// Wheel angular velocities of the robot
	next[3] = (robot_vx+((B*w)/2))/rad;
	next[4] = (robot_vx-((B*w)/2))/rad;

	// Correcting for small values
	/* Not able to understand the reason so far*/

	if (next[0]>-0.0001 && next[0]<0.0001)
		next[0] = 0;

	if (next[1]>-0.0001 && next[1]<0.0001)
		next[1] = 0;

	// Making sure that next[3] always got the outer wheel velocity
	if (robot_w < 0) {
		next[3] = (robot_vx-((B*w)/2))/rad;
		next[4] = (robot_vx+((B*w)/2))/rad;
	}

	// Computing turn radius
	if (w!=0) {
		next[5] = (robot_vx)/w;
	} else {
		next[5] = 1000;
	}

	DEBUG("MODEL => Model::next_state, "
		<< "state => " << state << ", "
		<< "next => " << next << ", "
		<< "control => " << control << ", "
		<< "robot omega => " << robot_w << ", "
		<< "v_x => " << v_x << ", "
		<< "v_y => " << v_y << ", "
		<< "sample time => " << integ_step_size
	);

	return next;
}

bool model::EnergyModel::is_valid(const State& current, const Control& control) {
	//checking the collision with obstacles
	GridCell obs_cell;
	this->calc_obs_grid_cell(current, &obs_cell);
	MultiGrid::iterator found_obs = obs_grid.find(obs_cell);
	if (found_obs != obs_grid.end()) {
		IndexList obs_test = found_obs->second;
		IndexList::iterator obs_it = obs_test.begin();
		for (; obs_it != obs_test.end(); ++obs_it) {
			if (robot.Intersect(current, obs_list[*obs_it], this->model_dof)) {
				return false;
			}
		}
	}
	return true;
}

bool model::EnergyModel::converge(const State& current,
	const State& goal,
	float threshold) {
	return this->distance(current, goal, model_dof) < threshold;
}

void model::to_json(json& j, const model::EnergyModel& model) {
	j = json{
		{"energy planning", model.energy_flag},
		{"control dof", model.control_dof_},
		{"model dof", model.model_dof},
		{"model type", model.model_type},
		{"expansion factor", model.expF},
		{"surface", model.surface},
		{"wheel radius", model.rad},
		{"vehicle width", model.B},
		{"payload", model.payload},
		{"slope", model.slope},
		{"slip x", model.slip_x},
		{"slip y", model.slip_y},
		{"slip z", model.slip_z},
		{"obstacle file", model.obstacle_file},
		{"obstacle grid resolution", model.obs_grid_resolution},
		{"ignore close obstacles", model.ignore_close_obs},
		{"ignore radius", model.ignore_radius},
		{"bounding box", {
			{"width", model.robot_width},
			{"length", model.robot_length}
		}}
	};
}

void model::from_json(const json& j, model::EnergyModel& model) {
	model.control_dof_ = j.at("control dof"); // Number of control parameters
	model.model_dof = j.at("model dof"); // Internal degrees of freedom
	model.model_type = j.at("model type");
	model.B = j.at("vehicle width"); //vehicle width
	model.rad = j.at("wheel radius"); //wheel radius
	model.energy_flag = j.at("energy planning"); // Energy or distance planning
	model.expF = j.at("expansion factor"); // Expansion factor
	model.slip_x = { j.at("slip x")[0], j.at("slip x")[1], j.at("slip x")[2] };
	model.slip_y = { j.at("slip y")[0], j.at("slip y")[1], j.at("slip y")[2] };
	model.slip_z = { j.at("slip z")[0], j.at("slip z")[1], j.at("slip z")[2] };
	model.surface = j.at("surface"); // Surface for slip model
	model.slope = j.at("slope");   // Uniform slope of surface
	model.payload = j.at("payload"); // Payload mass
	model.obstacle_file = j.at("obstacle file");
	model.obs_grid_resolution = j.at("obstacle grid resolution");
	model.ignore_close_obs = j.at("ignore close obstacles");
	model.ignore_radius = j.at("ignore radius");
	model.robot_width = j.at("bounding box").at("width");
	model.robot_length = j.at("bounding box").at("length");
}

void model::EnergyModel::readFile(string surface, int slope, int pload)
{
	float temp;
	char filename[256];
	char terrain[256];
	FILE *myfile;

	int i = 0;
	int j = 0;
	int radius=1000.0;

	slo = slope;
	sprintf(terrain,"%s",surface.c_str());

	sprintf(filename,"resources//kinematic//%s_turn_rad.txt",terrain);
	myfile = fopen(filename, "r");
	while ( fscanf(myfile, "%f", &temp) != EOF)
	{
		turn_rad[ i++ ] = (temp);
	}
	fclose(myfile);
	Ntr = i;
	Nh = 1;

	sprintf(filename,"resources//kinematic//%s_torque_in_%d_deg_%dkg_payload.txt",terrain,slope,pload);
	myfile= fopen(filename, "r");
	for(i=0;i<Ntr;i++)
	{
		for(j=0;j<Nh;j++)
		{
			fscanf(myfile,"%f", &temp);
			torque_in[i][j] = temp;
		}
	}
	fclose(myfile);

	sprintf(filename,"resources//kinematic//%s_torque_out_%d_deg_%dkg_payload.txt",terrain,slope,pload);
	myfile = fopen(filename, "r");
	for(i=0;i<Ntr;i++)
	{
		for(j=0;j<Nh;j++)
		{
			fscanf(myfile,"%f", &temp);
			torque_out[i][j] = temp;
		}
	}
	fclose(myfile);

	heading[0] = 0.0;
	for(int i = 1;i < Ntr;i++) {
		if(radius == turn_rad[i] ) {
			tout_min = torque_out[i][0];
			tin_min = torque_in[i][0];
		}
	}

}

float model::EnergyModel::compute_time(float x1, float y1, float x2, float y2, float vel)
{
	float d ;
	d = sqrt(pow(double(x1 - x2), 2.0) + pow(double(y1 - y2), 2.0));
	return d / vel;
}

float model::EnergyModel::distance_2d(float x1, float y1, float x2, float y2)
{
	return sqrt(pow(double(x1 - x2), 2.0) + pow(double(y1 - y2), 2.0));
}

float model::EnergyModel::hpow_distance_2d(float x1,
	float x2,
	float y1,
	float y2,
	float theta,
	float w,
	float vel)	 // normal heuristic function
{
	float time,Tin,Tout,Power,d,h,Pe;
	float p_in, p_out,t_in,t_out;
	float m1,m2,n1,n2;

	d = distance_2d(x1, y1, x2, y2);
	theta = theta - (PI)/2;
	h = (abs(y1-y2))*sin(slo*PI/180);
	Pe = Mass*gr*h;
	time = d / vel;

	t_in = tin_min;
	p_in = abs(t_in*w/eta) + (pow((t_in/(Kt*gRatio*eta)),2.0)*Re);

	t_out = tout_min;
	p_out = abs(t_out*w/eta) + (pow((t_out/(Kt*gRatio*eta)),2.0)*Re);
	Power = p_in + p_out;

	if (time != 0)   return (1*((Power*time)+(Pe)));
	else return 10000;
}

float model::EnergyModel::pow_distance_2d(
	float x1,
	float y1,
	float x2,
	float y2,
	float w_out,
	float w_in,
	float t_rad,
	float vel)
{
	float d, radius, time, delta,theta;
	float q11, q12, q21, q22;
	float Power, torque;
	float t_in = 0.0;
	float t_out = 0.0;
	float p_in,p_out;


	d = distance_2d(x1, y1, x2, y2);
    //mychange 
	time = d / vel/2;
	radius = fabs(t_rad);

	if (radius>1000) radius = 1000;
	if(radius<-1000) radius = 1000;


	for(int i = 1;i < Ntr;i++){
		if(radius > turn_rad[i-1] && radius <= turn_rad[i] ){
			int j=1;

			// Inner torque calculations
			q11 = torque_in[i-1][0];
			q22 = torque_in[i][0];
			t_in = q11 + ((q22-q11)*(radius-turn_rad[i-1])/(turn_rad[i]-turn_rad[i-1]));

			// Outer torque calculations
			q11 = torque_out[i-1][0];
			q22 = torque_out[i][0];
			t_out = q11 + ((q22-q11)*(radius-turn_rad[i-1])/(turn_rad[i]-turn_rad[i-1]));
			break;
		}
	}
	p_in = abs(t_in*w_in/eta) + (pow((t_in/(Kt*gRatio*eta)),2.0)*Re);
	p_out = abs(t_out*w_out/eta) + (pow((t_out/(Kt*gRatio*eta)),2.0)*Re);

	Power = p_in + p_out;

	if (Power<0) Power=0;

	if (time != 0)   return Power*time;
	else return 10000;  //return(torque);
}


float model::EnergyModel::distance(State a, State b, int model_dof) {
	if (model_dof == 2)
		return sqrt(
				pow(double(a[0] - b[0]), 2.0) + pow(double(a[1] - b[1]), 2.0));
	else
		return sqrt(
				pow(double(a[0] - b[0]), 2.0) + pow(double(a[1] - b[1]), 2.0)
						+ pow(double(a[2] - b[2]), 2.0));
}

void model::EnergyModel::fill_obs_grid(const State& state) {
	int num_obstacles = obs_list.size();
	float obs_radius = -1;

	int boundary_north, boundary_south, boundary_east, boundary_west,
		boundary_up, boundary_down;

	GridCell temp_cell(this->model_dof, 0);
	for (int obs = 0; obs < num_obstacles; ++obs) {
		if(this->ignore_close_obs){
			// Remove close obstacles as per user preference
			// TODO: FIND WAY TO GET THESE VALUES
			float robot_x = state[0];
			float robot_y = state[1];
			if(sqrt(pow(double(robot_x - this->obs_list[obs][0]), 2.0) + pow(double(robot_y - this->obs_list[obs][1]), 2.0)) <= this->ignore_radius){
				swap(this->obs_list[obs], this->obs_list.back());
					this->obs_list.pop_back();
				num_obstacles--;
			}
		}

		if (this->model_dof == 2)
			obs_radius = obs_list[obs][2];
		else if (this->model_dof == 3)
			obs_radius = obs_list[obs][3];

		boundary_west = floor(
				(obs_list[obs][0] - (obs_radius + (robot_width / 2)))
						/ obs_grid_resolution);
		boundary_east = floor(
				(obs_list[obs][0] + (obs_radius + (robot_width / 2)))
						/ obs_grid_resolution);
		boundary_south = floor(
				(obs_list[obs][1] - (obs_radius + (robot_length / 2)))
						/ obs_grid_resolution);
		boundary_north = floor(
				(obs_list[obs][1] + (obs_radius + (robot_length / 2)))
						/ obs_grid_resolution);
		if (this->model_dof == 3) {
			boundary_down = floor(
					(obs_list[obs][2] - (obs_radius + robot_length))
							/ obs_grid_resolution);
			boundary_up = floor(
					(obs_list[obs][2] + (obs_radius + robot_length))
							/ obs_grid_resolution);
		}

		for (int j = boundary_west; j <= boundary_east; ++j) {
			for (int k = boundary_south; k <= boundary_north; ++k) {
				temp_cell[0] = j;
				temp_cell[1] = k;
				if (this->model_dof == 3) {
					for (int l = boundary_down; l < boundary_up; ++l) {
						temp_cell[2] = l;
						obs_grid[temp_cell].push_back(obs);
					}
				} else if (this->model_dof == 2)
					obs_grid[temp_cell].push_back(obs); // obs-is the obstacle ID
			}
		}
	}
}

void model::EnergyModel::calc_obs_grid_cell(const State& state, GridCell * cell) {
	cell->resize(2); // gridding only on x&y
	for (int i = 0; i < this->model_dof; i++)
		(*cell)[i] = floor(state[i] / obs_grid_resolution);
}

void model::EnergyModel::import_obstacles(const string& filename) {
	this->obs_list.clear(); // clear existing obstacles from the list
	string line;
	/*----read obstacle file ----------------------*/
	ifstream obstacle_info(filename.c_str());
#ifdef DEBUG_BUILD
	ofstream file_copier("./results/obstacles.txt");
#endif
	while (getline(obstacle_info, line)) { // reading obstacles.txt file
#ifdef DEBUG_BUILD
		file_copier << line << endl;
#endif
		DEBUG("MODEL => import obstacles, "
			"file name => " << filename << ", "
			"obstacle => " << line
			<< std::endl
		);

		istringstream tokenizer(line);
		string token;
		Obstacle temp_obs(model_dof + 1, 0);
		for (int i = 0; i < (model_dof + 1); ++i) {
			getline(tokenizer, token, ' ');
			istringstream val(token);
			val >> temp_obs[i];
		}
		this->obs_list.push_back(temp_obs); // adding obstacle to the global list.
	}
#ifdef DEBUG_BUILD
	file_copier.close();
#endif
	obstacle_info.close();
	/*----done reading obs file -------------------*/
}
