/// @file sbmpo.cpp
///
/// @brief SBMPO function implementation.

#include "sbmpo.h"

sbmpo::Planner::Planner(
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
	) :
		model(model),
		start_global(start),
		goal_global(goal),
		branchout(branchout),
		max_iteration_limit(max_iteration_limit),
		T(T),
		goal_threshold(goal_threshold),
		samples(samples),
		control_constraints(control_constraints),
		grid_active(grid_active),
		grid_resolution(grid_resolution)
{}

sbmpo::Planner::~Planner() {}

void sbmpo::Planner::clear() {
	Q.clear();
	graph.clear();
	grid.clear();
}

Trajectory sbmpo::Planner::compute_trajectory(const State& start, const State& end) {
	this->start_global = start;
	this->goal_global = end;
	DEBUG("PLANNER => compute_trajectory, "
		<< "new start state => "
		<< start_global
		<< "new goal state => "
		<< goal_global
	);
	return this->compute_trajectory();
}

Trajectory sbmpo::Planner::compute_trajectory() {

	DEBUG("PLANNER => compute_trajectory, "
		<< "goal state => "
		<< goal_global
	);

#ifdef VISUALIZER
	config::json visualizer;
#endif

	float _heuristic;
	Trajectory result;

	// Set the data for starting vertex and add it to the graph and the grid
	GridCell cell;

	Control c(model->control_dof(), 0);
	start_id = 0;
	goal_id = 1;

	model->init(start_global, c);

	Vertex v_goal(1, -1, 0, goal_global, c);

	_heuristic = model->heuristic(start_global,
		goal_global,
		samples[0]); // use the first sample when creating the first heursitic

	Vertex v_start(0, -1,
		_heuristic,
		start_global,
		c);
	v_start.rhs = 0;
	graph.push_back(v_start);

	if (cell_empty(start_global, &cell)) { /*test*/
		pair<GridCell, Index> grid_item;
		grid_item.first = cell;
		grid_item.second = start_id;
		grid.insert(grid_item);
	}

	//set the data for the goal vertex and add it to the graph.
	graph.push_back(v_goal);

	//add the starting vertex to the priority queue.
	graph[start_id].key = calculate_key(start_id);
	graph[goal_id].key = calculate_key(goal_id);
	q_insert(0);

	//compute the shortest path.
	int u = 0, iteration_counter = 0;

	gettimeofday(&tod1, NULL);
	while (iteration_counter++ < this->max_iteration_limit) {

		//check the termination conditions.
		Key topkey = q_top_key();

		Key goalkey = calculate_key(goal_id);

		//get the first thing in the queue and remove it from the queue.
		u = q_top();
		q_pop();
		if (u < 0) {
			throw PlanningError("queue is empty");
		}
		DEBUG("PLANNER => Q, "
			<< "g + h => " << topkey.first << ",  "
			<< "h => " << topkey.second << ", "
			<< "graph[u].h =>  " << graph[u].h << ", "
			<< "graph[u].rhs =>  " << graph[u].rhs << ", "
			<< "graph[u].g =>  " << graph[u].g << ", "
			<< "graph id => " << u << ", "
			<< "state => " << graph[u].state
		);

#ifdef VISUALIZER
		json node;
		int p = u;
		while (graph[p].parent_id != -1) {
			node["parents"].push_back(graph[p].parent_id);
			node["trajectory"].push_back(graph[p].state);

			p = graph[p].parent_id;
		}

		node["rhs"] = graph[u].rhs;
		node["h"] = graph[u].h;
		node["state"] = graph[u].state;

		visualizer["iterations"].push_back(node);
#endif

		//check if the goal has been reached, if it has, then quit!
		//if it is within goal boundary
		if (model->converge(graph[u].state, goal_global, goal_threshold)) {
			DEBUG("PLANNER => converged, "
				<< "converged state => " << graph[u].state << ", "
				<< "goal => " << goal_global << ", "
				<< "threshold => " << goal_threshold
			);
			break;
		}

		//generate children for the current vertex u.
		generate_children(u, iteration_counter, control_constraints);

		int s;
		if (graph[u].g > graph[u].rhs) {
			graph[u].g = graph[u].rhs;
			q_remove(u);
			for (int i = 0; i < (int) graph[u].succ.size(); i++) {
				s = graph[u].succ[i].neighbor_id;
				if (s != start_id
						&& graph[s].rhs > graph[u].g + graph[u].succ[i].cost) {

					graph[s].rhs = graph[u].g + graph[u].succ[i].cost;
					graph[s].parent_id = u;
					graph[s].control = graph[u].succ[i].control_fwd;

					graph[s].state = graph[u].succ[i].state_fwd;

					update_vertex(s);
				}
			}
		} else {
			graph[u].g = FLT_MAX;
			update_vertex(u);
			for (int i = 0; i < (int) graph[u].succ.size(); i++) {
				s = graph[u].succ[i].neighbor_id;
				if (s != (int) start_id && (int) graph[s].parent_id == u) {
					update_rhs(s);
				}
			}
		}
	}
	gettimeofday(&tod2, NULL);
	result.computation_time = (tod_diff(tod2, tod1) / 1000000.0);
	// if the algorithm has exceeded max allowed iteration then throw exception
	if (iteration_counter > max_iteration_limit) {
		throw PlanningError("failed to converge within iteration limit");
	}

	//Section for exporting results in file--------------------------

	int parent, count = 0;
	float power = 0, ttime = 0;
	while (true) {
		count++;
		result.trajectory.push_back(graph[u]);
		parent = u;
		if ((int) graph[u].parent_id == -1)
			break;

		u = graph[u].parent_id;

		result.cost += model->cost(graph[parent].state, graph[u].state, samples[0]);
	}
	reverse(result.trajectory.begin(), result.trajectory.end());

	DEBUG("PLANNER => results, "
		<< "total cost => "
		<< result.cost
	);
	DEBUG("PLANNER => results, "
		<< "computation time => "
		<< (tod_diff(tod2, tod1) / 1000000.0)
		<< "seconds"
	);
	#ifdef DEBUG_BUILD
	config::json output = result;
	#endif
	DEBUG("PLANNER => results, "
		<< "json => "
		<< output
	);

#ifdef VISUALIZER
	std::ofstream vis_file("results/visualizer.json");
	vis_file << visualizer << std::endl;
#endif

	return result;
	//Section ends ---------------------------------------------------
}

Key sbmpo::Planner::calculate_key(int v) {
	Key k;
	float g = min(graph[v].g, graph[v].rhs);
	k.first = g + (graph[v].h);
	k.second = graph[v].h;
	return k;
}

void sbmpo::Planner::generate_children(Index u_id, int iteration, ConstraintList control_constraints) {
	GridCell cell;
	float _heuristic, _cost;

	Control sample(control_constraints.size());

	for (int i = 0; i < branchout; i++) {
		// Create corresponding states for the new child

		State state = model->next_state(graph[u_id].state,
			samples[i],
			T);

		bool isvalid = model->is_valid(state, samples[i]);
		DEBUG("PLANNER => generate children, "
			<< "branchout => " << i << ", "
			<< "is valid => " << isvalid << ", "
			<< "graph id => " << u_id << ", "
			<< "current state => " << graph[u_id].state << ", "
			<< "next state => " << state << ", "
			<< "sample => " << samples[i]
		);

		if (!isvalid) { break; }

		// Check that grid cell for the new state is empty and valid
		if (isvalid && cell_empty(state, &cell)) {
			//prepare the new vertex
			Vertex child_v;
			child_v.control = samples[i];
			child_v.id = graph.size();
			child_v.parent_id = u_id;
			child_v.queue_id = -1;
			child_v.g = FLT_MAX;
			child_v.rhs = FLT_MAX;
			child_v.state = state;

			child_v.h = model->heuristic(
				child_v.state,
				graph[goal_id].state,
				samples[i]);

			Neighbor child_n;

			child_n.cost = model->cost(
				graph[u_id].state,
				child_v.state,
				samples[i]);

			child_n.neighbor_id = u_id;
			child_n.control_fwd = samples[i];
			child_n.state_fwd = state;
			child_v.pred.push_back(child_n);

			// Add the new vertex to the graph and the grid.
			graph.push_back(child_v);
			pair<GridCell, Index> grid_item;
			grid_item.first = cell;
			grid_item.second = child_v.id;
			grid.insert(grid_item);

			// Update the neighbors of the current vertex, namely, these are the
			// successors.
			Neighbor un;
			un.cost = child_n.cost;
			un.neighbor_id = child_v.id;
			un.control_fwd = samples[i];
			un.state_fwd = state;
			graph[u_id].succ.push_back(un);

		} else if (isvalid) {
			// If the cell is occupied add the neighbor data to u and the vertex
			// already in the cell.
			Neighbor un;

			un.cost = model->cost(graph[u_id].state,
				state,
				samples[i]);

			un.neighbor_id = grid[cell];
			un.control_fwd = samples[i];
			un.state_fwd   = state;

			graph[u_id].succ.push_back(un);

			Neighbor child_n;
			child_n.cost        = un.cost;
			child_n.neighbor_id = u_id;
			child_n.control_fwd = samples[i];
			child_n.state_fwd   = state;
			graph[grid[cell]].pred.push_back(child_n);
		}
	}
}

void sbmpo::Planner::update_vertex(int u) {
	if (graph[u].g != graph[u].rhs) {
		graph[u].key = calculate_key(u);
		q_insert(u);
	} else {
		q_remove(u);
	}
}

void sbmpo::Planner::update_rhs(int u) {
	int p;
	graph[u].rhs = FLT_MAX;
	graph[u].parent_id = -1;
	for (int i = 0; i < (int) graph[u].pred.size(); i++) {
		p = graph[u].pred[i].neighbor_id;
		if (graph[u].rhs > graph[p].g + graph[u].pred[i].cost) {
			graph[u].rhs = graph[p].g + graph[u].pred[i].cost;
			graph[u].parent_id = p;
			graph[u].control[0] = graph[u].pred[i].control_fwd[0];
			graph[u].control[1] = graph[u].pred[i].control_fwd[1];

			graph[u].state = graph[u].pred[i].state_fwd;
		}
	}
	update_vertex(u);
}

/* ------------------- Grid Cell manipulation ------------------ */
bool sbmpo::Planner::cell_empty(const State& state, GridCell* cell) {
	calc_grid_cell(state, cell);
	return (grid.find((*cell)) == grid.end());
}

void sbmpo::Planner::calc_grid_cell(const State &state, GridCell * cell) {

	cell->resize(state.size());
	for (int i = 0; i < (int) cell->size(); i++)
		if (grid_active[i] == true)
			(*cell)[i] = floor(state[i] / grid_resolution[i]);
}

/*------------ Priority Queue manipulation section -------------- */

void sbmpo::Planner::percolate_down(int hole, int vid) {
	int hole_shift = hole + 1;
	int child;
	if (!Q.empty()) {
		for (; 2 * hole_shift <= (int) Q.size(); hole_shift = child) {
			child = 2 * hole_shift;
			if (child != (int) Q.size()
					&& compare_keys(graph[Q[child]].key,
							graph[Q[child - 1]].key))
				++child;
			if (compare_keys(graph[Q[child - 1]].key, graph[vid].key)) {
				Q[hole_shift - 1] = Q[child - 1];
				graph[Q[hole_shift - 1]].queue_id = hole_shift - 1;
			} else
				break;
		}
		Q[hole_shift - 1] = vid;
		graph[Q[hole_shift - 1]].queue_id = hole_shift - 1;
	}
}

void sbmpo::Planner::percolate_up(int hole, int vid) {
	int hole_shift = hole + 1;
	if (!Q.empty()) {
		for (;
				hole_shift > 1
						&& compare_keys(graph[vid].key,
								graph[Q[hole_shift / 2 - 1]].key); hole_shift /=
						2) {
			Q[hole_shift - 1] = Q[hole_shift / 2 - 1];
			graph[Q[hole_shift - 1]].queue_id = hole_shift - 1;
		}
		Q[hole_shift - 1] = vid;
		graph[Q[hole_shift - 1]].queue_id = hole_shift - 1;
	}
}

void sbmpo::Planner::percolate_up_or_down(int hole, int vid) {
	int hole_shift = hole + 1;
	if (!Q.empty()) {
		if (hole_shift > 1
				&& compare_keys(graph[vid].key,
						graph[Q[hole_shift / 2 - 1]].key))
			percolate_up(hole_shift - 1, vid);
		else
			percolate_down(hole_shift - 1, vid);
	}
}

Key sbmpo::Planner::q_top_key() {
	if (Q.empty()) {
		Key k = make_pair(FLT_MAX, FLT_MAX);
		return k;
	}
	return graph[Q.front()].key;
}

int sbmpo::Planner::q_top() {
	if (Q.empty())
		return -1;
	return Q.front();
}

void sbmpo::Planner::q_pop() {
	if (!Q.empty()) {
		graph[Q.front()].queue_id = -1;
		percolate_down(0, Q.back());
		Q.pop_back();
	}
}

bool sbmpo::Planner::compare_keys(Key k1, Key k2) {
	if (k1.first < k2.first)
		return true;
	if ((k1.first == k2.first) && (k1.second < k2.second))
		return true;
	return false;
}

void sbmpo::Planner::q_insert(int vid) {
	if (vid < (int) graph.size()) {
		if (graph[vid].queue_id == -1) { // Vertex not currently in Q
			Q.push_back(vid);
			percolate_up(Q.size() - 1, vid);
		} else
			percolate_up_or_down(graph[vid].queue_id, vid);
	}
}

void sbmpo::Planner::q_remove(int vid) {
	if (vid < (int) graph.size()) {
		if (graph[vid].queue_id != -1 && !Q.empty()) { // Vertex currently in Q
			percolate_up_or_down(graph[vid].queue_id, Q.back());
			graph[vid].queue_id = -1;
			Q.pop_back();
		}
	}
}

/* ------------------------------- Logging section ----------------------- */
void sbmpo::Planner::store_graph(char *filename) {
	ofstream graph_info;
	graph_info.open(filename);
	int j;
	for (int i = 0; i < graph.size(); i++) {
		for (j = 0; j < (graph[i].state.size() - 1); ++j) {
			graph_info << graph[i].state[j] << " ";
		}
		graph_info << graph[i].state[j] << "\n";
	}
	graph_info.close();
}

void sbmpo::Planner::store_trajectory(char* output_file, const Trajectory& path) {
	if (path.trajectory.size() <= 0) return;
	ofstream fout;
	int _num_states = path.trajectory[0].state.size();
	printf("Creating file %s\n", output_file);
	fout.open(output_file);
	if (fout.fail()) {
		printf("Unable to create output file '%s'\n", output_file);
		fout.clear();
		return;
	}

	fout << fixed << setprecision(6);
	for (int i = 0; i < (int) path.trajectory.size(); ++i) {
		// number of states
		fout << path.trajectory[i].id << "\t";
		for (int j = 0; j < _num_states - 1; ++j)
			fout << path.trajectory[i].state[j] << "\t";
		fout << path.trajectory[i].state[_num_states - 1] << endl;
	}

	fout.close();
	fout.clear();
}

void sbmpo::Planner::store_controls(char *filename, vector<Control> control_list) {
	int _num_controls = control_list[0].size();
	ofstream fout;
	printf("Creating file %s\n", filename);
	fout.open(filename);
	if (fout.fail()) {
		printf("Unable to create output file '%s'\n", filename);
		fout.clear();
		return;
	}

	fout << fixed << setprecision(6);
	for (int i = 0; i < control_list.size(); ++i) {
		// number of controls
		for (int j = 0; j < _num_controls - 1; ++j)
			fout << (control_list[i])[j] << "\t";
		fout << (control_list[i])[_num_controls - 1] << endl;
	}
	fout.close();
	fout.clear();
}

/** ------------------------- Misc section ------------------------ */
long int sbmpo::Planner::tod_diff(const timeval& tod1, const timeval& tod2) {
	long t1, t2;
	t1 = tod1.tv_sec * 1000000 + tod1.tv_usec;
	t2 = tod2.tv_sec * 1000000 + tod2.tv_usec;
	return t1 - t2;
}
