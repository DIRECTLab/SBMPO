#include "model/1D.h"



model::DoubleIntegrator::DoubleIntegrator(const json& j) {
    from_json(j, *this);

    #ifdef DEBUG_BUILD
    json debug = *this;
    #endif
    DEBUG("MODEL => " << this << ", "
        << "config values => " << debug
    );
}


int model::DoubleIntegrator::control_dof() { return 1; }


void model::DoubleIntegrator::init(const State& state, const Control& control) {
    this->start_ = state;

    DEBUG("MODEL => init, "
        << "control_dof => " << this->control_dof() << ", "
        << "start state => " << this->start_ << ", "
        << "control => " << control
    );
}


State model::DoubleIntegrator::next_state(const State& state,
    const Control& control,
    float integ_step_size) {

    State newState;
    State states = state;
    newState.resize(state.size());

    double command_Force = 0.;
    double max_Force = 0.0;
    double min_Force = -0.0;

    double J = 0.05;   // not used in robotic frame
    double m = 22.5;   // mass of the robot
    double g = 9.81;   // gravity constant
    double r = 0.1075; // radius of the robot wheels

    // terrain model Torque_resistance = (b*vel) + R_res, For robotic frame divide Torque_resistance by radius of the wheel
    double R_res = 0.0; // zero speed Rolling resistance considering left wheel
    double b = 0.0; // considering left wheel

    // Resistance due to slope
    double G = 0.0;

    /******************************* Terrain Parameters **********************************/
    double d = 1.22; // length of wooden planks in meters

    // HACK: 92 cm (0.92) is the actual veg_len, changed to add conservatism
    double veg_len = 1.22;
    double seg1 = d;
    double seg2 = 2*seg1;
    double seg3 = seg2 + seg1;

    // Motor Model //
    double w_nom = 487.16;
    double T_nom = 0.2775;
    double K_n = 0.0;
    double K_t = 0.023;
    double V_batt = 14.0;
    double gR = 49.8;
    double eff = 0.76;
    double i_max = 5;
    double i_perc = 30; //percentage of current limited to
    double i_lim = 0.0;
    double max_F_set = 0.0;
    double grad = 0.0;
    double T_s = 0.0;
    double w_n = 0.0;
    double wn_wheel = 0.0;
    double Ts_wheel = 0.0;
    double A = 0.0;

    double R_res_wood = 7.865/2.0;  // this is resistance for half side of the vehicle
    R_res_wood = 0;
    double R_res_veg = 27.5929/2.0; // I am going to assume that the veg is only a high resistance term
    R_res_veg = 0;
    // this is to simulate transition region between the two surfaces
    double x_trans = 0.15;
    x_trans = 0;

    K_n = 1/K_t;
    grad = w_nom/T_nom;
    T_s = K_n*V_batt/grad;
    w_n = K_n*V_batt;
    wn_wheel = w_n/gR;
    Ts_wheel = T_s*gR;
    i_lim = i_max*i_perc/100;

    A = -wn_wheel/Ts_wheel; // grad = wn_nom/Ts_nom

    max_Force = 2*((state[1])/(A*r) + (Ts_wheel))/r; // Force corresponding to Max Torque
    max_F_set = K_t*i_lim*gR*eff/r; // Force corresponding to Max Torque Set

    if(max_Force > max_F_set)
        max_Force = max_F_set;

    min_Force = -max_Force;

    // Motor model ends here//

    // Setting terrain parameters according to the current position
    // Includes smoothing function while transitioning from one terrain to another

    // Assuming (0,0) is at the start of the actual vegetation
    // xtrans defines half of the transition

    if((state[0] < -x_trans )) {
        R_res = R_res_wood;
        b = 3.36/2.0;
        b=0;
    }
    // Transition from one terrain to another at the beginning
    else if ((state[0] >= -x_trans) && (state[0] < (x_trans))) {
        R_res = R_res_veg + ((R_res_veg-R_res_wood)*(state[0]-x_trans)/(2*x_trans));
        b = 3.36/2.0;
        b=0;
    }
    else if ((state[0] >= x_trans) && (state[0] < (veg_len-x_trans))) {
        R_res = R_res_veg;
        b = 0.0; // N/(m/s) // vegetation
    }
    // Transition from one terrain to another at the end
    else if ((state[0] >= (veg_len-x_trans)) && (state[0] < (veg_len+x_trans))) {
        R_res = R_res_veg + ((R_res_wood-R_res_veg)*(state[0]-veg_len + x_trans)/(2*x_trans));
        b = 3.36/2.0; // wood
        b = 0;
    }
    else {
        R_res = R_res_wood;
        b = 3.36/2.0; // N/(m/s) // Wood
        b=0;
    }

    // evaluation of the sample generated
    states[2] = control[0];

    // Changing the sign of the rolling resistance according to the direction of the velocity

    if (state[1] >= 0 && R_res >= 0)
        R_res = R_res;
    else if (state[1] >= 0 && R_res < 0)
        R_res = -R_res;
    else if (state[1] < 0 && R_res >= 0)
        R_res = -R_res;
    else if (state[1] < 0 && R_res < 0)
        R_res = R_res;

    // Computing the force requirment for the required motion

    command_Force  =  ((0.5*m) * (states[2])) + G + R_res + b*state[1];

    // Computing the linear velocity and position from the unsaturated linear acceleration
    states[1] = state[1] + states[2]*integ_step_size; // linear velocity
    states[0] = state[0] + states[1]*integ_step_size; // x_position

    // Updating the computed states to the new states
    newState[0] = states[0];
    newState[1] = states[1];
    newState[2] = states[2];

    DEBUG("MODEL => Model::next_state, "
        << "state => " << states << ", "
        << "next => " << newState << ", "
        << "control => " << control << ", "
        << "sample time => " << integ_step_size
    );

    return newState;
}



float model::DoubleIntegrator::cost(const State& current,
    const State& next,
    const Control& control) {

    double cost;
    if (this->heuristic_type == "time optimal") {
        cost = this->time_step;
    } else {
        this->distance(current[0],next[0]);
    }

    DEBUG("MODEL => Model::cost, "
        << "current state => " << current << ", "
        << "next state => " << next << ", "
        << "cost => " << cost
    );

    return cost;
}



float model::DoubleIntegrator::heuristic(const State& current,
    const State& goal,
    const Control& control) {


    double h;
    if (this->heuristic_type == "distance optimal") {
        h = this->distance(goal[0], current[0]);
    }
    if (this->heuristic_type == "velocity aware") {
        h = this->velocity_aware(current, goal);
    }
    if (this->heuristic_type == "time optimal") {
        h = this->time_optimal(current, goal);
    }

    DEBUG("MODEL => Model::heuristic, "
        << "type => " << this->heuristic_type << ", "
        << "current state => " << current << ", "
        << "goal state => " << goal << ", "
        << "control => " << control << ", "
        << "heuristic => " << h
    );

    return h;
}


bool model::DoubleIntegrator::converge(const State& current,
    const State& goal,
    float threshold) {

    return threshold > this->distance(goal[0], current[0])
        && 0.1 > this->distance(goal[1], current[1]);
}


double model::DoubleIntegrator::distance(const double a, const double b) {
    return abs(a - b);
}


double model::DoubleIntegrator::velocity_aware(const State& current,
    const State& goal) {

    double h;
    double del_x;

    if (current[0] < goal[0]) {
        if(current[1] < 0) {
            del_x = sqrt(pow((-current[1]*current[1]/(2*this->acc_max)), 2.0));
            h = 2*del_x+(goal[0]-current[0]);
        } else {
            del_x = sqrt(pow((-current[1]*current[1]/(-2*this->acc_max)), 2.0));
            if (del_x > (goal[0]-current[0])) {
                h = 2*del_x - (goal[0]-current[0]);
            } else {
                h = goal[0] - current[0];
            }
         }
    } else {
        if(current[1]<=0) {
            del_x = sqrt(pow((-current[1]*current[1]/(2*this->acc_max)), 2.0));
            if (del_x > (current[0]-goal[0])) {
                h = 2*del_x - (current[0]-goal[0]);
            } else {
                h = current[0] - goal[0];
            }
        } else {
            del_x = sqrt(pow((-current[1] * current[1]/(-2*this->acc_max)), 2.0));
            h = 2*del_x+(current[0]-goal[0]);
        }
    }
    return h;
}


double model::DoubleIntegrator::time_optimal(const State& current,
    const State& goal) {
    /*In SBMPC.cpp new_state is placed in curr_state*/

    double temp[2];
    double a,b; // maximum constant acceleration and de-acceleration
    double tf = 0.; // computed time (heuristic cost)

    b = this->acc_max;
    a = this->acc_max;

    temp[0] = current[0] - goal[0]; // offsetting the goal position to zero
    temp[1] = current[1];

    if ((((temp[0] + (temp[1]*fabs(temp[1])/2.0/b)) >0) && temp[0] > 0)
        ||  ((temp[0] + (temp[1]*fabs(temp[1])/2.0/a)) >=0)) {

        double A, B, C;

        A =  1.0;
        B = -2.0/a * temp[1];
        C = -2.0*(a+b)*temp[0]/a/b - temp[1]*temp[1]/a/b;

        tf = (-B + sqrt(B*B-4.0*A*C))/2.0/A;
    }

    else if (((temp[0] + (temp[1]*fabs(temp[1])/2.0/b)) <= 0)
        ||  (((temp[0] + (temp[1]*fabs(temp[1])/2.0/a)) < 0) && (temp[0] < 0)) ) {

        double A, B, C;

        A = 1.0;
        B = 2.0/b * temp[1];
        C = 2.0*(a+b)*temp[0]/a/b - temp[1]*temp[1]/a/b;
        tf = (-B + sqrt(B*B-4.0*A*C))/2.0/A;
    }

    return tf;
}
