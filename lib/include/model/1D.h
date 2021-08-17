#pragma once

#include <iostream>
#include <fstream>
#include <array>
#include <stdexcept>

#include "../json.h"
#include "../model.h"
#include "../definitions.h"
#include "../collision.h"

using nlohmann::json;

namespace model {

/// @brief Contains 1-dimension double integrator models used for the workshop
class DoubleIntegrator: public Model {
public:
    DoubleIntegrator() {};
    ~DoubleIntegrator() {};
    DoubleIntegrator(const json&);

    /// Control degrees of freedom
    int control_dof();

    /// @brief Perform any initial calculation or initialization of model
    ///
    /// Called once planning starts to optionally remove close obstacles if
    /// the user asks to do so.
    void init(const State& state, const Control& control);

    State next_state(const State& state,
        const Control& control,
        float integ_step_size);

    /// @brief cost from current to next state
    ///
    /// When the velocity aware or distance optimal heuristic are selected,
    /// simply the euclidian distance from one state to the next.
    ///
    /// When the time-optimal
    float cost(const State& current,
        const State& next,
        const Control& control);

    /// @brief heuristic in terms of
    float heuristic(const State& current,
        const State& goal,
        const Control& control);

    bool converge(const State& current,
        const State& goal,
        float threshold);

    bool is_valid(const State& current, const Control& control) {
        return true;
    }

    friend void to_json(json& j, const DoubleIntegrator& model);

    friend void from_json(const json& j, DoubleIntegrator& model);

private:
    State start_;

    /// Maximum acceleration that we can achieve with our model
    double acc_max;

    /// Time-optimal planning requires knowledge of the interation time-step to
    /// calculate the proper cost.
    double time_step;

    /// velocity aware, time optimal, minimum distance
    std::string heuristic_type;

    /// The finite difference of the values, same as 1-dimensional distance
    double distance(const double, const double);

    /// @brief Velocity-aware heuristic
    double velocity_aware(const State&, const State&);

    /// @brief time_optimal planning
    double time_optimal(const State&, const State&);
};

inline void to_json(json& j, const DoubleIntegrator& model) {
    j["control dof"] = 1;
    j["heurstic"] = model.heuristic_type;
    j["max acceleration"] = model.acc_max;
    j["time step"] = model.time_step;
}

inline void from_json(const json& j, DoubleIntegrator& model) {

    model.heuristic_type = j.at("heuristic").get<std::string>();
    model.acc_max = j.at("max acceleration").get<double>();
    model.time_step = j.at("time step").get<double>();

    DEBUG("MODEL => from_json, "
        << "heuristic => " << model.heuristic_type << ", "
        << "max acceleration => " << model.acc_max << ", "
        << "time step => " << model.time_step
    );
}

}
