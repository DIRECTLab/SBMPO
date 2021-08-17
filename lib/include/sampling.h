#pragma once

#include <vector>

#include "definitions.h"
#include "controlConstraints.h"

namespace sampling {

/// @name Sampling
///
/// Sampling methods for exploring controls space, used as utilities to
/// pre-compute samples.
///
/// # Examples
///
/// Assuming we don't want fully pre-loaded samples
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// sbmpo::Planner p = config::Tuner::from_config(config)
///     .branchout(8) // branchout must match sample size
///     .samples(sampling::pre_halton(8,cl)) // pre-compute samples
///     .init(&model);
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///
/// @{

/// @brief Use the halton sequence to generate quasi-random samples
///
/// Effective for generating a sequence of numbers that appear random for most
/// purposes, but are well spaced with smaller gaps than normal pseudorandom
/// samples.
///
/// # Examples
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// sbmpo::Planner p = config::Tuner::from_config(config)
///     .branchout(8) // branchout must match sample size
///     .samples(sampling::pre_halton(8,cl)) // pre-compute samples
///     .init(&model);
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
vector<Control> pre_halton(const int branchout,
    const ConstraintList &cl);

/// @brief use a sequence of pseudorandom numbers to generate samples
vector<Control> pre_random(const int branchout,
    const ConstraintList &cl);

/// @brief transform halton samples using a cubic function
///
/// Allows tighter samples as we approach convergence
vector<Control> pre_cubic(const int branchout,
    const ConstraintList &cl);

/// @}

/// Return a random floating point number within the given range
float rand_float_range(float a, float b);

/// Converts a number to a certain base and puts the resulting sequence of
/// digits in a vector of int provided by the user.
///
/// \param number - Number to be converted.
/// \param base - Target base to convert the number.
/// \param digits - Pointer to a vector of int where the resulting sequences of
/// digits will be stored.
void convert_to_base(int number, int base, vector<int>* digits);

} // namespace sampling
