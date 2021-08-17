#include "sampling.h"

void sampling::convert_to_base(int number, int base, vector<int>* digits) {
    digits->clear();
    while (number > 0) {
        digits->push_back(number % base);
        number /= base;
    }
}

vector<Control> sampling::pre_random(const int branchout,
    const ConstraintList &cl) {

    vector<Control> pre_sample(branchout);

    for (int j = 0; j < branchout; ++j) {
        for (auto c : cl) {
            pre_sample[j].push_back(
                    (c.range() * ((float) rand() / RAND_MAX)) + c.min);
        }
    }

    return pre_sample;
}

vector<Control> sampling::pre_halton(const int branchout,
    const ConstraintList &cl) {

    vector<Control> pre_sample(branchout);

    int primes[] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53};
    vector<int> digits;
    vector<int>::iterator iter;
    float d, min, range;
    int power;

    for (int j = 0; j < branchout; j++) {
        for (int i = 0; i < cl.size(); i++) {
            convert_to_base(j, primes[i], &digits);
            d = 0;
            power = 1;
            for (iter = digits.begin(); iter != digits.end(); ++iter) {
                d += *iter / pow((double) primes[i], power);
                power++;
            }
            pre_sample[j].push_back(d * cl[i].range() + cl[i].min);
        }
    }

    return pre_sample;
}

vector<Control> sampling::pre_cubic(const int branchout,
    const ConstraintList &cl) {

    vector<Control> pre_sample(branchout);

    float min_range = 0; // lower bound of the range that we want to sample
    float max_range = 1; // upper bound of the range that we want to sample
    float full_range = 1;
    float dist_center = 0.5;
    float dist_width = 1;
    float center = 0.5;

    // Generate halton samples and apply pre_cupic transformation
    pre_sample = sampling::pre_halton(branchout, cl);

    for (int j = 0; j < cl.size(); ++j) {
        full_range = cl[j].range();
        min_range = cl[j].min;
        max_range = cl[j].max;

        center = (max_range + min_range) / 2;
        min_range -= center;
        max_range -= center;
        dist_center = -CUBEROOT(min_range / full_range);
        dist_width = (CUBEROOT(max_range / full_range)
                - CUBEROOT(min_range / full_range));

        for (int i = 0; i < branchout; ++i) {
            pre_sample[i][j] = (pre_sample[i][j]) * (dist_width);
            pre_sample[i][j] = CUBEOF(pre_sample[i][j] - dist_center)
                    + CUBEOF(dist_center);
            pre_sample[i][j] = pre_sample[i][j] * full_range + min_range
                    + center;
        }
    }

    return pre_sample;
}


float sampling::rand_float_range(float a, float b) {
	return ((b - a) * ((float) rand() / RAND_MAX)) + a;
}
