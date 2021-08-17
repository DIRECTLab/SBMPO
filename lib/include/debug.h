#pragma once

#ifdef DEBUG_BUILD

#include "json.h"
#include "definitions.h"
#include <chrono>
#include <fstream>
#include <iostream>

inline std::ostream& operator<< (std::ostream& o, const State xs) {
	nlohmann::json j = xs;
	o << j;
	return o;
}

namespace debug {

	const auto start = std::chrono::high_resolution_clock::now();
}

#define DEBUG(x) do { \
	auto finish = std::chrono::high_resolution_clock::now(); \
    std::clog \
		<< "DEBUG:" \
		<< std::chrono::duration_cast<std::chrono::nanoseconds>(finish-debug::start) \
			.count() \
		<< " | " \
		<< x << std::endl; \
	} while (false)

#else
#define DEBUG(x)
#endif
