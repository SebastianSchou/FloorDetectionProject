#ifndef HELPER_FUNCTION_HPP
#define HELPER_FUNCTION_HPP

#include <chrono>

namespace HelperFunction {
	float msUntilNow(const std::chrono::steady_clock::time_point& start);
};

#endif // HELPER_FUNCTION_HPP
