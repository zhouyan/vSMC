#ifndef VSMC_INTERNAL_COMMON_HPP
#define VSMC_INTERNAL_COMMON_HPP

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif // __STDC_CONSTANT_MACROS

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <utility>
#include <limits>
#include <map>
#include <set>
#include <deque>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

#ifdef __INTEL_COMPILER
// #pragma warning(push)
#pragma warning(disable:2196)
#pragma warning(disable:2536)
#endif // __INTEL_COMPILER

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wunique-enum"
#endif // __clang__

#include <Eigen/Dense>

#ifdef __INTEL_COMPILER
// #pragma warning(pop)
#endif // __INTEL_COMPILER

#ifdef __clang__
#pragma clang diagnostic pop
#endif // __clang__

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/version.hpp>
#include <vsmc/internal/function.hpp>
#include <vsmc/internal/type_traits.hpp>
#include <vsmc/internal/types.hpp>
#include <vsmc/internal/forward.hpp>

#endif // VSMC_INTERNAL_COMMON_HPP
