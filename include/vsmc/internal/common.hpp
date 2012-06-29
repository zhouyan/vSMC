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

#include <Eigen/Dense>

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/version.hpp>
#include <vsmc/internal/function.hpp>
#include <vsmc/internal/type_traits.hpp>
#include <vsmc/internal/types.hpp>
#include <vsmc/internal/forward.hpp>

#ifdef NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, message)
#else // NDEBUG
#define VSMC_RUNTIME_ASSERT(cond, message) \
{ \
    if (!(cond)) \
        std::cerr \
            << "vSMC runtime assertion failed:" << std::endl \
            << message << std::endl; \
    else \
        ; \
    assert(cond); \
}
#endif // NDEBUG

#endif // VSMC_INTERNAL_COMMON_HPP
