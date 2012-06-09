#ifndef VSMC_INTERNAL_FUNCTION_HPP
#define VSMC_INTERNAL_FUNCTION_HPP

#include <vsmc/internal/config.hpp>

#ifdef VSMC_USE_STD_FUNCTION

#include <functional>
namespace vsmc { namespace internal {
using std::function;
} }

#else // VSMC_USE_STD_FUNCTION

#include <boost/function.hpp>
namespace vsmc { namespace internal {
using boost::function;
} }

#endif // VSMC_USE_STD_FUNCTION

#endif // VSMC_INTERNAL_FUNCTION_HPP
