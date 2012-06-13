#ifndef VSMC_INTERNAL_FUNCTION_HPP
#define VSMC_INTERNAL_FUNCTION_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_FUNCTION

#include <functional>
namespace vsmc { namespace internal {
using std::function;
} }

#else // VSMC_HAS_CXX11LIB_FUNCTION

#include <boost/function.hpp>
namespace vsmc { namespace internal {
using boost::function;
} }

#endif // VSMC_HAS_CXX11LIB_FUNCTION

#endif // VSMC_INTERNAL_FUNCTION_HPP
