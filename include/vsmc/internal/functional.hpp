#ifndef VSMC_INTERNAL_FUNCTIONAL_HPP
#define VSMC_INTERNAL_FUNCTIONAL_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_FUNCTIONAL

#include <functional>
namespace vsmc { namespace internal {
using std::function;
} }

#else // VSMC_HAS_CXX11LIB_FUNCTIONAL

#include <boost/function.hpp>
namespace vsmc { namespace internal {
using boost::function;
} }

#endif // VSMC_HAS_CXX11LIB_FUNCTIONAL

#endif // VSMC_INTERNAL_FUNCTIONAL_HPP
