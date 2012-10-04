#ifndef VSMC_CXX11_FUNCTIONAL_HPP
#define VSMC_CXX11_FUNCTIONAL_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_FUNCTIONAL

#include <functional>
namespace vsmc { namespace cxx11 {
using std::function;
using std::ref;
using std::cref;
} }

#else // VSMC_HAS_CXX11LIB_FUNCTIONAL

#include <boost/function.hpp>
namespace vsmc { namespace cxx11 {
using boost::function;
} }

#include <boost/ref.hpp>
namespace vsmc { namespace cxx11 {
using boost::ref;
using boost::cref;
} }

#endif // VSMC_HAS_CXX11LIB_FUNCTIONAL

#endif // VSMC_CXX11_FUNCTIONAL_HPP
