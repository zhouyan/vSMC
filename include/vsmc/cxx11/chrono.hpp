#ifndef VSMC_CXX11_CHRONO_HPP
#define VSMC_CXX11_CHRONO_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_CHRONO
#include <chrono>
namespace vsmc { namespace cxx11 { namespace chrono {
using std::chrono::system_clock;
using std::chrono::duration;
} } }
#else
#include <boost/chrono.hpp>
namespace vsmc { namespace cxx11 { namespace chrono {
using boost::chrono::system_clock;
using boost::chrono::duration;
} } }
#endif // VSMC_HAS_CXX11LIB_CHRONO

#endif // VSMC_CXX11_CHRONO_HPP
