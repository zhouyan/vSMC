#ifndef VSMC_CXX11_CHRONO_HPP
#define VSMC_CXX11_CHRONO_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_CHRONO
#include <chrono>
namespace vsmc { namespace cxx11 { namespace chrono {
using std::chrono::duration;
using std::chrono::time_point;
using std::chrono::treat_as_floating_point;
using std::chrono::duration_values;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::minutes;
using std::chrono::hours;
using std::chrono::time_point_cast;
using std::chrono::system_clock;
using std::chrono::steady_clock;
using std::chrono::high_resolution_clock;
} } }
#else
#include <boost/chrono.hpp>
namespace vsmc { namespace cxx11 { namespace chrono {
using boost::chrono::duration;
using boost::chrono::time_point;
using boost::chrono::treat_as_floating_point;
using boost::chrono::duration_values;
using boost::chrono::duration_cast;
using boost::chrono::nanoseconds;
using boost::chrono::microseconds;
using boost::chrono::milliseconds;
using boost::chrono::seconds;
using boost::chrono::minutes;
using boost::chrono::hours;
using boost::chrono::time_point_cast;
using boost::chrono::system_clock;
using boost::chrono::steady_clock;
using boost::chrono::high_resolution_clock;
} } }
#endif // VSMC_HAS_CXX11LIB_CHRONO

#endif // VSMC_CXX11_CHRONO_HPP
