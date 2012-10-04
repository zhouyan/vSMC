#ifndef VSMC_CXX11_THREAD_HPP
#define VSMC_CXX11_THREAD_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_THREAD

#include <thread>
namespace vsmc { namespace cxx11 {
using std::thread;
} }

#else // VSMC_HAS_CXX11LIB_THREAD

#include <boost/thread.hpp>
namespace vsmc { namespace cxx11 {
using boost::thread;
} }

#endif // VSMC_HAS_CXX11LIB_THREAD

#endif // VSMC_CXX11_THREAD_HPP
