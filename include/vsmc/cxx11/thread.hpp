#ifndef VSMC_CXX11_THREAD_HPP
#define VSMC_CXX11_THREAD_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_THREAD
#include <thread>
namespace vsmc { namespace cxx11 {
using std::thread;
namespace this_thread {
#if VSMC_HAS_CXX11LIB_THREAD_COMPLETE
using std::this_thread::get_id;
using std::this_thread::yield;
using std::this_thread::sleep_until;
using std::this_thread::sleep_for;
#endif // VSMC_HAS_CXX11LIB_THREAD_COMPLETE
} } }
#else // VSMC_HAS_CXX11LIB_THREAD
#include <boost/thread.hpp>
namespace vsmc { namespace cxx11 {
using boost::thread;
namespace this_thread {
#if VSMC_HAS_CXX11LIB_THREAD_COMPLETE
using boost::this_thread::get_id;
using boost::this_thread::yield;
using boost::this_thread::sleep_until;
using boost::this_thread::sleep_for;
#endif // VSMC_HAS_CXX11LIB_THREAD_COMPLETE
} } }
#endif // VSMC_HAS_CXX11LIB_THREAD

#endif // VSMC_CXX11_THREAD_HPP
