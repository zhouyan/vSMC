#ifndef VSMC_CXX11_MUTEX_HPP
#define VSMC_CXX11_MUTEX_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_MUTEX
#include <mutex>
namespace vsmc { namespace cxx11 {
using std::mutex;
using std::lock_guard;
#if VSMC_HAS_CXX11LIB_MUTEX_COMPLETE
using std::recursive_mutex;
using std::timed_mutex;
using std::recursive_timed_mutex;
using std::defer_lock_t;
using std::try_to_lock_t;
using std::adopt_lock_t;
using std::defer_lock;
using std::try_to_lock;
using std::adopt_lock;
using std::unique_lock;
using std::try_lock;
using std::lock;
using std::once_flag;
using std::call_once;
#endif // VSMC_HAS_CXX11LIB_MUTEX_COMPLETE
} }
#else // VSMC_HAS_CXX11LIB_MUTEX
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/once.hpp>
namespace vsmc { namespace cxx11 {
using boost::mutex;
using boost::lock_guard;
#if VSMC_HAS_CXX11LIB_MUTEX_COMPLETE
using boost::recursive_mutex;
using boost::timed_mutex;
using boost::recursive_timed_mutex;
using boost::defer_lock_t;
using boost::try_to_lock_t;
using boost::adopt_lock_t;
using boost::defer_lock;
using boost::try_to_lock;
using boost::adopt_lock;
using boost::unique_lock;
using boost::try_lock;
using boost::lock;
using boost::once_flag;
using boost::call_once;
#endif // VSMC_HAS_CXX11LIB_MUTEX_COMPLETE
} }
#endif // VSMC_HAS_CXX11LIB_MUTEX

#endif // VSMC_CXX11_MUTEX_HPP
