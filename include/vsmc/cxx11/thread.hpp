#ifndef VSMC_CXX11_THREAD_HPP
#define VSMC_CXX11_THREAD_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_THREAD

#include <thread>
namespace vsmc { namespace cxx11 {
using std::mutex;
using std::lock_guard;
} }

#elif VSMC_HAS_LIB_THREAD // VSMC_HAS_CXX11LIB_THREAD

#include <boost/thread.hpp>
namespace vsmc { namespace cxx11 {
using boost::mutex;
using boost::lock_guard;
} }

#endif // VSMC_HAS_CXX11LIB_THREAD

namespace vsmc {

class NullMutex {};

template <typename Mutex>
class NullLockGuard
{
    public :

    NullLockGuard (Mutex &) {}
}; // class NullLockGuard

} // namespace vsmc

#endif // VSMC_CXX11_THREAD_HPP
