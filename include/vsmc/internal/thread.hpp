#ifndef VSMC_INTERNAL_THREAD_HPP
#define VSMC_INTERNAL_THREAD_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_THREAD

#include <thread>
namespace vsmc { namespace internal {
using std::mutex;
using std::lock_guard;
} }

#elif VSMC_HAS_LIB_THREAD // VSMC_HAS_CXX11LIB_THREAD

#include <boost/thread.hpp>
namespace vsmc { namespace internal {
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

#endif // VSMC_INTERNAL_THREAD_HPP
