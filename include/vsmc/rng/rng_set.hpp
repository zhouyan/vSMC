#ifndef VSMC_RNG_RNG_SET_HPP
#define VSMC_RNG_RNG_SET_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/seed.hpp>

#if VSMC_HAS_CXX11LIB_MUTEX
#include <mutex>
#endif

#if VSMC_USE_RANDOM123
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#endif // _MSC_VER
#include <Random123/threefry.h>
#include <Random123/philox.h>
#include <Random123/conventional/Engine.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER
#endif // VSMC_USE_RANDOM123

namespace vsmc {

struct ScalarRng;
struct VectorRng;
template <typename, typename> class RngSet;

/// \brief Scalar RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, ScalarRng>
{
    public :

    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSet (size_type N) : size_(N), rng_(Seed::instance().get()) {}

    size_type size () const {return size_;}

    rng_type &rng (size_type) {return rng_;}

    rng_type &operator[] (size_type) {return rng_;}

    private :

    std::size_t size_;
    rng_type rng_;
}; // class RngSet<RngType, ScalarRng>

/// \brief Vector RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, VectorRng>
{
    public :

    typedef RngType rng_type;
    typedef typename std::vector<rng_type>::size_type size_type;

    explicit RngSet (size_type N) : rng_(N)
    {
        for (size_type i = 0; i != N; ++i)
            rng_[i].seed(Seed::instance().get());
    }

    size_type size () const {return rng_.size();}

    rng_type &rng (size_type id) {return rng_[id];}

    rng_type &operator[] (size_type id) {return rng_[id];}

    private :

    std::vector<rng_type> rng_;
}; // class RngSet<RngType, VectorRng>

#if VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX

struct ThreadLocalRng;

/// \brief Thread local RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, ThreadLocalRng>
{
    public :

    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSet (size_type N) : size_(N) {}

    size_type size () const {return size_;}

    rng_type &rng (size_type)
    {
        static thread_local std::pair<rng_type, bool> rng_flag =
            std::make_pair(rng_type(), false);

        if (rng_flag.second) return rng_flag.first;

        std::lock_guard<std::mutex> lock(mtx_);
        rng_flag.first.seed(Seed::instance().get());
        shift_(rng_flag.first);
        rng_flag.second = true;

        return rng_flag.first;
    }

    rng_type &operator[] (size_type id) {return rng(id);}

    private :

    std::size_t size_;
    traits::RngShift<rng_type> shift_;
    std::mutex mtx_;
}; // class RngSet<RngType, ThreadLocalRng>

#endif // VSMC_HAS_CXX11_THREAD_LOCAL && VSMC_HAS_CXX11LIB_MUTEX

namespace traits {

/// \brief Particle::rng_set_type trait
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type,
        VSMC_DEFAULT_RNG_SET_TYPE)

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_HPP
