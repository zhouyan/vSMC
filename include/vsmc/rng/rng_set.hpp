#ifndef VSMC_RNG_RNG_SET_HPP
#define VSMC_RNG_RNG_SET_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/seed.hpp>

#if VSMC_USE_RANDOM123
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#endif // _MSC_VER
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER
#else //  VSMC_USE_RANDOM123
#include <vsmc/rng/philox.hpp>
#include <vsmc/rng/threefry.hpp>
#endif // VSMC_USE_RANDOM123

namespace vsmc {

template <typename, typename> class RngSet;

/// \brief Scalar RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, Scalar>
{
    public :

    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSet (size_type N) : size_(N), rng_(Seed::instance().get()) {}

    size_type size () const {return size_;}

    rng_type &operator[] (size_type) {return rng_;}

    private :

    std::size_t size_;
    rng_type rng_;
}; // class RngSet

/// \brief Vector RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, Vector>
{
    public :

    typedef RngType rng_type;
    typedef typename std::vector<rng_type>::size_type size_type;

    explicit RngSet (size_type N) : rng_(N)
    {
        for (size_type i = 0; i != N; ++i)
            rng_[i].seed(static_cast<typename rng_type::result_type>(
                        Seed::instance().get()));
    }

    size_type size () const {return rng_.size();}

    rng_type &operator[] (size_type id) {return rng_[id];}

    private :

    std::vector<rng_type> rng_;
}; // class RngSet

namespace traits {

/// \brief Particle::rng_set_type trait
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type,
        VSMC_DEFAULT_RNG_SET_TYPE)

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_HPP
