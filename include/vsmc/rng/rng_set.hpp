#ifndef VSMC_UTILITY_RNG_SET_HPP
#define VSMC_UTILITY_RNG_SET_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/seed.hpp>

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

#if VSMC_USE_RANDOM123
#define VSMC_DEFAULT_RNG_SET_TYPE \
    RngSet<r123::Engine<r123::Philox2x64>, VectorRng>
#else
#warning No Random123 found, RNG streams may not be independent
#define VSMC_DEFAULT_RNG_SET_TYPE \
    RngSet<vsmc::cxx11::mt19937, VectorRng>
#endif

namespace vsmc {

namespace traits {

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type,
        VSMC_DEFAULT_RNG_SET_TYPE)

} // namepsace vsmc::traits

/// \brief Scalar RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, ScalarRng>
{
    public :

    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSet (size_type N = 1) : size_(N), rng_(static_cast<
            typename rng_type::result_type>(Seed::instance().get())) {}

    size_type size () const {return size_;}

    rng_type &rng (size_type id = 0) {return rng_;}

    private :

    std::size_t size_;
    rng_type rng_;
}; // class RngSet

/// \brief Vector RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, VectorRng>
{
    public :

    typedef RngType rng_type;
    typedef typename std::vector<rng_type>::size_type size_type;

    explicit RngSet (size_type N = 1)
    {
        Seed &seed = Seed::instance();
        for (size_type i = 0; i != N; ++i) {
            rng_.push_back(rng_type(static_cast<
                        typename rng_type::result_type>(seed.get())));
        }
    }

    size_type size () const {return rng_.size();}

    rng_type &rng (size_type id = 0) {return rng_[id];}

    private :

    std::vector<rng_type> rng_;
}; // class RngSet

} // namespace vsmc

#endif // VSMC_UTILITY_RNG_SET_HPP
