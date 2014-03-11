#ifndef VSMC_RNG_RNG_SET_HPP
#define VSMC_RNG_RNG_SET_HPP

#include <vsmc/rng/seed.hpp>

#if VSMC_USE_RANDOM123
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#endif // _MSC_VER
#if VSMC_USE_AES_NI
#include <Random123/aes.h>
#include <Random123/ars.h>
#endif // VSMC_USE_AES_NI
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER
#else //  VSMC_USE_RANDOM123
#if VSMC_USE_AES_NI
#include <vsmc/rng/aes.h>
#include <vsmc/rng/ars.h>
#endif // VSMC_USE_AES_NI
#include <vsmc/rng/philox.hpp>
#include <vsmc/rng/threefry.hpp>
#endif // VSMC_USE_RANDOM123

/// \brief Default RNG set type
/// \ingroup Config
#ifndef VSMC_DEFAULT_RNG_SET_TYPE
#if VSMC_USE_RANDOM123
#if VSMC_USE_AES_NI
#define VSMC_DEFAULT_RNG_SET_TYPE \
    ::vsmc::RngSet< ::r123::Engine<r123::ARS4x32_R<7> >, ::vsmc::Vector>
#else
#define VSMC_DEFAULT_RNG_SET_TYPE \
    ::vsmc::RngSet< ::r123::Engine<r123::Philox2x64>, ::vsmc::Vector>
#endif // VSMC_USE_AES_NI
#else // VSMC_USE_RANDOM123
#if VSMC_USE_AES_NI
#define VSMC_DEFAULT_RNG_SET_TYPE \
    ::vsmc::RngSet< ::vsmc::ARSEngine<uint32_t, 1, 7>, ::vsmc::Vector>
#else // VSMC_USE_AES_NI
#define VSMC_DEFAULT_RNG_SET_TYPE \
    ::vsmc::RngSet< ::vsmc::Philox2x64, ::vsmc::Vector>
#endif // VSMC_USE_AES_NI
#endif // VSMC_USE_RANDOM123
#endif // VSMC_DEFAULT_RNG_SET_TYPE

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

    explicit RngSet (size_type N) : rng_(N, rng_type())
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
