#ifndef VSMC_CORE_RNG_HPP
#define VSMC_CORE_RNG_HPP

#include <vsmc/internal/common.hpp>

#if VSMC_USE_RANDOM123
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#pragma warning(pop)
#endif
#endif

namespace vsmc {
/// \brief Sequential RNG set class
/// \ingroup Core
class RngSetSeq
{
    public :

    /// The type of the random number generator C++11 engine
    typedef VSMC_SEQRNG_TYPE rng_type;

    /// The type of the seed sequence
    typedef VSMC_SEED_TYPE seed_type;

    /// The type of the size of the rng set
    typedef VSMC_SIZE_TYPE size_type;

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
        return rng_;
    }

    protected :

    explicit RngSetSeq (size_type N) :
        rng_(static_cast<rng_type::result_type>(seed_type::instance().get()))
    {}

    private :

    rng_type rng_;
}; // class RngSetSeq

/// \brief Parallel RNG set class
/// \ingroup Core
class RngSetPrl
{
    public :

    /// The type of the random number generator C++11 engine
    typedef VSMC_PRLRNG_TYPE rng_type;

    /// The type of the seed sequence
    typedef VSMC_SEED_TYPE seed_type;

    /// The type of the size of the rng set
    typedef std::vector<rng_type>::size_type size_type;

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
        return rng_[id];
    }

    protected :

    explicit RngSetPrl (size_type N)
    {
        seed_type &seed = seed_type::instance();
        for (size_type i = 0; i != N; ++i)
            rng_.push_back(rng_type(static_cast<rng_type::result_type>(
                            seed.get())));
    }

    private :

    std::vector<rng_type> rng_;
}; // class RngSetPrl

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type, RngSetPrl);

#endif // VSMC_CORE_RNG_HPP
