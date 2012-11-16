#ifndef VSMC_CORE_RNG_HPP
#define VSMC_CORE_RNG_HPP

#include <vsmc/internal/common.hpp>

#if VSMC_USE_RANDOM123
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#endif // _MSC_VER
#define R123_USE_U01_DOUBLE 1
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/u01.h>
#include <Random123/MicroURNG.hpp>
#include <Random123/conventional/Engine.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER
#endif // VSMC_USE_RANDOM123

namespace vsmc {
/// \brief Sequential RNG set class
/// \ingroup Core
class RngSetSeq
{
    public :

    typedef VSMC_SEQRNG_TYPE rng_type;
    typedef VSMC_SEED_TYPE seed_type;
    typedef VSMC_SIZE_TYPE size_type;

    explicit RngSetSeq (size_type N) :
        rng_(static_cast<rng_type::result_type>(seed_type::instance().get()))
    {}

    rng_type &rng (size_type id)
    {
        return rng_;
    }

    protected :

    template <typename T> void set_value_ptr (T *) {}

    private :

    rng_type rng_;
}; // class RngSetSeq

/// \brief Parallel RNG set class
/// \ingroup Core
class RngSetPrl
{
    public :

    typedef VSMC_PRLRNG_TYPE rng_type;
    typedef VSMC_SEED_TYPE seed_type;
    typedef std::vector<rng_type>::size_type size_type;

    explicit RngSetPrl (size_type N)
    {
        seed_type &seed = seed_type::instance();
        for (size_type i = 0; i != N; ++i)
            rng_.push_back(rng_type(static_cast<rng_type::result_type>(
                            seed.get())));
    }

    rng_type &rng (size_type id)
    {
        return rng_[id];
    }

    protected :

    template <typename T> void set_value_ptr (T *) {}

    private :

    std::vector<rng_type> rng_;
}; // class RngSetPrl

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type, RngSetPrl);
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(
        ResampleRngSetType, resample_rng_set_type, RngSetSeq);

#endif // VSMC_CORE_RNG_HPP
