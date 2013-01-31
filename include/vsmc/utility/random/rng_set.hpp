#ifndef VSMC_UTILITY_RANDOM_RNG_SET_HPP
#define VSMC_UTILITY_RANDOM_RNG_SET_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/utility/random/seed.hpp>

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

/// \brief Parallel RNG set class
/// \ingroup Random
class RngSet
{
    public :

    typedef VSMC_ENGINE_TYPE rng_type;
    typedef VSMC_SEED_TYPE seed_type;
    typedef std::vector<rng_type>::size_type size_type;

    explicit RngSet (size_type N)
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

    private :

    std::vector<rng_type> rng_;
}; // class RngSet

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type, RngSet)
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(ResampleRngType, resample_rng_type,
        cxx11::mt19937)

#endif // VSMC_UTILITY_RANDOM_RNG_SET_HPP
