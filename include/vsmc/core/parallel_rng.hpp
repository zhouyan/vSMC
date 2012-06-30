#ifndef VSMC_CORE_PARALLEL_RNG_HPP
#define VSMC_CORE_PARALLEL_RNG_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/random.hpp>

namespace vsmc {

/// \brief Parallel RNG set class
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
template <typename T>
class ParallelRNG
{
    protected :

    /// The type of the Counter-based random number generator C++11 engine
    typedef VSMC_RNG_TYPE rng_type;

    typedef typename SizeTypeTrait<T>::type size_type;

    ParallelRNG (size_type N) : rng_(N)
    {
        rng::Seed<rng_type> &seed = rng::Seed<rng_type>::create();
#if VSMC_USE_RANDOM123
        for (size_type i = 0; i != N; ++i) {
            rng_[i] = rng_type(
                    static_cast<rng_type::result_type>(seed.get()));
        }
#else
        rng_ = rng_type(static_cast<rng_type::result_type>(seed.get()));
#endif
    }

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
#if VSMC_USE_RANDOM123
        return rng_[id];
#else
        return rng_;
#endif
    }

    private :

#if VSMC_USE_RANDOM123
    std::vector<rng_type> rng_;
#else
    rng_type rng_;
#endif
}; // class Parallel RNG

} // namespace vsmc

#endif // VSMC_CORE_PARALLEL_RNG_HPP
