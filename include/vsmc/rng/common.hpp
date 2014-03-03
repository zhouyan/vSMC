#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <iostream>
#include <stdint.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4351)
#endif

namespace vsmc {

namespace internal {

template <typename ResultType>
inline void rng_array_shift (ResultType *state, Position<2>)
{
    state[0] = state[1];
}

template <typename ResultType, std::size_t K>
inline void rng_array_shift (ResultType *state, Position<K>)
{
    state[0] = state[1];
    rng_array_shift(state + 1, Position<K - 1>());
}

} // namespace vsmc::internal

} // namespace vsmc

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // VSMC_RNG_COMMON_HPP
