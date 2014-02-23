#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <iostream>
#include <stdint.h>

namespace vsmc {

namespace internal {

template <typename ResultType>
inline void rng_array_shift (ResultType *state, Position<2>)
{
    state[0] = state[1];
}

template <typename ResultType, std::size_t R>
inline void rng_array_shift (ResultType *state, Position<R>)
{
    state[0] = state[1];
    rng_array_shift(state + 1, Position<R - 1>());
}

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_RNG_COMMON_HPP
