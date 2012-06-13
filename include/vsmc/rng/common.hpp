#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <cassert>
#include <cmath>
#include <limits>

#define VSMC_PREVENT_MIN_MAX

#ifndef VSMC_RNG_SEED
#define VSMC_RNG_SEED 0xdeadbeefU
#endif // VSMC_CBRNG_SEED

#ifndef VSMC_CBRNG_TYPE
#define VSMC_CBRNG_TYPE r123::Threefry4x64
#endif // VSMC_CBRNG_TYPE

#ifndef VSMC_SEQRNG_TYPE
#define VSMC_SEQRNG_TYPE rng::mt19937
#endif

#endif // VSMC_RNG_COMMON_HPP
