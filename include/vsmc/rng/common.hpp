#ifndef VSMC_RNG_COMMON_HPP
#define VSMC_RNG_COMMON_HPP

#include <cassert>
#include <cmath>
#include <limits>

#define VSMC_PREVENT_MIN_MAX

/// The RNG  seed, unsigned
#ifndef VSMC_RNG_SEED
#define VSMC_RNG_SEED 0xdeadbeefU
#endif // VSMC_CBRNG_SEED

/// The parallel RNG (based on Random123) type, philox or threefry
#ifndef VSMC_CBRNG_TYPE
#define VSMC_CBRNG_TYPE r123::Threefry4x64
#endif // VSMC_CBRNG_TYPE

/// The sequential RNG (Boost or C++11) type
#ifndef VSMC_SEQRNG_TYPE
#define VSMC_SEQRNG_TYPE rng::mt19937
#endif

#endif // VSMC_RNG_COMMON_HPP
