#ifndef V_SMC_INTERNAL_RANDOM_HPP
#define V_SMC_INTERNAL_RANDOM_HPP

#include <Random123/aes.h>
#include <Random123/ars.h>
#include <Random123/philox.h>
#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#include <vSMC/internal/config.hpp>

/// The Parallel RNG (based on Random123) seed, unsigned
#ifndef V_SMC_CRNG_SEED
#define V_SMC_CRNG_SEED 0xdeadbeefU
#endif // V_SMC_CRNG_SEED

/// The Parallel RNG (based on Random123) type, philox or threefry
#ifndef V_SMC_CRNG_TYPE
#define V_SMC_CRNG_TYPE r123::Threefry4x64
#endif // V_SMC_CRNG_TYPE

#ifdef V_SMC_USE_STD_RANDOM

#include <random>
namespace vSMC { namespace internal {
using std::binomial_distribution;
using std::uniform_real_distribution;
} }

#else // V_SMC_USE_STD_RANDOM

#include <boost/random.hpp>
#if BOOST_VERSION < 104700
namespace vSMC { namespace internal {
using boost::binomial_distribution;
using boost::uniform_real_distribution;
} }
#else // BOOST_VERSION < 104700
namespace vSMC { namespace internal {
using boost::random::binomial_distribution;
using boost::random::uniform_real_distribution;
} }
#endif // BOOST_VERSION < 104700

#endif // V_SMC_USE_STD_RANDOM

#endif // V_SMC_INTERNAL_RANDOM_HPP
