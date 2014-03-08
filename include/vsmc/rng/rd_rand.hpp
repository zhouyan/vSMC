#ifndef VSMC_RNG_RD_RAND_HPP
#define VSMC_RNG_RD_RAND_HPP

#include <vsmc/rng/generator_wrapper.hpp>
#include <vsmc/rng/engine_result_wrapper.hpp>
#include <immintrin.h>

#ifndef VSMC_RD_RAND_NTRIAL_MAX
#define VSMC_RD_RAND_NTRIAL_MAX 10
#endif

#define VSMC_RUNTIME_WARNING_RNG_RD_RAND_NTRIAL(ntrial, NTrialMax) \
    VSMC_RUNTIME_WARNING((ntrial < NTrialMax),                                \
            ("**RdSeedStep::generate** MAXIMUM NUMBER OF TRIALS EXCEEDED"))

namespace vsmc {

/// \brief RdRand generator
/// \ingroup RDRNG
template <typename, std::size_t = VSMC_RD_RAND_NTRIAL_MAX> struct RdRandStep;

/// \brief RdRand generator with 16-bits output
/// \ingroup RDRNG
template <std::size_t NTrialMax> struct RdRandStep<uint16_t, NTrialMax>
{
    static uint16_t generate ()
    {
        unsigned short r = 0;
        std::size_t ntrial = 0;
        while (_rdrand16_step(&r) == 0 && ntrial != NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RD_RAND_NTRIAL(ntrial, NTrialMax);

        return static_cast<uint16_t>(r);
    }
}; // struct RdRandStep

/// \brief RdRand generator with 32-bits output
/// \ingroup RDRNG
template <std::size_t NTrialMax> struct RdRandStep<uint32_t, NTrialMax>
{
    static uint32_t generate ()
    {
        unsigned r = 0;
        std::size_t ntrial = 0;
        while (_rdrand32_step(&r) == 0 && ntrial != NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RD_RAND_NTRIAL(ntrial, NTrialMax);

        return static_cast<uint32_t>(r);
    }
}; // struct RdRandStep

/// \brief RdRand generator with 64-bits output
/// \ingroup RDRNG
template <std::size_t NTrialMax> struct RdRandStep<uint64_t, NTrialMax>
{
    static uint64_t generate ()
    {
        unsigned long long r = 0;
        std::size_t ntrial = 0;
        while (_rdrand64_step(&r) == 0 && ntrial != NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RD_RAND_NTRIAL(ntrial, NTrialMax);

        return static_cast<uint64_t>(r);
    }
}; // struct RdRandStep

/// \brief C++11 Engine that wraps _rdrand16_step
/// \ingroup RDRNG
typedef GeneratorWrapper<uint16_t, RdRandStep<uint16_t> > RdRand16;

/// \brief C++11 Engine that wraps _rdrand32_step
/// \ingroup RDRNG
typedef GeneratorWrapper<uint32_t, RdRandStep<uint32_t> > RdRand32;

/// \brief C++11 Engine that wraps _rdrand64_step
/// \ingroup RDRNG
typedef GeneratorWrapper<uint64_t, RdRandStep<uint64_t> > RdRand64;

/// \brief C++11 Engine that wraps _rdrand32_step but output 16-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint16_t, RdRand32> RdRand32_16;

/// \brief C++11 Engine that wraps _rdrand64_step but output 16-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint16_t, RdRand64> RdRand64_16;

/// \brief C++11 Engine that wraps _rdrand64_step but output 32-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint32_t, RdRand64> RdRand64_32;

} // namespace vsmc

#endif//  VSMC_RNG_RD_RAND_HPP
