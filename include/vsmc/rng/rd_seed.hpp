#ifndef VSMC_RD_RSEED_HPP
#define VSMC_RD_RSEED_HPP

#include <vsmc/rng/generator_wrapper.hpp>
#include <vsmc/rng/engine_result_wrapper.hpp>
#include <immintrin.h>

#ifndef VSMC_RD_SEED_NTRIAL_MAX
#define VSMC_RD_SEED_NTRIAL_MAX 10
#endif

#define VSMC_RUNTIME_WARNING_RNG_RD_RAND_NTRIAL(ntrial, NTrialMax) \
    VSMC_RUNTIME_WARNING((ntrial < NTrialMax),                                \
            ("**RdRandStep::generate** MAXIMUM NUMBER OF TRIALS EXCEEDED"))

namespace vsmc {

/// \brief RdSeed generator
/// \ingroup RDRNG
template <typename, std::size_t = VSMC_RD_SEED_NTRIAL_MAX> struct RdSeedStep;

/// \brief RdSeed generator with 16-bits output
/// \ingroup RDRNG
template <std::size_t NTrialMax>
struct RdSeedStep<uint16_t, NTrialMax>
{
    static uint16_t generate ()
    {
        unsigned short r = 0;
        std::size_t ntrial = 0;
        while (_rdseed16_step(&r) == 0 && ntrial <= NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RD_SEED_NTRIAL(ntrial, NTrialMax);

        return static_cast<uint16_t>(r);
    }
}; // struct RdSeedStep

/// \brief RdSeed generator with 32-bits output
/// \ingroup RDRNG
template <std::size_t NTrialMax>
struct RdSeedStep<uint32_t, NTrialMax>
{
    static uint32_t generate ()
    {
        unsigned r = 0;
        std::size_t ntrial = 0;
        while (_rdseed32_step(&r) == 0 && ntrial <= NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RD_SEED_NTRIAL(ntrial, NTrialMax);

        return static_cast<uint32_t>(r);
    }
}; // struct RdSeedStep

/// \brief RdSeed generator with 64-bits output
/// \ingroup RDRNG
template <std::size_t NTrialMax>
struct RdSeedStep<uint64_t, NTrialMax>
{
    static uint64_t generate ()
    {
        unsigned VSMC_INT64 r = 0;
        std::size_t ntrial = 0;
        while (_rdseed64_step(&r) == 0 && ntrial <= NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RD_SEED_NTRIAL(ntrial, NTrialMax);

        return static_cast<uint64_t>(r);
    }
}; // struct RdSeedStep

/// \brief C++11 Engine that wraps _rdseed16_step
/// \ingroup RDRNG
typedef RdSeedEngine<uint16_t, RdSeedStep<uint16_t> > RdSeed16;

/// \brief C++11 Engine that wraps _rdseed32_step
/// \ingroup RDRNG
typedef RdSeedEngine<uint32_t, RdSeedStep<uint32_t> > RdSeed32;

/// \brief C++11 Engine that wraps _rdseed64_step
/// \ingroup RDRNG
typedef RdSeedEngine<uint64_t, RdSeedStep<uint64_t> > RdSeed64;

/// \brief C++11 Engine that wraps _rdseed32_step but output 16-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint16_t, RdSeed32> RdSeed32_16;

/// \brief C++11 Engine that wraps _rdseed64_step but output 16-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint16_t, RdSeed64> RdSeed64_16;

/// \brief C++11 Engine that wraps _rdseed64_step but output 32-bits integers
/// \ingroup RDRNG
typedef EngineResultWrapper<uint32_t, RdSeed64> RdSeed64_32;

} // namespace vsmc

#endif // VSMC_RD_RSEED_HPP
