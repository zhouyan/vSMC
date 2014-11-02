//============================================================================
// include/vsmc/rng/rdrand.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_RDRAND_HPP
#define VSMC_RNG_RDRAND_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/generator_wrapper.hpp>

#ifdef _MSC_VER
#include <intrin.h>
#endif

#ifndef VSMC_RDRAND_NTRIAL_MAX
#define VSMC_RDRAND_NTRIAL_MAX 10
#endif

#define VSMC_STATIC_ASSERT_RNG_RDRAND_GENERATOR_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT((                                                     \
                cxx11::is_same<ResultType, uint16_t>::value ||               \
                cxx11::is_same<ResultType, uint32_t>::value ||               \
                cxx11::is_same<ResultType, uint64_t>::value),                \
            USE_RDRANDGenerator_WITH_RESULT_TYPE_OTHER_THAN_uint16_t_OR_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_RDRAND_GENERATOR \
    VSMC_STATIC_ASSERT_RNG_RDRAND_GENERATOR_RESULT_TYPE(ResultType);

#define VSMC_RUNTIME_WARNING_RNG_RDRAND_GENERATOR_NTRIAL(ntrial, NTrialMax) \
    VSMC_RUNTIME_WARNING((ntrial < NTrialMax),                                \
            ("**RDRAND::generate** MAXIMUM NUMBER OF TRIALS EXCEEDED"))

namespace vsmc {

/// \brief Invoke the RDRAND instruction and return the carry flag
/// \ingroup RDRNG
template <typename UIntType> inline bool rdrand (UIntType *);

#ifdef _MSC_VER

template <> inline bool rdrand<uint16_t> (uint16_t *rand)
{
    unsigned short r;
    int cf = _rdrand16_step(&r);
    *rand = static_cast<uint16_t>(r);

    return cf != 0;
}

template <> inline bool rdrand<uint32_t> (uint32_t *rand)
{
    unsigned r;
    int cf = _rdrand32_step(&r);
    *rand = static_cast<uint32_t>(r);

    return cf != 0;
}

template <> inline bool rdrand<uint64_t> (uint64_t *rand)
{
    unsigned __int64 r;
    int cf = _rdrand64_step(&r);
    *rand = static_cast<uint64_t>(r);

    return cf != 0;
}

#else // _MSC_VER

template <> inline bool rdrand<uint16_t> (uint16_t *rand)
{
    unsigned char cf = 0;
    __asm__ volatile
        (
         "rdrand %0\n\t"
         "setcb %1\n"
         : "=r" (*rand), "=qm" (cf)
        );

    return cf != 0;
}

template <> inline bool rdrand<uint32_t> (uint32_t *rand)
{
    unsigned char cf = 0;
    __asm__ volatile
        (
         "rdrand %0\n\t"
         "setcb %1\n"
         : "=r" (*rand), "=qm" (cf)
        );

    return cf != 0;
}

template <> inline bool rdrand<uint64_t> (uint64_t *rand)
{
    unsigned char cf = 0;
    __asm__ volatile
        (
         "rdrand %0\n\t"
         "setcb %1\n"
         : "=r" (*rand), "=qm" (cf)
        );

    return cf != 0;
}

#endif // _MSC_VER

/// \brief RDRAND generator
/// \ingroup RDRNG
template <typename ResultType, std::size_t NTrialMax = VSMC_RDRAND_NTRIAL_MAX>
class RDRANDGenerator
{
    public :

    typedef ResultType result_type;

    static result_type generate ()
    {
        VSMC_STATIC_ASSERT_RNG_RDRAND_GENERATOR;

        result_type r;
        std::size_t ntrial = 0;
        while (!rdrand<result_type>(&r) && ntrial != NTrialMax)
            ++ntrial;
        VSMC_RUNTIME_WARNING_RNG_RDRAND_GENERATOR_NTRIAL(ntrial, NTrialMax);

        return r;
    }
}; // class RDRANDGenertor

/// \brief C++11 Engine using 16-bits RDRAND instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint16_t, RDRANDGenerator<uint16_t> > RDRAND16;

/// \brief C++11 Engine using 32-bits RDRAND instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint32_t, RDRANDGenerator<uint32_t> > RDRAND32;

/// \brief C++11 Engine using 64-bits RDRAND instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint64_t, RDRANDGenerator<uint64_t> > RDRAND64;

} // namespace vsmc

#endif//  VSMC_RNG_RDRAND_HPP
