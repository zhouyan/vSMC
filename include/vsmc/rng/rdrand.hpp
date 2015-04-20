//============================================================================
// vSMC/include/vsmc/rng/rdrand.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_RNG_RDRAND_HPP
#define VSMC_RNG_RDRAND_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/generator_wrapper.hpp>
#include <immintrin.h>

#ifndef VSMC_RDRAND_NTRIAL_MAX
#define VSMC_RDRAND_NTRIAL_MAX 10
#endif

#define VSMC_STATIC_ASSERT_RNG_RDRAND_GENERATOR_RESULT_TYPE(ResultType)      \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, uint16_t>::value ||         \
                           std::is_same<ResultType, uint32_t>::value ||      \
                           std::is_same<ResultType, uint64_t>::value),       \
        USE_RDRANDGenerator_WITH_RESULT_TYPE_OTHER_THAN_uint16_t_OR_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_RDRAND_GENERATOR                              \
    VSMC_STATIC_ASSERT_RNG_RDRAND_GENERATOR_RESULT_TYPE(ResultType);

#define VSMC_RUNTIME_WARNING_RNG_RDRAND_GENERATOR_NTRIAL(ntrial, NTrialMax)  \
    VSMC_RUNTIME_WARNING((ntrial < NTrialMax),                               \
        ("**RDRAND::generate** MAXIMUM NUMBER OF TRIALS EXCEEDED"))

namespace vsmc
{

/// \brief Invoke the RDRAND instruction and return the carry flag
/// \ingroup RDRNG
template <typename UIntType> inline bool rdrand(UIntType *);

/// \brief Invoke the 16-bits RDRAND instruction and return the carry flag
/// \ingroup RDRNG
template <> inline bool rdrand<uint16_t>(uint16_t *rand)
{
    unsigned short r;
    int cf = _rdrand16_step(&r);
    *rand = static_cast<uint16_t>(r);

    return cf != 0;
}

/// \brief Invoke the 32-bits RDRAND instruction and return the carry flag
/// \ingroup RDRNG
template <> inline bool rdrand<uint32_t>(uint32_t *rand)
{
    unsigned r;
    int cf = _rdrand32_step(&r);
    *rand = static_cast<uint32_t>(r);

    return cf != 0;
}

/// \brief Invoke the 64-bits RDRAND instruction and return the carry flag
/// \ingroup RDRNG
template <> inline bool rdrand<uint64_t>(uint64_t *rand)
{
    unsigned VSMC_INT64 r;
    int cf = _rdrand64_step(&r);
    *rand = static_cast<uint64_t>(r);

    return cf != 0;
}

/// \brief RDRAND generator
/// \ingroup RDRNG
template <typename ResultType, std::size_t NTrialMax = VSMC_RDRAND_NTRIAL_MAX>
class RDRANDGenerator
{
    public:
    typedef ResultType result_type;

    static result_type generate()
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
typedef GeneratorWrapper<uint16_t, RDRANDGenerator<uint16_t>> RDRAND16;

/// \brief C++11 Engine using 32-bits RDRAND instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint32_t, RDRANDGenerator<uint32_t>> RDRAND32;

/// \brief C++11 Engine using 64-bits RDRAND instruction
/// \ingroup RDRNG
typedef GeneratorWrapper<uint64_t, RDRANDGenerator<uint64_t>> RDRAND64;

} // namespace vsmc

#endif //  VSMC_RNG_RDRAND_HPP
