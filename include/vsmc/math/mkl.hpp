//============================================================================
// vSMC/include/vsmc/math/mkl.hpp
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

#ifndef VSMC_MATH_MKL_HPP
#define VSMC_MATH_MKL_HPP

#include <vsmc/rng/internal/common.hpp>
#include <mkl.h>

#define VSMC_STATIC_ASSERT_MATH_MKL_SS_TASK_RESULT_TYPE(ResultType)           \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value),          \
        "**MKLSSTask** USED WITH A ResultType OTHER THAN float OR double")

#define VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType)         \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value ||         \
                           std::is_same<ResultType, MKL_Complex8>::value ||   \
                           std::is_same<ResultType, MKL_Complex16>::value),   \
        "**MKLConvTask** USED WITH A ResultType OTHER THAN float, double, "   \
        "MKL_Complex8, OR MKL_Complex16")

#define VSMC_STATIC_ASSERT_MATH_MKL_CORR_TASK_RESULT_TYPE(ResultType)         \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value ||         \
                           std::is_same<ResultType, MKL_Complex8>::value ||   \
                           std::is_same<ResultType, MKL_Complex16>::value),   \
        "**MKLCorrTask** USED WITH A ResultType OTHER THAN float, double, "   \
        "MKL_Complex8, OR MKL_Complex16")

#define VSMC_STATIC_ASSERT_MATH_MKL_DF_TASK_RESULT_TYPE(ResultType)           \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value),          \
        "**MKLDFTask** USED WITH A ResultType OTHER THAN float OR double")

#define VSMC_RUNTIME_ASSERT_MATH_MKL_VSL_OFFSET(offset)                       \
    VSMC_RUNTIME_ASSERT((offset < max VSMC_MNE()),                            \
        "**MKLOffsetDynamic** "                                               \
        "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS")

#define VSMC_DEFINE_MATH_MKL_VSL_ERROR(STATUS)                                \
    if (status == STATUS)                                                     \
        return #STATUS;

namespace vsmc
{

namespace internal
{

inline std::string mkl_vsl_error_str(int status)
{
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_ERROR_FEATURE_NOT_IMPLEMENTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_ERROR_UNKNOWN);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_ERROR_BADARGS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_ERROR_MEM_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_ERROR_NULL_PTR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_ERROR_CPU_NOT_SUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_INVALID_BRNG_INDEX);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_SKIPAHEAD_UNSUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BRNGS_INCOMPATIBLE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_STREAM);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BRNG_TABLE_FULL);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_STREAM_STATE_SIZE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_WORD_SIZE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_NSEEDS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_NBITS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_QRNG_PERIOD_ELAPSED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_LEAPFROG_NSTREAMS_TOO_BIG);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BRNG_NOT_SUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_UPDATE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_NO_NUMBERS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_INVALID_ABSTRACT_STREAM);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_NONDETERM_NOT_SUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_NONDETERM_NRETRIES_EXCEEDED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_FILE_CLOSE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_FILE_OPEN);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_FILE_WRITE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_FILE_READ);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_FILE_FORMAT);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_UNSUPPORTED_FILE_VER);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_RNG_ERROR_BAD_MEM_FORMAT);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_NOT_IMPLEMENTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_ALLOCATION_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_BAD_DESCRIPTOR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_SERVICE_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_EDIT_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_EDIT_PROHIBITED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_COMMIT_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_COPY_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_DELETE_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_BAD_ARGUMENT);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_DIMS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_START);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_DECIMATION);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_XSHAPE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_YSHAPE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_ZSHAPE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_XSTRIDE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_YSTRIDE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_ZSTRIDE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_X);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_Y);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_Z);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_JOB);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_KIND);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_MODE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_TYPE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_PRECISION);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_EXTERNAL_PRECISION);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_INTERNAL_PRECISION);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_METHOD);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_CC_ERROR_OTHER);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_NOT_FULL_RANK_MATRIX);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_SEMIDEFINITE_COR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_ALLOCATION_FAILURE);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_DIMEN);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_OBSERV_N);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_STORAGE_NOT_SUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_INDC_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_WEIGHTS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MEAN_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_2R_MOM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_3R_MOM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_4R_MOM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_2C_MOM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_3C_MOM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_4C_MOM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_KURTOSIS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_SKEWNESS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MIN_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MAX_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_VARIATION_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_COV_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_COR_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_ACCUM_WEIGHT_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_QUANT_ORDER_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_QUANT_ORDER);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_QUANT_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_ORDER_STATS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_MOMORDER_NOT_SUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_ALL_OBSERVS_OUTLIERS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_ROBUST_COV_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_ROBUST_MEAN_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_METHOD_NOT_SUPPORTED);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_GROUP_INDC_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_NULL_TASK_DESCRIPTOR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_OBSERV_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_SINGULAR_COV);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_POOLED_COV_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_POOLED_MEAN_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_GROUP_COV_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_GROUP_MEAN_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_GROUP_INDC);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_OUTLIERS_PARAMS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_OUTLIERS_PARAMS_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_OUTLIERS_WEIGHTS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_ROBUST_COV_PARAMS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_ROBUST_COV_PARAMS_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_STORAGE_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_PARTIAL_COV_IDX_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_PARTIAL_COV_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_PARTIAL_COR_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_PARAMS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_PARAMS_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_BAD_PARAMS_N);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_PARAMS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_INIT_ESTIMATES_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_INIT_ESTIMATES_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_SIMUL_VALS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_SIMUL_VALS_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_ESTIMATES_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_ESTIMATES_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_SIMUL_VALS_N);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_ESTIMATES_N);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_OUTPUT_PARAMS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_PRIOR_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_PRIOR_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MI_MISSING_VALS_N);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(
        VSL_SS_ERROR_BAD_STREAM_QUANT_PARAMS_N_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_STREAM_QUANT_PARAMS_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_STREAM_QUANT_PARAMS_N);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_STREAM_QUANT_PARAMS);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_STREAM_QUANT_ORDER_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_STREAM_QUANT_ORDER);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_STREAM_QUANT_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_PARAMTR_COR_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_COR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_PARTIAL_COV_IDX);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_SUM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_2R_SUM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_3R_SUM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_4R_SUM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_2C_SUM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_3C_SUM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_4C_SUM_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_CP_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MDAD_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_BAD_MNAD_ADDR);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_ROBCOV_INTERN_C1);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_PARTIALCOV_INTERN_C1);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_PARTIALCOV_INTERN_C2);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_MISSINGVALS_INTERN_C1);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_MISSINGVALS_INTERN_C2);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_MISSINGVALS_INTERN_C3);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_MISSINGVALS_INTERN_C4);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_MISSINGVALS_INTERN_C5);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_PARAMTRCOR_INTERN_C1);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_COVRANK_INTERNAL_ERROR_C1);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_INVCOV_INTERNAL_ERROR_C1);
    VSMC_DEFINE_MATH_MKL_VSL_ERROR(VSL_SS_ERROR_INVCOV_INTERNAL_ERROR_C2);

    return "UNKNOWN";
}

#if VSMC_NO_RUNTIME_ASSERT
inline void mkl_vsl_error_check(int, const char *, const char *) {}
#else
inline void mkl_vsl_error_check(int status, const char *func, const char *mklf)
{
    if (status == VSL_ERROR_OK)
        return;

    std::string msg("**vsmc::");
    msg += func;
    msg += "** failure";
    msg += "; MKL function: ";
    msg += mklf;
    msg += "; Error code: ";
    msg += mkl_vsl_error_str(status);
    msg += " (" + itos(status) + ")";

    VSMC_RUNTIME_ASSERT((status == VSL_ERROR_OK), msg.c_str());
}
#endif

struct MKLOffsetZero {
    static constexpr MKL_INT min VSMC_MNE() { return 0; }
    static constexpr MKL_INT max VSMC_MNE() { return 0; }
    static void offset(MKL_INT) {}
    static constexpr MKL_INT offset() { return 0; }
}; // struct OffsetZero

template <MKL_INT MaxOffset>
struct MKLOffsetDynamic {
    MKLOffsetDynamic() : offset_(0) {}

    static constexpr MKL_INT min VSMC_MNE() { return 0; }
    static constexpr MKL_INT max VSMC_MNE() { return MaxOffset; }

    void offset(MKL_INT n)
    {
        VSMC_RUNTIME_ASSERT_MATH_MKL_VSL_OFFSET(n);
        offset_ = n;
    }

    MKL_INT offset() const { return offset_; }

    private:
    MKL_INT offset_;
}; // struct OffsetDynamic

template <MKL_INT>
struct MKLOffset {
    typedef MKLOffsetZero type;
};

template <>
struct MKLOffset<VSL_BRNG_MT2203> {
    typedef MKLOffsetDynamic<6024> type;
};

template <>
struct MKLOffset<VSL_BRNG_WH> {
    typedef MKLOffsetDynamic<273> type;
};

} // namespace vsmc::internal

namespace traits
{

/// \brief Default seed for MKL RNG
/// \ingroup Traits
template <MKL_INT>
struct MKLSeedTrait : public std::integral_constant<MKL_UINT, 1> {
};

/// \brief Default seed for MKL Sobol quasi-RNG
template <>
struct MKLSeedTrait<VSL_BRNG_SOBOL>
    : public std::integral_constant<MKL_UINT, 10> {
};

/// \brief Default seed for MKL Niederr quasi-RNG
template <>
struct MKLSeedTrait<VSL_BRNG_NIEDERR>
    : public std::integral_constant<MKL_UINT, 10> {
};

} // namespace traits

/// \brief MKL `VSLStreamStatePtr`
/// \ingroup MKL
template <MKL_INT BRNG>
class MKLStream : public internal::MKLOffset<BRNG>::type
{
    public:
    explicit MKLStream(
        MKL_UINT s = traits::MKLSeedTrait<BRNG>::value, MKL_INT offset = 0)
    {
        this->offset(offset);
        VSLStreamStatePtr ptr = nullptr;
        internal::mkl_vsl_error_check(
            ::vslNewStream(&ptr, BRNG + this->offset(), s),
            "MKLStream::MKLStream", "::vslNewStream");
        stream_ptr_.reset(ptr);
    }

    template <typename SeedSeq>
    explicit MKLStream(
        SeedSeq &seq, typename std::enable_if<internal::is_seed_seq<SeedSeq,
                          MKL_UINT, MKLStream<BRNG>>::value>::type * = nullptr)
    {
        MKL_UINT s = 0;
        seq.generate(&s, &s + 1);
        VSLStreamStatePtr ptr = nullptr;
        internal::mkl_vsl_error_check(
            ::vslNewStream(&ptr, BRNG + this->offset(), s),
            "MKLStream::MKLStream", "::vslNewStream");
        stream_ptr_.reset(ptr);
    }

    MKLStream(const MKLStream<BRNG> &other)
        : internal::MKLOffset<BRNG>::type(other)
    {
        VSLStreamStatePtr ptr = nullptr;
        internal::mkl_vsl_error_check(
            ::vslCopyStream(&ptr, other.stream_ptr_.get()),
            "MKLStream::MKLStream", "::vslCopyStream");
        stream_ptr_.reset(ptr);
    }

    MKLStream<BRNG> &operator=(const MKLStream<BRNG> &other)
    {
        if (this != &other) {
            internal::MKLOffset<BRNG>::type::operator=(other);
            internal::mkl_vsl_error_check(
                ::vslCopyStreamState(
                    stream_ptr_.get(), other.stream_ptr_.get()),
                "MKLStream::operator=", "::vslCopyStreamState");
        }

        return *this;
    }

    MKLStream(MKLStream<BRNG> &&) = default;
    MKLStream<BRNG> &operator=(MKLStream<BRNG> &&) = default;

    void seed(MKL_UINT s)
    {
        VSLStreamStatePtr ptr = nullptr;
        internal::mkl_vsl_error_check(
            ::vslNewStream(&ptr, BRNG + this->offset(), s), "MKLStream::seed",
            "::vslNewStream");
        stream_ptr_.reset(ptr);
    }

    template <typename SeedSeq>
    void seed(
        SeedSeq &seq, typename std::enable_if<internal::is_seed_seq<SeedSeq,
                          MKL_UINT, MKLStream<BRNG>>::value>::type * = nullptr)
    {
        MKL_UINT s = 0;
        seq.generate(&s, &s + 1);
        seed(s);
    }

    VSLStreamStatePtr ptr() const { return stream_ptr_.get(); }

    private:
    struct deleter {
        void operator()(VSLStreamStatePtr ptr)
        {
            internal::mkl_vsl_error_check(::vslDeleteStream(&ptr),
                "MKLStream::deleter", "::vslDeleteStream");
        }
    };

    typedef std::unique_ptr<std::remove_pointer<VSLStreamStatePtr>::type,
        deleter> stream_ptr_type;

    MKL_UINT seed_;
    stream_ptr_type stream_ptr_;
}; // class MKLStream

/// \brief MKL `VSLSSTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLSSTask
{
    public:
    typedef ResultType result_type;

    MKLSSTask(const MKL_INT *p, const MKL_INT *n, const MKL_INT *xstorage,
        const result_type *x, const result_type *w = nullptr,
        const MKL_INT *indices = nullptr)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_SS_TASK_RESULT_TYPE(ResultType);
        VSLSSTaskPtr ptr = nullptr;
        new_task(&ptr, p, n, xstorage, x, w, indices);
        task_ptr_.reset(ptr);
    }

    MKLSSTask(const MKLSSTask<ResultType> &) = delete;
    MKLSSTask *operator=(const MKLSSTask<ResultType> &) = delete;
    MKLSSTask(MKLSSTask<ResultType> &&) = default;
    MKLSSTask *operator=(MKLSSTask<ResultType> &&) = default;

    VSLSSTaskPtr ptr() const { return task_ptr_.get(); }

    private:
    struct deleter {
        void operator()(VSLSSTaskPtr ptr)
        {
            internal::mkl_vsl_error_check(::vslSSDeleteTask(&ptr),
                "MKLSSTask::deleter", "::vslSSDeleteTask");
        }
    };

    typedef std::unique_ptr<std::remove_pointer<VSLSSTaskPtr>::type, deleter>
        task_ptr_type;

    task_ptr_type task_ptr_;

    void new_task(VSLSSTaskPtr *task, const MKL_INT *p, const MKL_INT *n,
        const MKL_INT *xstorage, const float *x, const float *w,
        const MKL_INT *indices)
    {
        internal::mkl_vsl_error_check(
            ::vslsSSNewTask(task, p, n, xstorage, x, w, indices),
            "MKLSSTask::MKLSSTask", "::vslsSSNewTask");
    }

    void new_task(VSLSSTaskPtr *task, const MKL_INT *p, const MKL_INT *n,
        const MKL_INT *xstorage, const double *x, const double *w,
        const MKL_INT *indices)
    {
        internal::mkl_vsl_error_check(
            ::vsldSSNewTask(task, p, n, xstorage, x, w, indices),
            "MKLSSTask::MKLSSTask", "::vsldSSNewTask");
    }
}; // class MKLSSTask

/// \brief MKL `VSLConvTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLConvTask
{
    public:
    typedef ResultType result_type;

    /// \brief `vslConvNewTask`
    MKLConvTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        VSLConvTaskPtr ptr = nullptr;
        new_task(&ptr, mode, dims, xshape, yshape, zshape,
            static_cast<result_type *>(nullptr));
        task_ptr_.reset(ptr);
    }

    /// \brief `vslConvNewTask1D`
    MKLConvTask(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        VSLConvTaskPtr ptr = nullptr;
        new_task(&ptr, mode, xshape, yshape, zshape,
            static_cast<result_type *>(nullptr));
        task_ptr_.reset(ptr);
    }

    /// \brief `vslConvNewTaskX`
    MKLConvTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        VSLConvTaskPtr ptr = nullptr;
        new_task(&ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        task_ptr_.reset(ptr);
    }

    /// \brief `vslConvNewTaskX1D`
    MKLConvTask(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        VSLConvTaskPtr ptr = nullptr;
        new_task(&ptr, mode, xshape, yshape, zshape, x, xstride);
        task_ptr_.reset(ptr);
    }

    MKLConvTask(const MKLConvTask<ResultType> &other)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        VSLConvTaskPtr ptr = nullptr;
        internal::mkl_vsl_error_check(::vslConvCopyTask(&ptr, other.ptr()),
            "MKLConvTask::MKLConvTask", "::vslConvCopyTask");
        task_ptr_.reset(ptr);
    }

    MKLConvTask *operator=(const MKLConvTask<ResultType> &other)
    {
        if (this != &other) {
            VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType);
            VSLConvTaskPtr ptr = nullptr;
            internal::mkl_vsl_error_check(::vslConvCopyTask(&ptr, other.ptr()),
                "MKLConvTask::MKLConvTask", "::vslConvCopyTask");
            task_ptr_.reset(ptr);
        }

        return *this;
    }

    MKLConvTask(MKLSSTask<ResultType> &&) = default;
    MKLConvTask *operator=(MKLSSTask<ResultType> &&) = default;

    private:
    struct deleter {
        void operator()(VSLConvTaskPtr ptr)
        {
            internal::mkl_vsl_error_check(::vslConvDeleteTask(&ptr),
                "MKLConvTask::deleter", "::vslConvDeleteTask");
        }
    };

    typedef std::unique_ptr<std::remove_pointer<VSLConvTaskPtr>::type, deleter>
        task_ptr_type;

    task_ptr_type task_ptr_;

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        float *)
    {
        internal::mkl_vsl_error_check(
            ::vslsConvNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vslsConvNewTask");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        double *)
    {
        internal::mkl_vsl_error_check(
            ::vsldConvNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vsldConvNewTask");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        MKL_Complex8 *)
    {
        internal::mkl_vsl_error_check(
            ::vslcConvNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vslcConvNewTask");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        MKL_Complex16 *)
    {
        internal::mkl_vsl_error_check(
            ::vslzConvNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vslzConvNewTask");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        float *)
    {
        internal::mkl_vsl_error_check(
            ::vslsConvNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vslsConvNewTask1D");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        double *)
    {
        internal::mkl_vsl_error_check(
            ::vsldConvNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vsldConvNewTask1D");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex8 *)
    {
        internal::mkl_vsl_error_check(
            ::vslcConvNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vslcConvNewTask1D");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex16 *)
    {
        internal::mkl_vsl_error_check(
            ::vslzConvNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLConvTask::MKLConvTask", "::vslzConvNewTask1D");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const float *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vslsConvNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vslsConvNewTaskX");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const double *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vsldConvNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vsldConvNewTaskX");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const MKL_Complex8 *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vslcConvNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vslcConvNewTaskX");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const MKL_Complex16 *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vslzConvNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vslzConvNewTaskX");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const float *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vslsConvNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vslsConvNewTaskX1D");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const double *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vsldConvNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vsldConvNewTaskX1D");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex8 *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vslcConvNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vslcConvNewTaskX1D");
    }

    void new_task(VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex16 *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vslzConvNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLConvTask::MKLConvTask", "::vslzConvNewTaskX1D");
    }
}; // class MKLConvTask

/// \brief MKL `VSLCorrTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLCorrTask
{
    public:
    typedef ResultType result_type;

    /// \brief `vslCorrNewTask`
    MKLCorrTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        VSLCorrTaskPtr ptr = nullptr;
        new_task(&ptr, mode, dims, xshape, yshape, zshape,
            static_cast<result_type *>(nullptr));
        task_ptr_.reset(ptr);
    }

    /// \brief `vslCorrNewTask1D`
    MKLCorrTask(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        VSLCorrTaskPtr ptr = nullptr;
        new_task(&ptr, mode, xshape, yshape, zshape,
            static_cast<result_type *>(nullptr));
        task_ptr_.reset(ptr);
    }

    /// \brief `vslCorrNewTaskX`
    MKLCorrTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        VSLCorrTaskPtr ptr = nullptr;
        new_task(&ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        task_ptr_.reset(ptr);
    }

    /// \brief `vslCorrNewTaskX1D`
    MKLCorrTask(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        VSLCorrTaskPtr ptr = nullptr;
        new_task(&ptr, mode, xshape, yshape, zshape, x, xstride);
        task_ptr_.reset(ptr);
    }

    MKLCorrTask(const MKLCorrTask<ResultType> &other)
    {
        VSMC_STATIC_ASSERT_MATH_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        VSLCorrTaskPtr ptr = nullptr;
        internal::mkl_vsl_error_check(::vslCorrCopyTask(&ptr, other.ptr()),
            "MKLCorrTask::MKLCorrTask", "::vslCorrCopyTask");
        task_ptr_.reset(ptr);
    }

    MKLCorrTask *operator=(const MKLCorrTask<ResultType> &other)
    {
        if (this != &other) {
            VSMC_STATIC_ASSERT_MATH_MKL_CONV_TASK_RESULT_TYPE(ResultType);
            VSLCorrTaskPtr ptr = nullptr;
            internal::mkl_vsl_error_check(::vslCorrCopyTask(&ptr, other.ptr()),
                "MKLCorrTask::MKLCorrTask", "::vslCorrCopyTask");
            task_ptr_.reset(ptr);
        }

        return *this;
    }

    MKLCorrTask(MKLSSTask<ResultType> &&) = default;
    MKLCorrTask *operator=(MKLSSTask<ResultType> &&) = default;

    private:
    struct deleter {
        void operator()(VSLCorrTaskPtr ptr)
        {
            internal::mkl_vsl_error_check(::vslCorrDeleteTask(&ptr),
                "MKLCorrTask::deleter", "::vslCorrDeleteTask");
        }
    };

    typedef std::unique_ptr<std::remove_pointer<VSLCorrTaskPtr>::type, deleter>
        task_ptr_type;

    task_ptr_type task_ptr_;

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        float *)
    {
        internal::mkl_vsl_error_check(
            ::vslsCorrNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vslsCorrNewTask");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        double *)
    {
        internal::mkl_vsl_error_check(
            ::vsldCorrNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vsldCorrNewTask");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        MKL_Complex8 *)
    {
        internal::mkl_vsl_error_check(
            ::vslcCorrNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vslcCorrNewTask");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        MKL_Complex16 *)
    {
        internal::mkl_vsl_error_check(
            ::vslzCorrNewTask(task, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vslzCorrNewTask");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        float *)
    {
        internal::mkl_vsl_error_check(
            ::vslsCorrNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vslsCorrNewTask1D");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        double *)
    {
        internal::mkl_vsl_error_check(
            ::vsldCorrNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vsldCorrNewTask1D");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex8 *)
    {
        internal::mkl_vsl_error_check(
            ::vslcCorrNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vslcCorrNewTask1D");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex16 *)
    {
        internal::mkl_vsl_error_check(
            ::vslzCorrNewTask1D(task, mode, xshape, yshape, zshape),
            "MKLCorrTask::MKLCorrTask", "::vslzCorrNewTask1D");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const float *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vslsCorrNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vslsCorrNewTaskX");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const double *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vsldCorrNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vsldCorrNewTaskX");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const MKL_Complex8 *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vslcCorrNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vslcCorrNewTaskX");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode, MKL_INT dims,
        const MKL_INT *xshape, const MKL_INT *yshape, const MKL_INT *zshape,
        const MKL_Complex16 *x, const MKL_INT *xstride)
    {
        internal::mkl_vsl_error_check(::vslzCorrNewTaskX(task, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vslzCorrNewTaskX");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const float *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vslsCorrNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vslsCorrNewTaskX1D");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const double *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vsldCorrNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vsldCorrNewTaskX1D");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex8 *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vslcCorrNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vslcCorrNewTaskX1D");
    }

    void new_task(VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex16 *x, const MKL_INT xstride)
    {
        internal::mkl_vsl_error_check(::vslzCorrNewTaskX1D(task, mode, xshape,
                                          yshape, zshape, x, xstride),
            "MKLCorrTask::MKLCorrTask", "::vslzCorrNewTaskX1D");
    }
}; // class MKLCorrTask

} // namespace vsmc

#endif // VSMC_MATH_MKL_HPP
