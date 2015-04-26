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

/// \brief MKL VSLStreamStatePtr
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
        int status = ::vslNewStream(&ptr, BRNG + this->offset(), s);
        internal::mkl_vsl_error_check(
            status, "MKLStream::Stream", "::vslNewStream");
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
        int status = ::vslNewStream(&ptr, BRNG + this->offset(), s);
        internal::mkl_vsl_error_check(
            status, "MKLStream::Stream", "::vslNewStream");
        stream_ptr_.reset(ptr);
    }

    MKLStream(const MKLStream<BRNG> &other)
        : internal::MKLOffset<BRNG>::type(other)
    {
        VSLStreamStatePtr ptr = nullptr;
        int status = ::vslCopyStream(&ptr, other.stream_ptr_.get());
        internal::mkl_vsl_error_check(
            status, "MKLStream::Stream", "::vslCopyStream");
        stream_ptr_.reset(ptr);
    }

    MKLStream<BRNG> &operator=(const MKLStream<BRNG> &other)
    {
        if (this != &other) {
            internal::MKLOffset<BRNG>::type::operator=(other);
            int status = ::vslCopyStreamState(
                stream_ptr_.get(), other.stream_ptr_.get());
            internal::mkl_vsl_error_check(
                status, "MKLStream::operator=", "::vslCopyStreamState");
        }

        return *this;
    }

    MKLStream(MKLStream<BRNG> &&other) = default;
    MKLStream<BRNG> &operator=(MKLStream<BRNG> &&other) = default;

    void seed(MKL_UINT s)
    {
        VSLStreamStatePtr ptr = nullptr;
        int status = ::vslNewStream(&ptr, BRNG + this->offset(), s);
        internal::mkl_vsl_error_check(
            status, "MKLStream::seed", "::vslNewStream");
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

    /// \brief Get stream state size
    int size() const { return ::vslGetStreamSize(ptr()); }

    /// \brief Save a stream to a file
    int save(const std::string &fname) const
    {
        int status = ::vslSaveStreamF(ptr(), fname.c_str());
        internal::mkl_vsl_error_check(
            status, "MKLStream::save", "::vslSaveStreamF");

        return status
    }

    /// \brief Save a stream to a memory buffer (query size using `size()`)
    int save(void *mem) const
    {
        int status = ::vslSaveStreamM(ptr(), static_cast<char *>(mem));
        internal::mkl_vsl_error_check(
            status, "MKLStream::save", "::vslSaveStreamM");

        return status;
    }

    /// \brief Load a stream from a file
    int load(const std::string &fname)
    {
        int status = ::vslLoadStreamF(ptr(), fname.c_str());
        internal::mkl_vsl_error_check(
            status, "MKLStream::load", "::vslLoadStreamF");

        return status;
    }

    /// \brief Load a stream from a memory buffer
    int load(const void *mem)
    {
        int status = ::vslLoadStreamM(ptr(), static_cast<const char *>(mem));
        intrnal::mkl_vsl_error_check(
            status, "MKLStream::load", "::vslLoadStreamM");

        return status;
    }

    /// \brief Leap frog the stream
    int leap_frog(MKL_INT k, MKL_INT nstream)
    {
        int status = ::vslLeapfrogStream(ptr(), k, nstream);
        internal::mkl_vsl_error_check(
            status, "MKLStream::leap_frog", "::vslLeapfrogStream");

        return status;
    }

    /// \brief Skip ahead the stream
    int skip_ahead(MKL_INT nskip)
    {
        int status = ::vslSkipAheadStream(ptr(), nskip);
        internal::mkl_vsl_error_check(
            status, "MKLStream::skip_ahead", "::vslSkipAheadStream");

        return status;
    }

    private:
    struct deleter {
        void operator()(VSLStreamStatePtr ptr)
        {
            int status = ::vslDeleteStream(&ptr);
            internal::mkl_vsl_error_check(
                status, "MKLStream::delete_stream", "::vslDeleteStream");
        }
    };

    typedef std::unique_ptr<std::remove_pointer<VSLStreamStatePtr>::type,
        deleter> stream_ptr_type;

    MKL_UINT seed_;
    stream_ptr_type stream_ptr_;
}; // class MKLStream

} // namespace vsmc

#endif // VSMC_MATH_MKL_HPP
