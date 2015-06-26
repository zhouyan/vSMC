//============================================================================
// vSMC/include/vsmc/utility/mkl.hpp
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

#ifndef VSMC_UTILITY_MKL_HPP
#define VSMC_UTILITY_MKL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/internal/common.hpp>
#include <mkl.h>

#define VSMC_STATIC_ASSERT_UTILITY_MKL_SS_TASK_RESULT_TYPE(ResultType)        \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value),          \
        "**MKLSSTask** USED WITH A ResultType OTHER THAN float OR double")

#define VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType)      \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value ||         \
                           std::is_same<ResultType, MKL_Complex8>::value ||   \
                           std::is_same<ResultType, MKL_Complex16>::value),   \
        "**MKLConvTask** USED WITH A ResultType OTHER THAN float, double, "   \
        "MKL_Complex8, OR MKL_Complex16")

#define VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType)      \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value ||         \
                           std::is_same<ResultType, MKL_Complex8>::value ||   \
                           std::is_same<ResultType, MKL_Complex16>::value),   \
        "**MKLCorrTask** USED WITH A ResultType OTHER THAN float, double, "   \
        "MKL_Complex8, OR MKL_Complex16")

#define VSMC_STATIC_ASSERT_UTILITY_MKL_DF_TASK_RESULT_TYPE(ResultType)        \
    VSMC_STATIC_ASSERT((std::is_same<ResultType, float>::value ||             \
                           std::is_same<ResultType, double>::value),          \
        "**MKLDFTask** USED WITH A ResultType OTHER THAN float OR double")

#define VSMC_RUNTIME_ASSERT_UTILITY_MKL_VSL_OFFSET(offset)                    \
    VSMC_RUNTIME_ASSERT((offset < max VSMC_MNE()),                            \
        "**MKLOffsetDynamic** "                                               \
        "EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS")

namespace vsmc
{

namespace internal
{

#if VSMC_NO_RUNTIME_ASSERT
inline void mkl_error_check(int, const char *, const char *) {}
#else
inline void mkl_error_check(int status, const char *func, const char *mklf)
{
    if (status == 0)
        return;

    std::string msg("**");
    msg += func;
    msg += "** failure";
    msg += "; MKL function: ";
    msg += mklf;
    msg += "; Error code: ";
    msg += itos(status);

    VSMC_RUNTIME_ASSERT((status == 0), msg.c_str());
}
#endif

} // namespace vsmc::internal

/// \brief MKL resource management base class
/// \ingroup MKL
template <typename MKLPtr, typename Derived>
class MKLBase
{
    public:
    using pointer = MKLPtr;
    using element_type = typename std::remove_pointer<MKLPtr>::type;

    class deleter_type
    {
        public:
        void operator()(MKLPtr ptr) { status_ = Derived::release(ptr); }

        int status() const { return status_; }

        private:
        int status_;
    }; // class deleter_type

    MKLBase() = default;
    MKLBase(const MKLBase<MKLPtr, Derived> &) = delete;
    MKLBase<MKLPtr, Derived> &operator=(
        const MKLBase<MKLPtr, Derived> &) = delete;
    MKLBase(MKLBase<MKLPtr, Derived> &&) = default;
    MKLBase<MKLPtr, Derived> &operator=(MKLBase<MKLPtr, Derived> &&) = default;

    int release() { return Derived::release(ptr_.get()); }

    void reset(pointer ptr = nullptr)
    {
        if (ptr != ptr_.get())
            ptr_.reset(ptr);
    }

    void swap(MKLBase<MKLPtr, Derived> &other) { ptr_.swap(other.ptr_); }

    pointer get() const { return ptr_.get(); }

    deleter_type &get_deleter() { return ptr_.get_deleter(); }
    const deleter_type &get_deleter() const { return ptr_.get_deleter(); }

    explicit operator bool() const { return bool(ptr_); }

    protected:
    void reset_ptr(pointer ptr = nullptr) { reset(ptr); }

    private:
    std::unique_ptr<element_type, deleter_type> ptr_;
}; // class MKLBase

/// \brief Comparison of equality of two MKLBase objects
/// \ingroup MKL
template <typename MKLPtr, typename Derived>
inline bool operator==(
    const MKLBase<MKLPtr, Derived> &ptr1, const MKLBase<MKLPtr, Derived> &ptr2)
{
    return ptr1.get() == ptr2.get();
}

/// \brief Comparison of inequality of two MKLBase objects
/// \ingroup MKL
template <typename MKLPtr, typename Derived>
inline bool operator!=(
    const MKLBase<MKLPtr, Derived> &ptr1, const MKLBase<MKLPtr, Derived> &ptr2)
{
    return ptr1.get() == ptr2.get();
}

/// \brief Swap two MKLBase objects
/// \ingroup MKL
template <typename MKLPtr, typename Derived>
inline void swap(
    const MKLBase<MKLPtr, Derived> &ptr1, const MKLBase<MKLPtr, Derived> &ptr2)
{
    ptr1.swap(ptr2);
}

/// \brief MKL `VSLStreamStatePtr`
/// \ingroup MKL
class MKLStream : public MKLBase<::VSLStreamStatePtr, MKLStream>
{
    public:
    MKLStream() = default;

    MKLStream(MKL_INT brng, MKL_UINT seed) { reset(brng, seed); }

    MKLStream(MKL_INT brng, MKL_INT n, unsigned *params)
    {
        reset(brng, n, params);
    }

    MKLStream(const MKLStream &other)
        : MKLBase<::VSLStreamStatePtr, MKLStream>()
    {
        ::VSLStreamStatePtr ptr = nullptr;
        internal::mkl_error_check(::vslCopyStream(&ptr, other.get()),
            "MKLStream::MKLStream", "::vslCopyStream");
        this->reset_ptr(ptr);
    }

    MKLStream &operator=(const MKLStream &other)
    {
        if (this != &other) {
            if (this->get() == nullptr) {
                ::VSLStreamStatePtr ptr = nullptr;
                internal::mkl_error_check(::vslCopyStream(&ptr, other.get()),
                    "MKLStream::operator=", "::vslCopyStream");
                this->reset_ptr(ptr);
            } else {
                internal::mkl_error_check(
                    ::vslCopyStreamState(this->get(), other.get()),
                    "MKLStream::operator=", "::vslCopyStreamState");
            }
        }

        return *this;
    }

    MKLStream(MKLStream &&) = default;
    MKLStream &operator=(MKLStream &&) = default;

    static int release(::VSLStreamStatePtr ptr)
    {
        if (ptr == nullptr)
            return 0;

        int status = ::vslDeleteStream(&ptr);
        internal::mkl_error_check(
            status, "MKLStream::release", "::vslDeleteStream");

        return status;
    }

    int reset(MKL_INT brng, MKL_UINT seed)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = ::vslNewStream(&ptr, brng, seed);
        internal::mkl_error_check(
            status, "MKLStream::reset", "::vslNewStream");
        this->reset_ptr(ptr);

        return status;
    }

    int reset(MKL_INT brng, MKL_INT n, unsigned *params)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = ::vslNewStreamEx(&ptr, brng, n, params);
        internal::mkl_error_check(
            status, "MKLStream::reset", "::vslNewStreamEx");
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslSaveStreamF`
    int save_f(const std::string &fname) const
    {
        int status = ::vslSaveStreamF(this->get(), fname.c_str());
        internal::mkl_error_check(
            status, "MKLStream::save_f", "::vslSaveStreamF");

        return status;
    }

    /// \brief `vslSaveStreamF`
    int load_f(const std::string &fname)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = ::vslSaveStreamF(&ptr, fname.c_str());
        internal::mkl_error_check(
            status, "MKLStream::load_f", "::vslSaveStreamF");
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslSaveStreamM`
    int save_m(char *memptr) const
    {
        int status = ::vslSaveStreamM(this->get(), memptr);
        internal::mkl_error_check(
            status, "MKLStream::save_m", "::vslSaveStreamM");

        return status;
    }

    /// \brief `vslLoadStreamM`
    int load_m(const char *memptr)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = ::vslLoadStreamM(&ptr, memptr);
        internal::mkl_error_check(
            status, "MKLStream::load_m", "::vslLoadStreamM");
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslGetStreamSize`
    int get_size() const { return ::vslGetStreamSize(this->get()); }

    /// \brief `vslLeapfrogStream`
    int leapfrog(MKL_INT k, MKL_INT nstreams) const
    {
        int status = ::vslLeapfrogStream(this->get(), k, nstreams);
        internal::mkl_error_check(
            status, "MKLStream::leapfrog", "::vslLeapfrogStream");

        return status;
    }

    /// \brief `vslSkipAheadStream`
    int skip_ahead(long long nskip) const
    {
        int status = ::vslSkipAheadStream(this->get(), nskip);
        internal::mkl_error_check(
            status, "MKLStream::skip_ahead", "::vslSkipAheadStream");

        return status;
    }

    /// \brief `vslGetStreamStateBrng`
    int get_brng() const { return ::vslGetStreamStateBrng(this->get()); }

    /// \brief `vslGetNumRegBrngs`
    static int get_num_reg_brngs() { return ::vslGetNumRegBrngs(); }

    /// \brief `vslGetBrngProperties`
    static int get_brng_properties(
        MKL_INT brng, ::VSLBRngProperties &properties)
    {
        int status = ::vslGetBrngProperties(brng, &properties);
        internal::mkl_error_check(status, "MKLStream::get_brng_properties",
            "::vslGetBrngProperties");

        return status;
    }

    /// \brief `vsRngUniform`
    int uniform(MKL_INT n, float *r, float a = 0, float b = 1,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD) const
    {
        int status = ::vsRngUniform(method, this->get(), n, r, a, b);
        internal::mkl_error_check(
            status, "MKLStream::uniform", "::vsRngUniform");

        return status;
    }

    /// \brief `vdRngUniform`
    int uniform(MKL_INT n, double *r, double a = 0, double b = 1,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD) const
    {
        int status = ::vdRngUniform(method, this->get(), n, r, a, b);
        internal::mkl_error_check(
            status, "MKLStream::uniform", "::vdRngUniform");

        return status;
    }

    /// \brief `vsRngGaussian`
    int gaussian(MKL_INT n, float *r, float a = 0, float sigma = 1,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2) const
    {
        int status = ::vsRngGaussian(method, this->get(), n, r, a, sigma);
        internal::mkl_error_check(
            status, "MKLStream::gaussian", "::vsRngGaussian");

        return status;
    }

    /// \brief `vdRngGaussian`
    int gaussian(MKL_INT n, double *r, double a = 0, double sigma = 1,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2) const
    {
        int status = ::vdRngGaussian(method, this->get(), n, r, a, sigma);
        internal::mkl_error_check(
            status, "MKLStream::gaussian", "::vdRngGaussian");

        return status;
    }

    /// \brief `vsRngGaussianMV`
    int gaussian_mv(MKL_INT n, float *r, MKL_INT dimen, MKL_INT mstorage,
        const float *a, const float *t,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2) const
    {
        int status = ::vsRngGaussianMV(
            method, this->get(), n, r, dimen, mstorage, a, t);
        internal::mkl_error_check(
            status, "MKLStream::gaussian_mv", "::vsRngGaussianMV");

        return status;
    }

    /// \brief `vdRngGaussianMV`
    int gaussian_mv(MKL_INT n, double *r, MKL_INT dimen, MKL_INT mstorage,
        const double *a, const double *t,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2) const
    {
        int status = ::vdRngGaussianMV(
            method, this->get(), n, r, dimen, mstorage, a, t);
        internal::mkl_error_check(
            status, "MKLStream::gaussian_mv", "::vdRngGaussianMV");

        return status;
    }

    /// \brief `vsRngExponential`
    int exponential(MKL_INT n, float *r, float a = 0, float beta = 1,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF) const
    {
        int status = ::vsRngExponential(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::exponential", "::vsRngExponential");

        return status;
    }

    /// \brief `vdRngExponential`
    int exponential(MKL_INT n, double *r, double a = 0, double beta = 1,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF) const
    {
        int status = ::vdRngExponential(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::exponential", "::vdRngExponential");

        return status;
    }

    /// \brief `vsRngLaplace`
    int laplace(MKL_INT n, float *r, float a = 0, float beta = 1,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF) const
    {
        int status = ::vsRngLaplace(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::laplace", "::vsRngLaplace");

        return status;
    }

    /// \brief `vdRngLaplace`
    int laplace(MKL_INT n, double *r, double a = 0, double beta = 1,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF) const
    {
        int status = ::vdRngLaplace(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::laplace", "::vdRngLaplace");

        return status;
    }

    /// \brief `vsRngWeibull`
    int weibull(MKL_INT n, float *r, float alpha = 1, float a = 0,
        float beta = 1, MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF) const
    {
        int status = ::vsRngWeibull(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::weibull", "::vsRngWeibull");

        return status;
    }

    /// \brief `vdRngWeibull`
    int weibull(MKL_INT n, double *r, double alpha = 1, double a = 0,
        double beta = 1, MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF) const
    {
        int status = ::vdRngWeibull(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::weibull", "::vdRngWeibull");

        return status;
    }

    /// \brief `vsRngCauchy`
    int cauchy(MKL_INT n, float *r, float a = 0, float beta = 1,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF) const
    {
        int status = ::vsRngCauchy(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::cauchy", "::vsRngCauchy");

        return status;
    }

    /// \brief `vdRngCauchy`
    int cauchy(MKL_INT n, double *r, double a = 0, double beta = 1,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF) const
    {
        int status = ::vdRngCauchy(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::cauchy", "::vdRngCauchy");

        return status;
    }

    /// \brief `vsRngRayleigh`
    int rayleigh(MKL_INT n, float *r, float a = 0, float beta = 1,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF) const
    {
        int status = ::vsRngRayleigh(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::rayleigh", "::vsRngRayleigh");

        return status;
    }

    /// \brief `vdRngRayleigh`
    int rayleigh(MKL_INT n, double *r, double a = 0, double beta = 1,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF) const
    {
        int status = ::vdRngRayleigh(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::rayleigh", "::vdRngRayleigh");

        return status;
    }

    /// \brief `vsRngLognormal`
    int lognormal(MKL_INT n, float *r, float a = 0, float sigma = 1,
        float b = 0, float beta = 1,
        MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2) const
    {
        int status =
            ::vsRngLognormal(method, this->get(), n, r, a, sigma, b, beta);
        internal::mkl_error_check(
            status, "MKLStream::lognormal", "::vsRngLognormal");

        return status;
    }

    /// \brief `vdRngLognormal`
    int lognormal(MKL_INT n, double *r, double a = 0, double sigma = 1,
        double b = 0, double beta = 1,
        MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2) const
    {
        int status =
            ::vdRngLognormal(method, this->get(), n, r, a, sigma, b, beta);
        internal::mkl_error_check(
            status, "MKLStream::lognormal", "::vdRngLognormal");

        return status;
    }

    /// \brief `vsRngGumbel`
    int gumbel(MKL_INT n, float *r, float a = 0, float beta = 1,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF) const
    {
        int status = ::vsRngGumbel(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::gumbel", "::vsRngGumbel");

        return status;
    }

    /// \brief `vdRngGumbel`
    int gumbel(MKL_INT n, double *r, double a = 0, double beta = 1,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF) const
    {
        int status = ::vdRngGumbel(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::gumbel", "::vdRngGumbel");

        return status;
    }

    /// \brief `vsRngGamma`
    int gamma(MKL_INT n, float *r, float alpha = 1, float a = 0,
        float beta = 1, MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM) const
    {
        int status = ::vsRngGamma(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(status, "MKLStream::gamma", "::vsRngGamma");

        return status;
    }

    /// \brief `vdRngGamma`
    int gamma(MKL_INT n, double *r, double alpha = 1, double a = 0,
        double beta = 1, MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM) const
    {
        int status = ::vdRngGamma(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(status, "MKLStream::gamma", "::vdRngGamma");

        return status;
    }

    /// \brief `vsRngBeta`
    int beta(MKL_INT n, float *r, float p = 1, float q = 1, float a = 0,
        float beta = 1, MKL_INT method = VSL_RNG_METHOD_BETA_CJA) const
    {
        int status = ::vsRngBeta(method, this->get(), n, r, p, q, a, beta);
        internal::mkl_error_check(status, "MKLStream::beta", "::vsRngBeta");

        return status;
    }

    /// \brief `vdRngBeta`
    int beta(MKL_INT n, double *r, double p = 1, double q = 1, double a = 0,
        double beta = 1, MKL_INT method = VSL_RNG_METHOD_BETA_CJA) const
    {
        int status = ::vdRngBeta(method, this->get(), n, r, p, q, a, beta);
        internal::mkl_error_check(status, "MKLStream::beta", "::vdRngBeta");

        return status;
    }

    /// \brief `viRngUniform`
    int uniform(MKL_INT n, int *r, int a = 0,
        int b = std::numeric_limits<int>::max VSMC_MNE(),
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD) const
    {
        int status = ::viRngUniform(method, this->get(), n, r, a, b);
        internal::mkl_error_check(
            status, "MKLStream::uniform", "::viRngUniform");

        return status;
    }

    /// \brief `viRngUniform`
    int uniform_bits(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS_STD) const
    {
        int status = ::viRngUniformBits(method, this->get(), n, r);
        internal::mkl_error_check(
            status, "MKLStream::uniform_bits", "::viRngUniformBits");

        return status;
    }

    /// \brief `viRngUniform32`
    int uniform_bits32(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS32_STD) const
    {
        int status = ::viRngUniformBits32(method, this->get(), n, r);
        internal::mkl_error_check(
            status, "MKLStream::uniform_bits32", "::viRngUniformBits32");

        return status;
    }

    /// \brief `viRngUniform64`
    int uniform_bits64(MKL_INT n, unsigned MKL_INT64 *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS64_STD) const
    {
        int status = ::viRngUniformBits64(method, this->get(), n, r);
        internal::mkl_error_check(
            status, "MKLStream::uniform_bits64", "::viRngUniformBits64");

        return status;
    }

    /// \brief `viRngBernoulli`
    int bernoulli(MKL_INT n, int *r, double p = 0.5,
        MKL_INT method = VSL_RNG_METHOD_BERNOULLI_ICDF) const
    {
        int status = ::viRngBernoulli(method, this->get(), n, r, p);
        internal::mkl_error_check(
            status, "MKLStream::bernoulli", "::viRngBernoulli");

        return status;
    }

    /// \brief `viRngGeometric`
    int geometric(MKL_INT n, int *r, double p = 0.5,
        MKL_INT method = VSL_RNG_METHOD_GEOMETRIC_ICDF) const
    {
        int status = ::viRngGeometric(method, this->get(), n, r, p);
        internal::mkl_error_check(
            status, "MKLStream::geometric", "::viRngGeometric");

        return status;
    }

    /// \brief `viRngBinomial`
    int binomial(MKL_INT n, int *r, int ntrial = 1, double p = 0.5,
        MKL_INT method = VSL_RNG_METHOD_BINOMIAL_BTPE) const
    {
        int status = ::viRngBinomial(method, this->get(), n, r, ntrial, p);
        internal::mkl_error_check(
            status, "MKLStream::binomial", "::viRngBinomial");

        return status;
    }

    /// \brief `viRngHypergeometric`
    int hypergeometric(MKL_INT n, int *r, int l, int s, int m,
        MKL_INT method = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE) const
    {
        int status = ::viRngHypergeometric(method, this->get(), n, r, l, s, m);
        internal::mkl_error_check(
            status, "MKLStream::hypergeometric", "::viRngHypergeometric");

        return status;
    }

    /// \brief `viRngPoisson`
    int poisson(MKL_INT n, int *r, double lambda = 1,
        MKL_INT method = VSL_RNG_METHOD_POISSON_PTPE) const
    {
        int status = ::viRngPoisson(method, this->get(), n, r, lambda);
        internal::mkl_error_check(
            status, "MKLStream::poisson", "::viRngPoisson");

        return status;
    }

    /// \brief `viRngPoissonV`
    int poisson_v(MKL_INT n, int *r, const double *lambda,
        MKL_INT method = VSL_RNG_METHOD_POISSONV_POISNORM) const
    {
        int status = ::viRngPoissonV(method, this->get(), n, r, lambda);
        internal::mkl_error_check(
            status, "MKLStream::poisson_v", "::viRngPoissonV");

        return status;
    }

    /// \brief `viRngNegbinomial`
    int neg_binomial(MKL_INT n, int *r, double a = 1, double p = 0.5,
        MKL_INT method = VSL_RNG_METHOD_NEGBINOMIAL_NBAR) const
    {
        int status = ::viRngNegbinomial(method, this->get(), n, r, a, p);
        internal::mkl_error_check(
            status, "MKLStream::neg_binomial", "::viRngNegbinomial");

        return status;
    }
}; // class MKLStream

/// \brief MKL `VSLSSTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLSSTask : public MKLBase<::VSLSSTaskPtr, MKLSSTask<ResultType>>
{
    public:
    using result_type = ResultType;

    MKLSSTask()
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_SS_TASK_RESULT_TYPE(ResultType);
    }

    MKLSSTask(const MKL_INT *p, const MKL_INT *n, const MKL_INT *xstorage,
        const result_type *x, const result_type *w = nullptr,
        const MKL_INT *indices = nullptr)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_SS_TASK_RESULT_TYPE(ResultType);
        reset(p, n, xstorage, x, w, indices);
    }

    static int release(::VSLSSTaskPtr ptr)
    {
        if (ptr == nullptr)
            return 0;

        int status = ::vslSSDeleteTask(&ptr);
        internal::mkl_error_check(
            status, "MKLSSTask::release", "::vslDeleteTask");

        return status;
    }

    int reset(const MKL_INT *p, const MKL_INT *n, const MKL_INT *xstorage,
        const result_type *x, const result_type *w = nullptr,
        const MKL_INT *indices = nullptr)
    {
        ::VSLSSTaskPtr ptr = nullptr;
        int status = reset_dispatch(&ptr, p, n, xstorage, x, w, indices);
        this->reset_ptr(ptr);

        return status;
    }

    int edit_task(MKL_INT parameter, const result_type *par_addr)
    {
        return edit_task_dispatch(this->get(), parameter, par_addr);
    }

    int edit_task(MKL_INT parameter, const MKL_INT *par_addr)
    {
        return edit_task_dispatch(this->get(), parameter, par_addr);
    }

    int compute(unsigned MKL_INT64 estimates, MKL_INT method)
    {
        return compute_dispatch(this->get(), estimates, method,
            static_cast<result_type *>(nullptr));
    }

    private:
    static int reset_dispatch(::VSLSSTaskPtr *task, const MKL_INT *p,
        const MKL_INT *n, const MKL_INT *xstorage, const float *x,
        const float *w, const MKL_INT *indices)
    {
        int status = ::vslsSSNewTask(task, p, n, xstorage, x, w, indices);
        internal::mkl_error_check(
            status, "MKLSSTask::reset", "::vslsSSNewTask");

        return status;
    }

    static int reset_dispatch(::VSLSSTaskPtr *task, const MKL_INT *p,
        const MKL_INT *n, const MKL_INT *xstorage, const double *x,
        const double *w, const MKL_INT *indices)
    {
        int status = ::vsldSSNewTask(task, p, n, xstorage, x, w, indices);
        internal::mkl_error_check(
            status, "MKLSSTask::reset", "::vsldSSNewTask");

        return status;
    }

    static int edit_task_dispatch(
        ::VSLSSTaskPtr task, MKL_INT parameter, const float *par_addr)
    {
        int status = ::vslsSSEditTask(task, parameter, par_addr);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_task", "::vslsSSEditTask");

        return status;
    }

    static int edit_task_dispatch(
        ::VSLSSTaskPtr task, MKL_INT parameter, const double *par_addr)
    {
        int status = ::vsldSSEditTask(task, parameter, par_addr);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_task", "::vsldSSEditTask");

        return status;
    }

    static int edit_task_dispatch(
        ::VSLSSTaskPtr task, MKL_INT parameter, const MKL_INT *par_addr)
    {
        int status = ::vsliSSEditTask(task, parameter, par_addr);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_task", "::vsliSSEditTask");

        return status;
    }

    static int compute_dispatch(::VSLSSTaskPtr task,
        unsigned MKL_INT64 estimates, MKL_INT method, const float *)
    {
        int status = ::vslsSSCompute(task, estimates, method);
        internal::mkl_error_check(
            status, "MKLSSTask::compute", "::vslsSSCompute");

        return status;
    }

    static int compute_dispatch(::VSLSSTaskPtr task,
        unsigned MKL_INT64 estimates, MKL_INT method, const double *)
    {
        int status = ::vsldSSCompute(task, estimates, method);
        internal::mkl_error_check(
            status, "MKLSSTask::compute", "::vsldSSCompute");

        return status;
    }
}; // class MKLSSTask

/// \brief MKL `VSLConvTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLConvTask : public MKLBase<::VSLConvTaskPtr, MKLConvTask<ResultType>>
{
    public:
    using result_type = ResultType;

    MKLConvTask()
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType);
    }

    /// \brief `vslConvNewTask`
    MKLConvTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        reset(mode, dims, xshape, yshape, zshape);
    }

    /// \brief `vslConvNewTask1D`
    MKLConvTask(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        reset(mode, xshape, yshape, zshape);
    }

    /// \brief `vslConvNewTaskX`
    MKLConvTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        reset(mode, dims, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslConvNewTaskX1D`
    MKLConvTask(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType);
        reset(mode, xshape, yshape, zshape, x, xstride);
    }

    MKLConvTask(const MKLConvTask<ResultType> &other)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType);

        ::VSLConvTaskPtr ptr = nullptr;
        internal::mkl_error_check(::vslConvCopyTask(&ptr, other.get()),
            "MKLConvTask::MKLConvTask", "::vslConvCopyTask");
        this->reset_ptr(ptr);
    }

    MKLConvTask<ResultType> &operator=(const MKLConvTask<ResultType> &other)
    {
        if (this != &other) {
            ::VSLConvTaskPtr ptr = nullptr;
            internal::mkl_error_check(::vslConvCopyTask(&ptr, other.get()),
                "MKLConvTask::operator=", "::vslConvCopyTask");
            this->reset_ptr(ptr);
        }

        return *this;
    }

    MKLConvTask(MKLConvTask<ResultType> &&) = default;
    MKLConvTask<ResultType> &operator=(MKLConvTask<ResultType> &&) = default;

    static int release(::VSLConvTaskPtr ptr)
    {
        if (ptr == nullptr)
            return 0;

        int status = ::vslConvDeleteTask(&ptr);
        internal::mkl_error_check(
            status, "MKLConvTask::release", "::vslConvDeleteTask");

        return status;
    }

    /// \brief `vslConvNewTask`
    int reset(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        ::VSLConvTaskPtr ptr = nullptr;
        int status = reset_dispatch(&ptr, mode, dims, xshape, yshape, zshape);
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslConvNewTask1D`
    int reset(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        ::VSLConvTaskPtr ptr = nullptr;
        int status = reset_dispatch(&ptr, mode, xshape, yshape, zshape);
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslConvNewTaskX`
    int reset(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr = nullptr;
        int status = reset_dispatch(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslConvNewTaskX1D`
    int reset(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr = nullptr;
        int status =
            reset_dispatch(&ptr, mode, xshape, yshape, zshape, x, xstride);
        this->reset_ptr(ptr);

        return status;
    }

    private:
    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, float *)
    {
        int status =
            ::vslsConvNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTask");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, double *)
    {
        int status =
            ::vsldConvNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTask");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, MKL_Complex8 *)
    {
        int status =
            ::vslcConvNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTask");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, MKL_Complex16 *)
    {
        int status =
            ::vslzConvNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTask");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        float *)
    {
        int status = ::vslsConvNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        double *)
    {
        int status = ::vsldConvNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex8 *)
    {
        int status = ::vslcConvNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex16 *)
    {
        int status = ::vslzConvNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const float *x, const MKL_INT *xstride)
    {
        int status = ::vslsConvNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const double *x, const MKL_INT *xstride)
    {
        int status = ::vsldConvNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const MKL_Complex8 *x, const MKL_INT *xstride)
    {
        int status = ::vslcConvNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const MKL_Complex16 *x, const MKL_INT *xstride)
    {
        int status = ::vslzConvNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const float *x, const MKL_INT xstride)
    {
        int status = ::vslsConvNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTaskX1D");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const double *x, const MKL_INT xstride)
    {
        int status = ::vsldConvNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTaskX1D");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex8 *x, const MKL_INT xstride)
    {
        int status = ::vslcConvNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTaskX1D");

        return status;
    }

    static int reset_dispatch(::VSLConvTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex16 *x, const MKL_INT xstride)
    {
        int status = ::vslzConvNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTaskX1D");

        return status;
    }
}; // class MKLConvTask

/// \brief MKL `VSLCorrTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLCorrTask : public MKLBase<::VSLCorrTaskPtr, MKLCorrTask<ResultType>>
{
    public:
    using result_type = ResultType;

    MKLCorrTask()
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType);
    }

    /// \brief `vslCorrNewTask`
    MKLCorrTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        reset(mode, dims, xshape, yshape, zshape);
    }

    /// \brief `vslCorrNewTask1D`
    MKLCorrTask(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        reset(mode, xshape, yshape, zshape);
    }

    /// \brief `vslCorrNewTaskX`
    MKLCorrTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        reset(mode, dims, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslCorrNewTaskX1D`
    MKLCorrTask(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType);
        reset(mode, xshape, yshape, zshape, x, xstride);
    }

    MKLCorrTask(const MKLCorrTask<ResultType> &other)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType);

        ::VSLCorrTaskPtr ptr = nullptr;
        internal::mkl_error_check(::vslCorrCopyTask(&ptr, other.get()),
            "MKLCorrTask::MKLCorrTask", "::vslCorrCopyTask");
        this->reset_ptr(ptr);
    }

    MKLCorrTask<ResultType> &operator=(const MKLCorrTask<ResultType> &other)
    {
        if (this != &other) {
            ::VSLCorrTaskPtr ptr = nullptr;
            internal::mkl_error_check(::vslCorrCopyTask(&ptr, other.get()),
                "MKLCorrTask::operator=", "::vslCorrCopyTask");
            this->reset_ptr(ptr);
        }

        return *this;
    }

    MKLCorrTask(MKLCorrTask<ResultType> &&) = default;
    MKLCorrTask<ResultType> &operator=(MKLCorrTask<ResultType> &&) = default;

    static int release(::VSLCorrTaskPtr ptr)
    {
        if (ptr == nullptr)
            return 0;

        int status = ::vslCorrDeleteTask(&ptr);
        internal::mkl_error_check(
            status, "MKLCorrTask::release", "::vslCorrDeleteTask");

        return status;
    }

    /// \brief `vslCorrNewTask`
    int reset(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        ::VSLCorrTaskPtr ptr = nullptr;
        int status = reset_dispatch(&ptr, mode, dims, xshape, yshape, zshape);
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslCorrNewTask1D`
    int reset(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        ::VSLCorrTaskPtr ptr = nullptr;
        int status = reset_dispatch(&ptr, mode, xshape, yshape, zshape);
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslCorrNewTaskX`
    int reset(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr = nullptr;
        int status = reset_dispatch(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslCorrNewTaskX1D`
    int reset(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr = nullptr;
        int status =
            reset_dispatch(&ptr, mode, xshape, yshape, zshape, x, xstride);
        this->reset_ptr(ptr);

        return status;
    }

    private:
    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, float *)
    {
        int status =
            ::vslsCorrNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTask");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, double *)
    {
        int status =
            ::vsldCorrNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTask");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, MKL_Complex8 *)
    {
        int status =
            ::vslcCorrNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTask");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, MKL_Complex16 *)
    {
        int status =
            ::vslzCorrNewTask(task, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTask");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        float *)
    {
        int status = ::vslsCorrNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        double *)
    {
        int status = ::vsldCorrNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex8 *)
    {
        int status = ::vslcCorrNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        MKL_Complex16 *)
    {
        int status = ::vslzCorrNewTask1D(task, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTask1D");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const float *x, const MKL_INT *xstride)
    {
        int status = ::vslsCorrNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const double *x, const MKL_INT *xstride)
    {
        int status = ::vsldCorrNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const MKL_Complex8 *x, const MKL_INT *xstride)
    {
        int status = ::vslcCorrNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        MKL_INT dims, const MKL_INT *xshape, const MKL_INT *yshape,
        const MKL_INT *zshape, const MKL_Complex16 *x, const MKL_INT *xstride)
    {
        int status = ::vslzCorrNewTaskX(
            task, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTaskX");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const float *x, const MKL_INT xstride)
    {
        int status = ::vslsCorrNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTaskX1D");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const double *x, const MKL_INT xstride)
    {
        int status = ::vsldCorrNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTaskX1D");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex8 *x, const MKL_INT xstride)
    {
        int status = ::vslcCorrNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTaskX1D");

        return status;
    }

    static int reset_dispatch(::VSLCorrTaskPtr *task, const MKL_INT mode,
        const MKL_INT xshape, const MKL_INT yshape, const MKL_INT zshape,
        const MKL_Complex16 *x, const MKL_INT xstride)
    {
        int status = ::vslzCorrNewTaskX1D(
            task, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTaskX1D");

        return status;
    }
}; // class MKLCorrTask

/// \brief MKL `DFTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLDFTask
{
    public:
    using result_type = ResultType;

    MKLDFTask()
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_DF_TASK_RESULT_TYPE(ResultType);
    }

    MKLDFTask(MKL_INT nx, const result_type *x, MKL_INT xhint, MKL_INT ny,
        const result_type *y, MKL_INT yhint)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_DF_TASK_RESULT_TYPE(ResultType);
        reset(nx, x, xhint, ny, y, yhint);
    }

    static int release(::DFTaskPtr ptr)
    {
        if (ptr == nullptr)
            return 0;

        int status = ::dfDeleteTask(&ptr);
        internal::mkl_error_check(
            status, "MKLDFTask::release", "::dfDeleteTask");

        return status;
    }

    int reset(MKL_INT nx, const result_type *x, MKL_INT xhint, MKL_INT ny,
        const result_type *y, MKL_INT yhint)
    {
        ::DFTaskPtr ptr = nullptr;
        int status = reset_dispatch(&ptr, nx, x, xhint, ny, y, yhint);
        this->reset_ptr(ptr);

        return status;
    }

    private:
    static int reset_dispatch(::DFTaskPtr *task, MKL_INT nx, const float *x,
        MKL_INT xhint, MKL_INT ny, const float *y, MKL_INT yhint)
    {
        int status = ::dfsNewTask1D(task, nx, x, xhint, ny, y, yhint);
        internal::mkl_error_check(
            status, "MKLDFTask::reset", "::dfsNewTask1D");

        return status;
    }

    static int reset_dispatch(::DFTaskPtr *task, MKL_INT nx, const double *x,
        MKL_INT xhint, MKL_INT ny, const double *y, MKL_INT yhint)
    {
        int status = ::dfdNewTask1D(task, nx, x, xhint, ny, y, yhint);
        internal::mkl_error_check(
            status, "MKLDFTask::reset", "::dfdNewTask1D");

        return status;
    }
}; // class MKLDFTask

} // namespace vsmc

#endif // VSMC_UTILITY_MKL_HPP
