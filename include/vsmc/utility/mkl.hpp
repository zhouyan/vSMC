//============================================================================
// vSMC/include/vsmc/utility/mkl.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/internal/common.hpp>
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

    void reset(pointer ptr)
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
    void reset_ptr(pointer ptr) { reset(ptr); }

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

    /// \brief `vslNewStream`
    MKLStream(MKL_INT brng, MKL_UINT seed) { reset(brng, seed); }

    /// \brief `vslNewStreamEx`
    MKLStream(MKL_INT brng, MKL_INT n, unsigned *params)
    {
        reset(brng, n, params);
    }

    /// \brief `vslCopyStream`
    MKLStream(const MKLStream &other)
        : MKLBase<::VSLStreamStatePtr, MKLStream>()
    {
        ::VSLStreamStatePtr ptr = nullptr;
        internal::mkl_error_check(::vslCopyStream(&ptr, other.get()),
            "MKLStream::MKLStream", "::vslCopyStream");
        this->reset_ptr(ptr);
    }

    /// \brief `vslCopyStream`/`vslCopySreamState`
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

    /// \brief `vslNewStream`
    int reset(MKL_INT brng, MKL_UINT seed)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = ::vslNewStream(&ptr, brng, seed);
        internal::mkl_error_check(
            status, "MKLStream::reset", "::vslNewStream");
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslNewStreamEx`
    int reset(MKL_INT brng, MKL_INT n, unsigned *params)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = ::vslNewStreamEx(&ptr, brng, n, params);
        internal::mkl_error_check(
            status, "MKLStream::reset", "::vslNewStreamEx");
        this->reset_ptr(ptr);

        return status;
    }

    /// \brief `vslDeleteStream`
    static int release(::VSLStreamStatePtr ptr)
    {
        if (ptr == nullptr)
            return 0;

        int status = ::vslDeleteStream(&ptr);
        internal::mkl_error_check(
            status, "MKLStream::release", "::vslDeleteStream");

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
    int leapfrog(MKL_INT k, MKL_INT nstreams)
    {
        int status = ::vslLeapfrogStream(this->get(), k, nstreams);
        internal::mkl_error_check(
            status, "MKLStream::leapfrog", "::vslLeapfrogStream");

        return status;
    }

    /// \brief `vslSkipAheadStream`
    int skip_ahead(long long nskip)
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
    int uniform(MKL_INT n, float *r, float a, float b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        int status = ::vsRngUniform(method, this->get(), n, r, a, b);
        internal::mkl_error_check(
            status, "MKLStream::uniform", "::vsRngUniform");

        return status;
    }

    /// \brief `vdRngUniform`
    int uniform(MKL_INT n, double *r, double a, double b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        int status = ::vdRngUniform(method, this->get(), n, r, a, b);
        internal::mkl_error_check(
            status, "MKLStream::uniform", "::vdRngUniform");

        return status;
    }

    /// \brief `vsRngGaussian`
    int gaussian(MKL_INT n, float *r, float a, float sigma,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2)
    {
        int status = ::vsRngGaussian(method, this->get(), n, r, a, sigma);
        internal::mkl_error_check(
            status, "MKLStream::gaussian", "::vsRngGaussian");

        return status;
    }

    /// \brief `vdRngGaussian`
    int gaussian(MKL_INT n, double *r, double a, double sigma,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2)
    {
        int status = ::vdRngGaussian(method, this->get(), n, r, a, sigma);
        internal::mkl_error_check(
            status, "MKLStream::gaussian", "::vdRngGaussian");

        return status;
    }

    /// \brief `vsRngGaussianMV`
    int gaussian_mv(MKL_INT n, float *r, MKL_INT dimen, MKL_INT mstorage,
        const float *a, const float *t,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2)
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
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2)
    {
        int status = ::vdRngGaussianMV(
            method, this->get(), n, r, dimen, mstorage, a, t);
        internal::mkl_error_check(
            status, "MKLStream::gaussian_mv", "::vdRngGaussianMV");

        return status;
    }

    /// \brief `vsRngExponential`
    int exponential(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF)
    {
        int status = ::vsRngExponential(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::exponential", "::vsRngExponential");

        return status;
    }

    /// \brief `vdRngExponential`
    int exponential(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF)
    {
        int status = ::vdRngExponential(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::exponential", "::vdRngExponential");

        return status;
    }

    /// \brief `vsRngLaplace`
    int laplace(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF)
    {
        int status = ::vsRngLaplace(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::laplace", "::vsRngLaplace");

        return status;
    }

    /// \brief `vdRngLaplace`
    int laplace(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF)
    {
        int status = ::vdRngLaplace(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::laplace", "::vdRngLaplace");

        return status;
    }

    /// \brief `vsRngWeibull`
    int weibull(MKL_INT n, float *r, float alpha, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF)
    {
        int status = ::vsRngWeibull(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::weibull", "::vsRngWeibull");

        return status;
    }

    /// \brief `vdRngWeibull`
    int weibull(MKL_INT n, double *r, double alpha, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF)
    {
        int status = ::vdRngWeibull(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::weibull", "::vdRngWeibull");

        return status;
    }

    /// \brief `vsRngCauchy`
    int cauchy(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF)
    {
        int status = ::vsRngCauchy(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::cauchy", "::vsRngCauchy");

        return status;
    }

    /// \brief `vdRngCauchy`
    int cauchy(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF)
    {
        int status = ::vdRngCauchy(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::cauchy", "::vdRngCauchy");

        return status;
    }

    /// \brief `vsRngRayleigh`
    int rayleigh(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF)
    {
        int status = ::vsRngRayleigh(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::rayleigh", "::vsRngRayleigh");

        return status;
    }

    /// \brief `vdRngRayleigh`
    int rayleigh(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF)
    {
        int status = ::vdRngRayleigh(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::rayleigh", "::vdRngRayleigh");

        return status;
    }

    /// \brief `vsRngLognormal`
    int lognormal(MKL_INT n, float *r, float a, float sigma, float b,
        float beta, MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2)
    {
        int status =
            ::vsRngLognormal(method, this->get(), n, r, a, sigma, b, beta);
        internal::mkl_error_check(
            status, "MKLStream::lognormal", "::vsRngLognormal");

        return status;
    }

    /// \brief `vdRngLognormal`
    int lognormal(MKL_INT n, double *r, double a, double sigma, double b,
        double beta, MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2)
    {
        int status =
            ::vdRngLognormal(method, this->get(), n, r, a, sigma, b, beta);
        internal::mkl_error_check(
            status, "MKLStream::lognormal", "::vdRngLognormal");

        return status;
    }

    /// \brief `vsRngGumbel`
    int gumbel(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF)
    {
        int status = ::vsRngGumbel(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::gumbel", "::vsRngGumbel");

        return status;
    }

    /// \brief `vdRngGumbel`
    int gumbel(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF)
    {
        int status = ::vdRngGumbel(method, this->get(), n, r, a, beta);
        internal::mkl_error_check(
            status, "MKLStream::gumbel", "::vdRngGumbel");

        return status;
    }

    /// \brief `vsRngGamma`
    int gamma(MKL_INT n, float *r, float alpha, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM)
    {
        int status = ::vsRngGamma(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(status, "MKLStream::gamma", "::vsRngGamma");

        return status;
    }

    /// \brief `vdRngGamma`
    int gamma(MKL_INT n, double *r, double alpha, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM)
    {
        int status = ::vdRngGamma(method, this->get(), n, r, alpha, a, beta);
        internal::mkl_error_check(status, "MKLStream::gamma", "::vdRngGamma");

        return status;
    }

    /// \brief `vsRngBeta`
    int beta(MKL_INT n, float *r, float p, float q, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_BETA_CJA)
    {
        int status = ::vsRngBeta(method, this->get(), n, r, p, q, a, beta);
        internal::mkl_error_check(status, "MKLStream::beta", "::vsRngBeta");

        return status;
    }

    /// \brief `vdRngBeta`
    int beta(MKL_INT n, double *r, double p, double q, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_BETA_CJA)
    {
        int status = ::vdRngBeta(method, this->get(), n, r, p, q, a, beta);
        internal::mkl_error_check(status, "MKLStream::beta", "::vdRngBeta");

        return status;
    }

    /// \brief `viRngUniform`
    int uniform(MKL_INT n, int *r, int a, int b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        int status = ::viRngUniform(method, this->get(), n, r, a, b);
        internal::mkl_error_check(
            status, "MKLStream::uniform", "::viRngUniform");

        return status;
    }

    /// \brief `viRngUniform`
    int uniform_bits(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS_STD)
    {
        int status = ::viRngUniformBits(method, this->get(), n, r);
        internal::mkl_error_check(
            status, "MKLStream::uniform_bits", "::viRngUniformBits");

        return status;
    }

    /// \brief `viRngUniform32`
    int uniform_bits32(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS32_STD)
    {
        int status = ::viRngUniformBits32(method, this->get(), n, r);
        internal::mkl_error_check(
            status, "MKLStream::uniform_bits32", "::viRngUniformBits32");

        return status;
    }

    /// \brief `viRngUniform64`
    int uniform_bits64(MKL_INT n, unsigned MKL_INT64 *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS64_STD)
    {
        int status = ::viRngUniformBits64(method, this->get(), n, r);
        internal::mkl_error_check(
            status, "MKLStream::uniform_bits64", "::viRngUniformBits64");

        return status;
    }

    /// \brief `viRngBernoulli`
    int bernoulli(MKL_INT n, int *r, double p,
        MKL_INT method = VSL_RNG_METHOD_BERNOULLI_ICDF)
    {
        int status = ::viRngBernoulli(method, this->get(), n, r, p);
        internal::mkl_error_check(
            status, "MKLStream::bernoulli", "::viRngBernoulli");

        return status;
    }

    /// \brief `viRngGeometric`
    int geometric(MKL_INT n, int *r, double p,
        MKL_INT method = VSL_RNG_METHOD_GEOMETRIC_ICDF)
    {
        int status = ::viRngGeometric(method, this->get(), n, r, p);
        internal::mkl_error_check(
            status, "MKLStream::geometric", "::viRngGeometric");

        return status;
    }

    /// \brief `viRngBinomial`
    int binomial(MKL_INT n, int *r, int ntrial, double p,
        MKL_INT method = VSL_RNG_METHOD_BINOMIAL_BTPE)
    {
        int status = ::viRngBinomial(method, this->get(), n, r, ntrial, p);
        internal::mkl_error_check(
            status, "MKLStream::binomial", "::viRngBinomial");

        return status;
    }

    /// \brief `viRngHypergeometric`
    int hypergeometric(MKL_INT n, int *r, int l, int s, int m,
        MKL_INT method = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE)
    {
        int status = ::viRngHypergeometric(method, this->get(), n, r, l, s, m);
        internal::mkl_error_check(
            status, "MKLStream::hypergeometric", "::viRngHypergeometric");

        return status;
    }

    /// \brief `viRngPoisson`
    int poisson(MKL_INT n, int *r, double lambda,
        MKL_INT method = VSL_RNG_METHOD_POISSON_PTPE)
    {
        int status = ::viRngPoisson(method, this->get(), n, r, lambda);
        internal::mkl_error_check(
            status, "MKLStream::poisson", "::viRngPoisson");

        return status;
    }

    /// \brief `viRngPoissonV`
    int poisson_v(MKL_INT n, int *r, const double *lambda,
        MKL_INT method = VSL_RNG_METHOD_POISSONV_POISNORM)
    {
        int status = ::viRngPoissonV(method, this->get(), n, r, lambda);
        internal::mkl_error_check(
            status, "MKLStream::poisson_v", "::viRngPoissonV");

        return status;
    }

    /// \brief `viRngNegbinomial`
    int neg_binomial(MKL_INT n, int *r, double a, double p,
        MKL_INT method = VSL_RNG_METHOD_NEGBINOMIAL_NBAR)
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

    /// \brief `vslSSNewTask`
    MKLSSTask(const MKL_INT *p, const MKL_INT *n, const MKL_INT *xstorage,
        const result_type *x, const result_type *w, const MKL_INT *indices)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_SS_TASK_RESULT_TYPE(ResultType);
        reset(p, n, xstorage, x, w, indices);
    }

    /// \brief `vslSSNewTask`
    int reset(const MKL_INT *p, const MKL_INT *n, const MKL_INT *xstorage,
        const result_type *x, const result_type *w, const MKL_INT *indices)
    {
        return reset_dispatch(p, n, xstorage, x, w, indices);
    }

    /// \brief `vslSSDeleteTask`
    static int release(::VSLSSTaskPtr ptr)
    {
        if (ptr == nullptr)
            return 0;

        int status = ::vslSSDeleteTask(&ptr);
        internal::mkl_error_check(
            status, "MKLSSTask::release", "::vslSSDeleteTask");

        return status;
    }

    /// \brief `vslSSEditTask`
    int edit_task(MKL_INT parameter, const result_type *par_addr)
    {
        return edit_task_dispatch(parameter, par_addr);
    }

    /// \brief `vsliSSEditTask`
    int edit_task(MKL_INT parameter, const MKL_INT *par_addr)
    {
        return edit_task_dispatch(parameter, par_addr);
    }

    /// \brief `vslSSEditMoments`
    int edit_moments(const result_type *mean, const result_type *r2m,
        const result_type *r3m, const result_type *r4m, const result_type *c2m,
        const result_type *c3m, const result_type *c4m)
    {
        return edit_moments_dispatch(mean, r2m, r3m, r4m, c2m, c3m, c4m);
    }

    /// \brief `vslSSEditSums`
    int edit_sums(const result_type *sum, const result_type *r2s,
        const result_type *r3s, const result_type *r4s, const result_type *c2s,
        const result_type *c3s, const result_type *c4s)
    {
        return edit_moments_dispatch(sum, r2s, r3s, r4s, c2s, c3s, c4s);
    }

    /// \brief `vslSSEditCovCor`
    int edit_cov_cor(const result_type *mean, const result_type *cov,
        const MKL_INT *cov_storage, const result_type *cor,
        const MKL_INT *cor_storage)
    {
        return edit_cov_cor_dispatch(mean, cov, cov_storage, cor, cor_storage);
    }

    /// \brief `vslSSEditCP`
    int edit_cp(const result_type *mean, const result_type *sum,
        const result_type *cp, const MKL_INT *cp_storage)
    {
        return edit_cp_dispatch(mean, sum, cp, cp_storage);
    }

    /// \brief `vslSSEditPartialCovCor`
    int edit_partial_cov_cor(const MKL_INT *p_idx_array,
        const result_type *cov, const MKL_INT *cov_storage,
        const result_type *cor, const MKL_INT *cor_storage,
        const result_type *p_cov, const MKL_INT *p_cov_storage,
        const result_type *p_cor, const MKL_INT *p_cor_storage) const
    {
        return edit_partial_cov_cor_dispatch(p_idx_array, cov, cov_storage,
            cor, cor_storage, p_cov, p_cov_storage, p_cor, p_cor_storage);
    }

    /// \brief `vslSSEditQuantiles`
    int edit_quantiles(const MKL_INT *quant_order_n,
        const result_type *quant_order, const result_type *quant,
        const result_type *order_stats, const MKL_INT *order_stats_storage)
    {
        return edit_quantiles_dispatch(quant_order_n, quant_order, quant,
            order_stats, order_stats_storage);
    }

    /// \brief `vslSSEditStreamQuantiles`
    int edit_stream_quantiles(const MKL_INT *quant_order_n,
        const result_type *quant_order, const result_type *quants,
        const MKL_INT *nparams, const result_type *params)
    {
        return edit_stream_quantiles_dispatch(
            quant_order_n, quant_order, quants, nparams, params);
    }

    /// \brief `vslSSEditPooledCovariance`
    int edit_pooled_covariance(const MKL_INT *grp_indices,
        const result_type *pld_mean, const result_type *pld_cov,
        const MKL_INT *req_grp_indices, const result_type *grp_means,
        const result_type *grp_cov)
    {
        return edit_pooled_covariance_dispatch(grp_indices, pld_mean, pld_cov,
            req_grp_indices, grp_means, grp_cov);
    }

    /// \brief `vslSSEditRobustCovariance`
    int edit_robust_covariance(const MKL_INT *rcov_storage,
        const MKL_INT *nparams, const result_type *params,
        const result_type *rmean, const result_type *rcov)
    {
        return edit_robust_covariance_dispatch(
            rcov_storage, nparams, params, rmean, rcov);
    }

    /// \brief `vslSSEditOutliersDetection`
    int edit_outliers_detection(const MKL_INT *nparams,
        const result_type *params, const result_type *w)
    {
        return edit_outliers_detection_dispatch(nparams, params, w);
    }

    /// \brief `vslSSEditMissingValues`
    int edit_missing_values(const MKL_INT *nparams, const result_type *params,
        const MKL_INT *init_estimates_n, const result_type *init_estimates,
        const MKL_INT *prior_n, const result_type *prior,
        const MKL_INT *simul_missing_vals_n,
        const result_type *simul_missing_vals, const MKL_INT *estimates_n,
        const result_type *estimates)
    {
        return edit_missing_values_dispatch(nparams, params, init_estimates_n,
            init_estimates, prior_n, prior, simul_missing_vals_n,
            simul_missing_vals, estimates_n, estimates);
    }

    /// \brief `vslSSEditCorParameterization`
    int edit_cor_parameterization(const result_type *cor,
        const MKL_INT *cor_storage, const result_type *pcor,
        const MKL_INT *pcor_storage)
    {
        return edit_cor_parameterization_dispatch(
            cor, cor_storage, pcor, pcor_storage);
    }

    /// \brief `vslSSCompute`
    int compute(unsigned MKL_INT64 estimates, MKL_INT method)
    {
        return compute_dispatch(
            estimates, method, static_cast<result_type *>(nullptr));
    }

    private:
    int reset_dispatch(const MKL_INT *p, const MKL_INT *n,
        const MKL_INT *xstorage, const float *x, const float *w,
        const MKL_INT *indices)
    {
        ::VSLSSTaskPtr ptr;
        int status = ::vslsSSNewTask(&ptr, p, n, xstorage, x, w, indices);
        internal::mkl_error_check(
            status, "MKLSSTask::reset", "::vslsSSNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT *p, const MKL_INT *n,
        const MKL_INT *xstorage, const double *x, const double *w,
        const MKL_INT *indices)
    {
        ::VSLSSTaskPtr ptr;
        int status = ::vsldSSNewTask(&ptr, p, n, xstorage, x, w, indices);
        internal::mkl_error_check(
            status, "MKLSSTask::reset", "::vsldSSNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int edit_task_dispatch(MKL_INT parameter, const float *par_addr)
    {
        int status = ::vslsSSEditTask(this->get(), parameter, par_addr);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_task", "::vslsSSEditTask");

        return status;
    }

    int edit_task_dispatch(MKL_INT parameter, const double *par_addr)
    {
        int status = ::vsldSSEditTask(this->get(), parameter, par_addr);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_task", "::vsldSSEditTask");

        return status;
    }

    int edit_task_dispatch(MKL_INT parameter, const MKL_INT *par_addr)
    {
        int status = ::vsliSSEditTask(this->get(), parameter, par_addr);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_task", "::vsliSSEditTask");

        return status;
    }

    int edit_moments_dispatch(const float *mean, const float *r2m,
        const float *r3m, const float *r4m, const float *c2m, const float *c3m,
        const float *c4m)
    {
        int status = ::vslsSSEditMoments(
            this->get(), mean, r2m, r3m, r4m, c2m, c3m, c4m);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_moments", "::vslsSSEditMoments");

        return status;
    }

    int edit_moments_dispatch(const double *mean, const double *r2m,
        const double *r3m, const double *r4m, const double *c2m,
        const double *c3m, const double *c4m)
    {
        int status = ::vsldSSEditMoments(
            this->get(), mean, r2m, r3m, r4m, c2m, c3m, c4m);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_moments", "::vsldSSEditMoments");

        return status;
    }

    int edit_sums_dispatch(const float *sum, const float *r2s,
        const float *r3s, const float *r4s, const float *c2s, const float *c3s,
        const float *c4s)
    {
        int status =
            ::vslsSSEditSums(this->get(), sum, r2s, r3s, r4s, c2s, c3s, c4s);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_sums", "::vslsSSEditSums");

        return status;
    }

    int edit_sums_dispatch(const double *sum, const double *r2s,
        const double *r3s, const double *r4s, const double *c2s,
        const double *c3s, const double *c4s)
    {
        int status =
            ::vsldSSEditSums(this->get(), sum, r2s, r3s, r4s, c2s, c3s, c4s);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_sums", "::vsldSSEditSums");

        return status;
    }

    int edit_cov_cor_dispatch(const float *mean, const float *cov,
        const MKL_INT *cov_storage, const float *cor,
        const MKL_INT *cor_storage)
    {
        int status = ::vslsSSEditCovCor(
            this->get(), mean, cov, cov_storage, cor, cor_storage);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_cov_cor", "::vslsSSEditCovCor");

        return status;
    }

    int edit_cov_cor_dispatch(const double *mean, const double *cov,
        const MKL_INT *cov_storage, const double *cor,
        const MKL_INT *cor_storage)
    {
        int status = ::vsldSSEditCovCor(
            this->get(), mean, cov, cov_storage, cor, cor_storage);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_cov_cor", "::vsldSSEditCovCor");

        return status;
    }

    int edit_cp_dispatch(const float *mean, const float *sum, const float *cp,
        const MKL_INT *cp_storage)
    {
        int status = ::vslsSSEditCP(this->get(), mean, sum, cp, cp_storage);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_cp", "::vslsSSEditCP");

        return status;
    }

    int edit_cp_dispatch(const double *mean, const double *sum,
        const double *cp, const MKL_INT *cp_storage)
    {
        int status = ::vsldSSEditCP(this->get(), mean, sum, cp, cp_storage);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_cp", "::vsldSSEditCP");

        return status;
    }

    int edit_partial_cov_cor_dispatch(const MKL_INT *p_idx_array,
        const float *cov, const MKL_INT *cov_storage, const float *cor,
        const MKL_INT *cor_storage, const float *p_cov,
        const MKL_INT *p_cov_storage, const float *p_cor,
        const MKL_INT *p_cor_storage) const
    {
        int status = ::vslsSSEditPartialCovCor(this->get(), p_idx_array, cov,
            cov_storage, cor, cor_storage, p_cov, p_cov_storage, p_cor,
            p_cor_storage);
        internal::mkl_error_check(status, "MKLSSTask::edit_partial_cov_cor",
            "::vslsSSEditPartialCovCor");

        return status;
    }

    int edit_partial_cov_cor_dispatch(const MKL_INT *p_idx_array,
        const double *cov, const MKL_INT *cov_storage, const double *cor,
        const MKL_INT *cor_storage, const double *p_cov,
        const MKL_INT *p_cov_storage, const double *p_cor,
        const MKL_INT *p_cor_storage) const
    {
        int status = ::vsldSSEditPartialCovCor(this->get(), p_idx_array, cov,
            cov_storage, cor, cor_storage, p_cov, p_cov_storage, p_cor,
            p_cor_storage);
        internal::mkl_error_check(status, "MKLSSTask::edit_partial_cov_cor",
            "::vsldSSEditPartialCovCor");

        return status;
    }

    int edit_quantiles_dispatch(const MKL_INT *quant_order_n,
        const float *quant_order, const float *quant, const float *order_stats,
        const MKL_INT *order_stats_storage)
    {
        int status = ::vslsSSEditQuantiles(this->get(), quant_order_n,
            quant_order, quant, order_stats, order_stats_storage);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_quantiles", "::vslsSSEditQuantiles");

        return status;
    }

    int edit_quantiles_dispatch(const MKL_INT *quant_order_n,
        const double *quant_order, const double *quant,
        const double *order_stats, const MKL_INT *order_stats_storage)
    {
        int status = ::vsldSSEditQuantiles(this->get(), quant_order_n,
            quant_order, quant, order_stats, order_stats_storage);
        internal::mkl_error_check(
            status, "MKLSSTask::edit_quantiles", "::vsldSSEditQuantiles");

        return status;
    }

    int edit_stream_quantiles_dispatch(const MKL_INT *quant_order_n,
        const float *quant_order, const float *quants, const MKL_INT *nparams,
        const float *params)
    {
        int status = ::vslsSSEditStreamQuantiles(
            this->get(), quant_order_n, quant_order, quants, nparams, params);
        internal::mkl_error_check(status, "MKLSSTask::edit_stream_quantiles",
            "::vslsSSEditStreamQuantiles");

        return status;
    }

    int edit_stream_quantiles_dispatch(const MKL_INT *quant_order_n,
        const double *quant_order, const double *quants,
        const MKL_INT *nparams, const double *params)
    {
        int status = ::vsldSSEditStreamQuantiles(
            this->get(), quant_order_n, quant_order, quants, nparams, params);
        internal::mkl_error_check(status, "MKLSSTask::edit_stream_quantiles",
            "::vsldSSEditStreamQuantiles");

        return status;
    }

    int edit_pooled_covariance_dispatch(const MKL_INT *grp_indices,
        const float *pld_mean, const float *pld_cov,
        const MKL_INT *req_grp_indices, const float *grp_means,
        const float *grp_cov)
    {
        int status = ::vslsSSEditPooledCovariance(this->get(), grp_indices,
            pld_mean, pld_cov, req_grp_indices, grp_means, grp_cov);
        internal::mkl_error_check(status, "MKLSSTask::edit_pooled_covariance",
            "::vslsSSEditPooledCovariance");

        return status;
    }

    int edit_pooled_covariance_dispatch(const MKL_INT *grp_indices,
        const double *pld_mean, const double *pld_cov,
        const MKL_INT *req_grp_indices, const double *grp_means,
        const double *grp_cov)
    {
        int status = ::vsldSSEditPooledCovariance(this->get(), grp_indices,
            pld_mean, pld_cov, req_grp_indices, grp_means, grp_cov);
        internal::mkl_error_check(status, "MKLSSTask::edit_pooled_covariance",
            "::vsldSSEditPooledCovariance");

        return status;
    }

    int edit_robust_covariance_dispatch(const MKL_INT *rcov_storage,
        const MKL_INT *nparams, const float *params, const float *rmean,
        const float *rcov)
    {
        int status = ::vslsSSEditRobustCovariance(
            this->get(), rcov_storage, nparams, params, rmean, rcov);
        internal::mkl_error_check(status, "MKLSSTask::edit_robust_covariance",
            "::vslsSSEditRobustCovariance");

        return status;
    }

    int edit_robust_covariance_dispatch(const MKL_INT *rcov_storage,
        const MKL_INT *nparams, const double *params, const double *rmean,
        const double *rcov)
    {
        int status = ::vsldSSEditRobustCovariance(
            this->get(), rcov_storage, nparams, params, rmean, rcov);
        internal::mkl_error_check(status, "MKLSSTask::edit_robust_covariance",
            "::vsldSSEditRobustCovariance");

        return status;
    }

    int edit_outliers_detection_dispatch(
        const MKL_INT *nparams, const float *params, const float *w)
    {
        int status =
            ::vslsSSEditOutliersDetection(this->get(), nparams, params, w);
        internal::mkl_error_check(status, "MKLSSTask::edit_outliers_detection",
            "::vslsSSEditOutliersDetection");

        return status;
    }

    int edit_outliers_detection_dispatch(
        const MKL_INT *nparams, const double *params, const double *w)
    {
        int status =
            ::vsldSSEditOutliersDetection(this->get(), nparams, params, w);
        internal::mkl_error_check(status, "MKLSSTask::edit_outliers_detection",
            "::vsldSSEditOutliersDetection");

        return status;
    }

    int edit_missing_values_dispatch(const MKL_INT *nparams,
        const float *params, const MKL_INT *init_estimates_n,
        const float *init_estimates, const MKL_INT *prior_n,
        const float *prior, const MKL_INT *simul_missing_vals_n,
        const float *simul_missing_vals, const MKL_INT *estimates_n,
        const float *estimates)
    {
        int status = ::vslsSSEditMissingValues(this->get(), nparams, params,
            init_estimates_n, init_estimates, prior_n, prior,
            simul_missing_vals_n, simul_missing_vals, estimates_n, estimates);
        internal::mkl_error_check(status, "MKLSSTask::edit_missing_values",
            "::vslsSSEditMissingValues");

        return status;
    }

    int edit_missing_values_dispatch(const MKL_INT *nparams,
        const double *params, const MKL_INT *init_estimates_n,
        const double *init_estimates, const MKL_INT *prior_n,
        const double *prior, const MKL_INT *simul_missing_vals_n,
        const double *simul_missing_vals, const MKL_INT *estimates_n,
        const double *estimates)
    {
        int status = ::vsldSSEditMissingValues(this->get(), nparams, params,
            init_estimates_n, init_estimates, prior_n, prior,
            simul_missing_vals_n, simul_missing_vals, estimates_n, estimates);
        internal::mkl_error_check(status, "MKLSSTask::edit_missing_values",
            "::vsldSSEditMissingValues");

        return status;
    }

    int edit_cor_parameterization_dispatch(const float *cor,
        const MKL_INT *cor_storage, const float *pcor,
        const MKL_INT *pcor_storage)
    {
        int status = ::vslsSSEditCorParameterization(
            this->get(), cor, cor_storage, pcor, pcor_storage);
        internal::mkl_error_check(status,
            "MKLSSTask::edit_cor_parameterization",
            "::vslsSSEditCorParameterization");

        return status;
    }

    int edit_cor_parameterization_dispatch(const double *cor,
        const MKL_INT *cor_storage, const double *pcor,
        const MKL_INT *pcor_storage)
    {
        int status = ::vsldSSEditCorParameterization(
            this->get(), cor, cor_storage, pcor, pcor_storage);
        internal::mkl_error_check(status,
            "MKLSSTask::edit_cor_parameterization",
            "::vsldSSEditCorParameterization");

        return status;
    }

    int compute_dispatch(
        unsigned MKL_INT64 estimates, MKL_INT method, const float *)
    {
        int status = ::vslsSSCompute(this->get(), estimates, method);
        internal::mkl_error_check(
            status, "MKLSSTask::compute", "::vslsSSCompute");

        return status;
    }

    int compute_dispatch(
        unsigned MKL_INT64 estimates, MKL_INT method, const double *)
    {
        int status = ::vsldSSCompute(this->get(), estimates, method);
        internal::mkl_error_check(
            status, "MKLSSTask::compute", "::vsldSSCompute");

        return status;
    }
}; // class MKLSSTask

/// \brief Compute covariance matrix using `MKLSSTask`
/// \ingroup MKL
template <typename ResultType = double>
class MKLCovTask
{
    public:
    using result_type = ResultType;

    void operator()(MatrixLayout layout, std::size_t N, std::size_t dim,
        const result_type *x, const result_type *w, result_type *mean,
        result_type *cov, result_type *cor, MatrixLayout cov_layout = RowMajor,
        bool cov_upper = false, bool cov_packed = false,
        MatrixLayout cor_layout = RowMajor, bool cor_upper = false,
        bool cor_packed = false)
    {
        if (N * dim == 0)
            return;

        if (x == nullptr)
            return;

        MKL_INT p = static_cast<MKL_INT>(dim);
        MKL_INT n = static_cast<MKL_INT>(N);
        MKL_INT xstorage = layout == RowMajor ? VSL_SS_MATRIX_STORAGE_COLS :
                                                VSL_SS_MATRIX_STORAGE_ROWS;
        MKL_INT cov_storage = storage(cov_layout, cov_upper, cov_packed);
        MKL_INT cor_storage = storage(cor_layout, cor_upper, cor_packed);
        unsigned MKL_INT64 estimates = 0;
        if (mean != nullptr)
            estimates |= VSL_SS_MEAN;
        if (cov != nullptr)
            estimates |= VSL_SS_COV;
        if (cor != nullptr)
            estimates |= VSL_SS_COR;

        MKLSSTask<result_type> task(&p, &n, &xstorage, x, w, nullptr);
        task.reset(&p, &n, &xstorage, x, w, nullptr);
        task.edit_cov_cor(mean, cov, &cov_storage, cor, &cor_storage);
        vsldSSCompute(task.get(), estimates, VSL_SS_METHOD_FAST);
    }

    private:
    MKL_INT storage(MatrixLayout layout, bool upper, bool packed)
    {
        if (!packed)
            return VSL_SS_MATRIX_STORAGE_FULL;

        if (layout == RowMajor)
            return upper ? VSL_SS_MATRIX_STORAGE_U_PACKED :
                           VSL_SS_MATRIX_STORAGE_L_PACKED;

        return upper ? VSL_SS_MATRIX_STORAGE_L_PACKED :
                       VSL_SS_MATRIX_STORAGE_U_PACKED;
    }
}; // class MKLCovTask

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

    /// \brief `vslConvCopyTask`
    MKLConvTask(const MKLConvTask<ResultType> &other)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CONV_TASK_RESULT_TYPE(ResultType);

        ::VSLConvTaskPtr ptr = nullptr;
        internal::mkl_error_check(::vslConvCopyTask(&ptr, other.get()),
            "MKLConvTask::MKLConvTask", "::vslConvCopyTask");
        this->reset_ptr(ptr);
    }

    /// \brief `vslConvCopyTask`
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

    /// \brief `vslConvDeleteTask`
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
        return reset_dispatch(mode, dims, xshape, yshape, zshape);
    }

    /// \brief `vslConvNewTask1D`
    int reset(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        return reset_dispatch(mode, xshape, yshape, zshape);
    }

    /// \brief `vslConvNewTaskX`
    int reset(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        return reset_dispatch(mode, dims, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslConvNewTaskX1D`
    int reset(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        return reset_dispatch(mode, xshape, yshape, zshape, x, xstride);
    }

    private:
    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, float *)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            ::vslsConvNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, double *)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            ::vsldConvNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex8 *)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            ::vslcConvNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex16 *)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            ::vslzConvNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, float *)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslsConvNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, double *)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vsldConvNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex8 *)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslcConvNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex16 *)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslzConvNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const float *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslsConvNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const double *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vsldConvNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex8 *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslcConvNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex16 *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslzConvNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const float *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslsConvNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslsConvNewTaskX1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const double *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vsldConvNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vsldConvNewTaskX1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex8 *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslcConvNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslcConvNewTaskX1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex16 *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status = ::vslzConvNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLConvTask::reset", "::vslzConvNewTaskX1D");
        this->reset_ptr(ptr);

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

    /// \brief `vslCorrCopyTask`
    MKLCorrTask(const MKLCorrTask<ResultType> &other)
    {
        VSMC_STATIC_ASSERT_UTILITY_MKL_CORR_TASK_RESULT_TYPE(ResultType);

        ::VSLCorrTaskPtr ptr = nullptr;
        internal::mkl_error_check(::vslCorrCopyTask(&ptr, other.get()),
            "MKLCorrTask::MKLCorrTask", "::vslCorrCopyTask");
        this->reset_ptr(ptr);
    }

    /// \brief `vslCorrCopyTask`
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

    /// \brief `vslCorrDeleteTask`
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
        return reset_dispatch(mode, dims, xshape, yshape, zshape);
    }

    /// \brief `vslCorrNewTask1D`
    int reset(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        return reset_dispatch(mode, xshape, yshape, zshape);
    }

    /// \brief `vslCorrNewTaskX`
    int reset(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        return reset_dispatch(mode, dims, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslCorrNewTaskX1D`
    int reset(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        return reset_dispatch(mode, xshape, yshape, zshape, x, xstride);
    }

    private:
    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, float *)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            ::vslsCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, double *)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            ::vsldCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex8 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            ::vslcCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex16 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            ::vslzCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTask");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, float *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslsCorrNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, double *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vsldCorrNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex8 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslcCorrNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex16 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslzCorrNewTask1D(&ptr, mode, xshape, yshape, zshape);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const float *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslsCorrNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const double *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vsldCorrNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex8 *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslcCorrNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex16 *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslzCorrNewTaskX(
            &ptr, mode, dims, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTaskX");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const float *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslsCorrNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslsCorrNewTaskX1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const double *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vsldCorrNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vsldCorrNewTaskX1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex8 *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslcCorrNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslcCorrNewTaskX1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex16 *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status = ::vslzCorrNewTaskX1D(
            &ptr, mode, xshape, yshape, zshape, x, xstride);
        internal::mkl_error_check(
            status, "MKLCorrTask::reset", "::vslzCorrNewTaskX1D");
        this->reset_ptr(ptr);

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
        return reset_dispatch(nx, x, xhint, ny, y, yhint);
    }

    private:
    int reset_dispatch(MKL_INT nx, const float *x, MKL_INT xhint, MKL_INT ny,
        const float *y, MKL_INT yhint)
    {
        ::DFTaskPtr ptr;
        int status = ::dfsNewTask1D(&ptr, nx, x, xhint, ny, y, yhint);
        internal::mkl_error_check(
            status, "MKLDFTask::reset", "::dfsNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(MKL_INT nx, const double *x, MKL_INT xhint, MKL_INT ny,
        const double *y, MKL_INT yhint)
    {
        ::DFTaskPtr ptr;
        int status = ::dfdNewTask1D(&ptr, nx, x, xhint, ny, y, yhint);
        internal::mkl_error_check(
            status, "MKLDFTask::reset", "::dfdNewTask1D");
        this->reset_ptr(ptr);

        return status;
    }
}; // class MKLDFTask

} // namespace vsmc

#endif // VSMC_UTILITY_MKL_HPP
