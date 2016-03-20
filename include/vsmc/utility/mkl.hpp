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

namespace vsmc
{

namespace internal
{

static constexpr int MKL_SUCCESS = 0;

#if VSMC_NO_RUNTIME_ASSERT
inline int mkl_error_check(int status, const char *, const char *)
{
    return status;
}
#else  // VSMC_NO_RUNTIME_ASSERT
inline int mkl_error_check(int status, const char *cpp, const char *c)
{
    if (status == MKL_SUCCESS)
        return status;

    std::string msg;
    msg += "**";
    msg += cpp;
    msg += "**";
    msg += " failed";
    msg += "; MKL function: ";
    msg += c;
    msg += "; Error code: ";
    msg += itos(status);

    VSMC_RUNTIME_ASSERT((status == MKL_SUCCESS), msg.c_str());

    return status;
}
#endif // VSMC_NO_RUNTIME_ASSERT

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
    explicit MKLStream(::VSLStreamStatePtr ptr = nullptr) { reset_ptr(ptr); }

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
        if (internal::mkl_error_check(::vslCopyStream(&ptr, other.get()),
                "MKLStream::MKLStream",
                "::vslCopyStream") == internal::MKL_SUCCESS) {
            reset_ptr(ptr);
        }
    }

    /// \brief `vslCopyStream`/`vslCopySreamState`
    MKLStream &operator=(const MKLStream &other)
    {
        if (this != &other) {
            ::VSLStreamStatePtr ptr = nullptr;
            if (internal::mkl_error_check(::vslCopyStream(&ptr, other.get()),
                    "MKLStream::operator=",
                    "::vslCopyStream") == internal::MKL_SUCCESS) {
                reset_ptr(ptr);
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
        int status =
            internal::mkl_error_check(::vslNewStream(&ptr, brng, seed),
                "MKLStream::reset", "::vslNewStream");
        if (status == internal::MKL_SUCCESS)
            reset_ptr(ptr);

        return status;
    }

    /// \brief `vslNewStreamEx`
    int reset(MKL_INT brng, MKL_INT n, unsigned *params)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status =
            internal::mkl_error_check(::vslNewStreamEx(&ptr, brng, n, params),
                "MKLStream::reset", "::vslNewStreamEx");
        if (status == internal::MKL_SUCCESS)
            reset_ptr(ptr);

        return status;
    }

    /// \brief `vslDeleteStream`
    static int release(::VSLStreamStatePtr ptr)
    {
        if (ptr == nullptr)
            return internal::MKL_SUCCESS;

        return internal::mkl_error_check(::vslDeleteStream(&ptr),
            "MKLStream::release", "::vslDeleteStream");
    }

    /// \brief `vslSaveStreamF`
    int save_f(const std::string &fname) const
    {
        return internal::mkl_error_check(
            ::vslSaveStreamF(get(), fname.c_str()), "MKLStream::save_f",
            "::vslSaveStreamF");
    }

    /// \brief `vslSaveStreamF`
    int load_f(const std::string &fname)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status =
            internal::mkl_error_check(::vslSaveStreamF(&ptr, fname.c_str()),
                "MKLStream::load_f", "::vslSaveStreamF");
        if (status == internal::MKL_SUCCESS)
            reset_ptr(ptr);

        return status;
    }

    /// \brief `vslSaveStreamM`
    int save_m(char *memptr) const
    {
        return internal::mkl_error_check(::vslSaveStreamM(get(), memptr),
            "MKLStream::save_m", "::vslSaveStreamM");
    }

    /// \brief `vslLoadStreamM`
    int load_m(const char *memptr)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = internal::mkl_error_check(::vslLoadStreamM(&ptr, memptr),
            "MKLStream::load_m", "::vslLoadStreamM");
        if (status == internal::MKL_SUCCESS)
            reset_ptr(ptr);

        return status;
    }

    /// \brief `vslGetStreamSize`
    int get_size() const { return ::vslGetStreamSize(get()); }

    /// \brief `vslLeapfrogStream`
    int leapfrog(MKL_INT k, MKL_INT nstreams)
    {
        return internal::mkl_error_check(
            ::vslLeapfrogStream(get(), k, nstreams), "MKLStream::leapfrog",
            "::vslLeapfrogStream");
    }

    /// \brief `vslSkipAheadStream`
    int skip_ahead(long long nskip)
    {
        return internal::mkl_error_check(::vslSkipAheadStream(get(), nskip),
            "MKLStream::skip_ahead", "::vslSkipAheadStream");
    }

    /// \brief `vslGetStreamStateBrng`
    int get_brng() const { return ::vslGetStreamStateBrng(get()); }

    /// \brief `vslGetNumRegBrngs`
    static int get_num_reg_brngs() { return ::vslGetNumRegBrngs(); }

    /// \brief `vslGetBrngProperties`
    static int get_brng_properties(
        MKL_INT brng, ::VSLBRngProperties &properties)
    {
        return internal::mkl_error_check(
            ::vslGetBrngProperties(brng, &properties),
            "MKLStream::get_brng_properties", "::vslGetBrngProperties");
    }

    /// \brief `vsRngUniform`
    int uniform(MKL_INT n, float *r, float a, float b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        return internal::mkl_error_check(
            ::vsRngUniform(method, get(), n, r, a, b), "MKLStream::uniform",
            "::vsRngUniform");
    }

    /// \brief `vdRngUniform`
    int uniform(MKL_INT n, double *r, double a, double b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        return internal::mkl_error_check(
            ::vdRngUniform(method, get(), n, r, a, b), "MKLStream::uniform",
            "::vdRngUniform");
    }

    /// \brief `vsRngGaussian`
    int gaussian(MKL_INT n, float *r, float a, float sigma,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vsRngGaussian(method, get(), n, r, a, sigma),
            "MKLStream::gaussian", "::vsRngGaussian");
    }

    /// \brief `vdRngGaussian`
    int gaussian(MKL_INT n, double *r, double a, double sigma,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vdRngGaussian(method, get(), n, r, a, sigma),
            "MKLStream::gaussian", "::vdRngGaussian");
    }

    /// \brief `vsRngGaussianMV`
    int gaussian_mv(MKL_INT n, float *r, MKL_INT dimen, MKL_INT mstorage,
        const float *a, const float *t,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vsRngGaussianMV(method, get(), n, r, dimen, mstorage, a, t),
            "MKLStream::gaussian_mv", "::vsRngGaussianMV");
    }

    /// \brief `vdRngGaussianMV`
    int gaussian_mv(MKL_INT n, double *r, MKL_INT dimen, MKL_INT mstorage,
        const double *a, const double *t,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vdRngGaussianMV(method, get(), n, r, dimen, mstorage, a, t),
            "MKLStream::gaussian_mv", "::vdRngGaussianMV");
    }

    /// \brief `vsRngExponential`
    int exponential(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngExponential(method, get(), n, r, a, beta),
            "MKLStream::exponential", "::vsRngExponential");
    }

    /// \brief `vdRngExponential`
    int exponential(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngExponential(method, get(), n, r, a, beta),
            "MKLStream::exponential", "::vdRngExponential");
    }

    /// \brief `vsRngLaplace`
    int laplace(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngLaplace(method, get(), n, r, a, beta), "MKLStream::laplace",
            "::vsRngLaplace");
    }

    /// \brief `vdRngLaplace`
    int laplace(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngLaplace(method, get(), n, r, a, beta), "MKLStream::laplace",
            "::vdRngLaplace");
    }

    /// \brief `vsRngWeibull`
    int weibull(MKL_INT n, float *r, float alpha, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngWeibull(method, get(), n, r, alpha, a, beta),
            "MKLStream::weibull", "::vsRngWeibull");
    }

    /// \brief `vdRngWeibull`
    int weibull(MKL_INT n, double *r, double alpha, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngWeibull(method, get(), n, r, alpha, a, beta),
            "MKLStream::weibull", "::vdRngWeibull");
    }

    /// \brief `vsRngCauchy`
    int cauchy(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngCauchy(method, get(), n, r, a, beta), "MKLStream::cauchy",
            "::vsRngCauchy");
    }

    /// \brief `vdRngCauchy`
    int cauchy(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngCauchy(method, get(), n, r, a, beta), "MKLStream::cauchy",
            "::vdRngCauchy");
    }

    /// \brief `vsRngRayleigh`
    int rayleigh(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngRayleigh(method, get(), n, r, a, beta),
            "MKLStream::rayleigh", "::vsRngRayleigh");
    }

    /// \brief `vdRngRayleigh`
    int rayleigh(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngRayleigh(method, get(), n, r, a, beta),
            "MKLStream::rayleigh", "::vdRngRayleigh");
    }

    /// \brief `vsRngLognormal`
    int lognormal(MKL_INT n, float *r, float a, float sigma, float b,
        float beta, MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vsRngLognormal(method, get(), n, r, a, sigma, b, beta),
            "MKLStream::lognormal", "::vsRngLognormal");
    }

    /// \brief `vdRngLognormal`
    int lognormal(MKL_INT n, double *r, double a, double sigma, double b,
        double beta, MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vdRngLognormal(method, get(), n, r, a, sigma, b, beta),
            "MKLStream::lognormal", "::vdRngLognormal");
    }

    /// \brief `vsRngGumbel`
    int gumbel(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngGumbel(method, get(), n, r, a, beta), "MKLStream::gumbel",
            "::vsRngGumbel");
    }

    /// \brief `vdRngGumbel`
    int gumbel(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngGumbel(method, get(), n, r, a, beta), "MKLStream::gumbel",
            "::vdRngGumbel");
    }

    /// \brief `vsRngGamma`
    int gamma(MKL_INT n, float *r, float alpha, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM)
    {
        return internal::mkl_error_check(
            ::vsRngGamma(method, get(), n, r, alpha, a, beta),
            "MKLStream::gamma", "::vsRngGamma");
    }

    /// \brief `vdRngGamma`
    int gamma(MKL_INT n, double *r, double alpha, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM)
    {
        return internal::mkl_error_check(
            ::vdRngGamma(method, get(), n, r, alpha, a, beta),
            "MKLStream::gamma", "::vdRngGamma");
    }

    /// \brief `vsRngBeta`
    int beta(MKL_INT n, float *r, float p, float q, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_BETA_CJA)
    {
        return internal::mkl_error_check(
            ::vsRngBeta(method, get(), n, r, p, q, a, beta), "MKLStream::beta",
            "::vsRngBeta");
    }

    /// \brief `vdRngBeta`
    int beta(MKL_INT n, double *r, double p, double q, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_BETA_CJA)
    {
        return internal::mkl_error_check(
            ::vdRngBeta(method, get(), n, r, p, q, a, beta), "MKLStream::beta",
            "::vdRngBeta");
    }

    /// \brief `viRngUniform`
    int uniform(MKL_INT n, int *r, int a, int b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniform(method, get(), n, r, a, b), "MKLStream::uniform",
            "::viRngUniform");
    }

    /// \brief `viRngUniform`
    int uniform_bits(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniformBits(method, get(), n, r), "MKLStream::uniform_bits",
            "::viRngUniformBits");
    }

    /// \brief `viRngUniform32`
    int uniform_bits32(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS32_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniformBits32(method, get(), n, r),
            "MKLStream::uniform_bits32", "::viRngUniformBits32");
    }

    /// \brief `viRngUniform64`
    int uniform_bits64(MKL_INT n, unsigned MKL_INT64 *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS64_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniformBits64(method, get(), n, r),
            "MKLStream::uniform_bits64", "::viRngUniformBits64");
    }

    /// \brief `viRngBernoulli`
    int bernoulli(MKL_INT n, int *r, double p,
        MKL_INT method = VSL_RNG_METHOD_BERNOULLI_ICDF)
    {
        return internal::mkl_error_check(
            ::viRngBernoulli(method, get(), n, r, p), "MKLStream::bernoulli",
            "::viRngBernoulli");
    }

    /// \brief `viRngGeometric`
    int geometric(MKL_INT n, int *r, double p,
        MKL_INT method = VSL_RNG_METHOD_GEOMETRIC_ICDF)
    {
        return internal::mkl_error_check(
            ::viRngGeometric(method, get(), n, r, p), "MKLStream::geometric",
            "::viRngGeometric");
    }

    /// \brief `viRngBinomial`
    int binomial(MKL_INT n, int *r, int ntrial, double p,
        MKL_INT method = VSL_RNG_METHOD_BINOMIAL_BTPE)
    {
        return internal::mkl_error_check(
            ::viRngBinomial(method, get(), n, r, ntrial, p),
            "MKLStream::binomial", "::viRngBinomial");
    }

    /// \brief `viRngHypergeometric`
    int hypergeometric(MKL_INT n, int *r, int l, int s, int m,
        MKL_INT method = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE)
    {
        return internal::mkl_error_check(
            ::viRngHypergeometric(method, get(), n, r, l, s, m),
            "MKLStream::hypergeometric", "::viRngHypergeometric");
    }

    /// \brief `viRngPoisson`
    int poisson(MKL_INT n, int *r, double lambda,
        MKL_INT method = VSL_RNG_METHOD_POISSON_PTPE)
    {
        return internal::mkl_error_check(
            ::viRngPoisson(method, get(), n, r, lambda), "MKLStream::poisson",
            "::viRngPoisson");
    }

    /// \brief `viRngPoissonV`
    int poisson_v(MKL_INT n, int *r, const double *lambda,
        MKL_INT method = VSL_RNG_METHOD_POISSONV_POISNORM)
    {
        return internal::mkl_error_check(
            ::viRngPoissonV(method, get(), n, r, lambda),
            "MKLStream::poisson_v", "::viRngPoissonV");
    }

    /// \brief `viRngNegbinomial`
    int neg_binomial(MKL_INT n, int *r, double a, double p,
        MKL_INT method = VSL_RNG_METHOD_NEGBINOMIAL_NBAR)
    {
        return internal::mkl_error_check(
            ::viRngNegbinomial(method, get(), n, r, a, p),
            "MKLStream::neg_binomial", "::viRngNegbinomial");
    }
}; // class MKLStream

/// \brief MKL `VSLSSTaskPtr`
/// \ingroup MKL
template <typename RealType = double>
class MKLSSTask : public MKLBase<::VSLSSTaskPtr, MKLSSTask<RealType>>
{
    static_assert(internal::is_one_of<RealType, float, double>::value,
        "**MKLSSTask** USED WITH RealType OTHER THAN float or double");

    public:
    using result_type = RealType;

    explicit MKLSSTask(::VSLSSTaskPtr ptr = nullptr) { this->reset_ptr(ptr); }

    /// \brief `vslSSNewTask`
    MKLSSTask(const MKL_INT *p, const MKL_INT *n, const MKL_INT *xstorage,
        const result_type *x, const result_type *w, const MKL_INT *indices)
    {
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
            return internal::MKL_SUCCESS;

        return internal::mkl_error_check(::vslSSDeleteTask(&ptr),
            "MKLSSTask::release", "::vslSSDeleteTask");
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
        int status = internal::mkl_error_check(
            ::vslsSSNewTask(&ptr, p, n, xstorage, x, w, indices),
            "MKLSSTask::reset", "::vslsSSNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT *p, const MKL_INT *n,
        const MKL_INT *xstorage, const double *x, const double *w,
        const MKL_INT *indices)
    {
        ::VSLSSTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vsldSSNewTask(&ptr, p, n, xstorage, x, w, indices),
            "MKLSSTask::reset", "::vsldSSNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int edit_task_dispatch(MKL_INT parameter, const float *par_addr)
    {
        return internal::mkl_error_check(
            ::vslsSSEditTask(this->get(), parameter, par_addr),
            "MKLSSTask::edit_task", "::vslsSSEditTask");
    }

    int edit_task_dispatch(MKL_INT parameter, const double *par_addr)
    {
        return internal::mkl_error_check(
            ::vsldSSEditTask(this->get(), parameter, par_addr),
            "MKLSSTask::edit_task", "::vsldSSEditTask");
    }

    int edit_task_dispatch(MKL_INT parameter, const MKL_INT *par_addr)
    {
        return internal::mkl_error_check(
            ::vsliSSEditTask(this->get(), parameter, par_addr),
            "MKLSSTask::edit_task", "::vsliSSEditTask");
    }

    int edit_moments_dispatch(const float *mean, const float *r2m,
        const float *r3m, const float *r4m, const float *c2m, const float *c3m,
        const float *c4m)
    {
        return internal::mkl_error_check(::vslsSSEditMoments(this->get(), mean,
                                             r2m, r3m, r4m, c2m, c3m, c4m),
            "MKLSSTask::edit_moments", "::vslsSSEditMoments");
    }

    int edit_moments_dispatch(const double *mean, const double *r2m,
        const double *r3m, const double *r4m, const double *c2m,
        const double *c3m, const double *c4m)
    {
        return internal::mkl_error_check(::vsldSSEditMoments(this->get(), mean,
                                             r2m, r3m, r4m, c2m, c3m, c4m),
            "MKLSSTask::edit_moments", "::vsldSSEditMoments");
    }

    int edit_sums_dispatch(const float *sum, const float *r2s,
        const float *r3s, const float *r4s, const float *c2s, const float *c3s,
        const float *c4s)
    {
        return internal::mkl_error_check(
            ::vslsSSEditSums(this->get(), sum, r2s, r3s, r4s, c2s, c3s, c4s),
            "MKLSSTask::edit_sums", "::vslsSSEditSums");
    }

    int edit_sums_dispatch(const double *sum, const double *r2s,
        const double *r3s, const double *r4s, const double *c2s,
        const double *c3s, const double *c4s)
    {
        return internal::mkl_error_check(
            ::vsldSSEditSums(this->get(), sum, r2s, r3s, r4s, c2s, c3s, c4s),
            "MKLSSTask::edit_sums", "::vsldSSEditSums");
    }

    int edit_cov_cor_dispatch(const float *mean, const float *cov,
        const MKL_INT *cov_storage, const float *cor,
        const MKL_INT *cor_storage)
    {
        return internal::mkl_error_check(
            ::vslsSSEditCovCor(
                this->get(), mean, cov, cov_storage, cor, cor_storage),
            "MKLSSTask::edit_cov_cor", "::vslsSSEditCovCor");
    }

    int edit_cov_cor_dispatch(const double *mean, const double *cov,
        const MKL_INT *cov_storage, const double *cor,
        const MKL_INT *cor_storage)
    {
        return internal::mkl_error_check(
            ::vsldSSEditCovCor(
                this->get(), mean, cov, cov_storage, cor, cor_storage),
            "MKLSSTask::edit_cov_cor", "::vsldSSEditCovCor");
    }

    int edit_cp_dispatch(const float *mean, const float *sum, const float *cp,
        const MKL_INT *cp_storage)
    {
        return internal::mkl_error_check(
            ::vslsSSEditCP(this->get(), mean, sum, cp, cp_storage),
            "MKLSSTask::edit_cp", "::vslsSSEditCP");
    }

    int edit_cp_dispatch(const double *mean, const double *sum,
        const double *cp, const MKL_INT *cp_storage)
    {
        return internal::mkl_error_check(
            ::vsldSSEditCP(this->get(), mean, sum, cp, cp_storage),
            "MKLSSTask::edit_cp", "::vsldSSEditCP");
    }

    int edit_partial_cov_cor_dispatch(const MKL_INT *p_idx_array,
        const float *cov, const MKL_INT *cov_storage, const float *cor,
        const MKL_INT *cor_storage, const float *p_cov,
        const MKL_INT *p_cov_storage, const float *p_cor,
        const MKL_INT *p_cor_storage) const
    {
        return internal::mkl_error_check(
            ::vslsSSEditPartialCovCor(this->get(), p_idx_array, cov,
                cov_storage, cor, cor_storage, p_cov, p_cov_storage, p_cor,
                p_cor_storage),
            "MKLSSTask::edit_partial_cov_cor", "::vslsSSEditPartialCovCor");
    }

    int edit_partial_cov_cor_dispatch(const MKL_INT *p_idx_array,
        const double *cov, const MKL_INT *cov_storage, const double *cor,
        const MKL_INT *cor_storage, const double *p_cov,
        const MKL_INT *p_cov_storage, const double *p_cor,
        const MKL_INT *p_cor_storage) const
    {
        return internal::mkl_error_check(
            ::vsldSSEditPartialCovCor(this->get(), p_idx_array, cov,
                cov_storage, cor, cor_storage, p_cov, p_cov_storage, p_cor,
                p_cor_storage),
            "MKLSSTask::edit_partial_cov_cor", "::vsldSSEditPartialCovCor");
    }

    int edit_quantiles_dispatch(const MKL_INT *quant_order_n,
        const float *quant_order, const float *quant, const float *order_stats,
        const MKL_INT *order_stats_storage)
    {
        return internal::mkl_error_check(
            ::vslsSSEditQuantiles(this->get(), quant_order_n, quant_order,
                quant, order_stats, order_stats_storage),
            "MKLSSTask::edit_quantiles", "::vslsSSEditQuantiles");
    }

    int edit_quantiles_dispatch(const MKL_INT *quant_order_n,
        const double *quant_order, const double *quant,
        const double *order_stats, const MKL_INT *order_stats_storage)
    {
        return internal::mkl_error_check(
            ::vsldSSEditQuantiles(this->get(), quant_order_n, quant_order,
                quant, order_stats, order_stats_storage),
            "MKLSSTask::edit_quantiles", "::vsldSSEditQuantiles");
    }

    int edit_stream_quantiles_dispatch(const MKL_INT *quant_order_n,
        const float *quant_order, const float *quants, const MKL_INT *nparams,
        const float *params)
    {
        return internal::mkl_error_check(
            ::vslsSSEditStreamQuantiles(this->get(), quant_order_n,
                quant_order, quants, nparams, params),
            "MKLSSTask::edit_stream_quantiles", "::vslsSSEditStreamQuantiles");
    }

    int edit_stream_quantiles_dispatch(const MKL_INT *quant_order_n,
        const double *quant_order, const double *quants,
        const MKL_INT *nparams, const double *params)
    {
        return internal::mkl_error_check(
            ::vsldSSEditStreamQuantiles(this->get(), quant_order_n,
                quant_order, quants, nparams, params),
            "MKLSSTask::edit_stream_quantiles", "::vsldSSEditStreamQuantiles");
    }

    int edit_pooled_covariance_dispatch(const MKL_INT *grp_indices,
        const float *pld_mean, const float *pld_cov,
        const MKL_INT *req_grp_indices, const float *grp_means,
        const float *grp_cov)
    {
        return internal::mkl_error_check(
            ::vslsSSEditPooledCovariance(this->get(), grp_indices, pld_mean,
                pld_cov, req_grp_indices, grp_means, grp_cov),
            "MKLSSTask::edit_pooled_covariance",
            "::vslsSSEditPooledCovariance");
    }

    int edit_pooled_covariance_dispatch(const MKL_INT *grp_indices,
        const double *pld_mean, const double *pld_cov,
        const MKL_INT *req_grp_indices, const double *grp_means,
        const double *grp_cov)
    {
        return internal::mkl_error_check(
            ::vsldSSEditPooledCovariance(this->get(), grp_indices, pld_mean,
                pld_cov, req_grp_indices, grp_means, grp_cov),
            "MKLSSTask::edit_pooled_covariance",
            "::vsldSSEditPooledCovariance");
    }

    int edit_robust_covariance_dispatch(const MKL_INT *rcov_storage,
        const MKL_INT *nparams, const float *params, const float *rmean,
        const float *rcov)
    {
        return internal::mkl_error_check(
            ::vslsSSEditRobustCovariance(
                this->get(), rcov_storage, nparams, params, rmean, rcov),
            "MKLSSTask::edit_robust_covariance",
            "::vslsSSEditRobustCovariance");
    }

    int edit_robust_covariance_dispatch(const MKL_INT *rcov_storage,
        const MKL_INT *nparams, const double *params, const double *rmean,
        const double *rcov)
    {
        return internal::mkl_error_check(
            ::vsldSSEditRobustCovariance(
                this->get(), rcov_storage, nparams, params, rmean, rcov),
            "MKLSSTask::edit_robust_covariance",
            "::vsldSSEditRobustCovariance");
    }

    int edit_outliers_detection_dispatch(
        const MKL_INT *nparams, const float *params, const float *w)
    {
        return internal::mkl_error_check(
            ::vslsSSEditOutliersDetection(this->get(), nparams, params, w),
            "MKLSSTask::edit_outliers_detection",
            "::vslsSSEditOutliersDetection");
    }

    int edit_outliers_detection_dispatch(
        const MKL_INT *nparams, const double *params, const double *w)
    {
        return internal::mkl_error_check(
            ::vsldSSEditOutliersDetection(this->get(), nparams, params, w),
            "MKLSSTask::edit_outliers_detection",
            "::vsldSSEditOutliersDetection");
    }

    int edit_missing_values_dispatch(const MKL_INT *nparams,
        const float *params, const MKL_INT *init_estimates_n,
        const float *init_estimates, const MKL_INT *prior_n,
        const float *prior, const MKL_INT *simul_missing_vals_n,
        const float *simul_missing_vals, const MKL_INT *estimates_n,
        const float *estimates)
    {
        return internal::mkl_error_check(
            ::vslsSSEditMissingValues(this->get(), nparams, params,
                init_estimates_n, init_estimates, prior_n, prior,
                simul_missing_vals_n, simul_missing_vals, estimates_n,
                estimates),
            "MKLSSTask::edit_missing_values", "::vslsSSEditMissingValues");
    }

    int edit_missing_values_dispatch(const MKL_INT *nparams,
        const double *params, const MKL_INT *init_estimates_n,
        const double *init_estimates, const MKL_INT *prior_n,
        const double *prior, const MKL_INT *simul_missing_vals_n,
        const double *simul_missing_vals, const MKL_INT *estimates_n,
        const double *estimates)
    {
        return internal::mkl_error_check(
            ::vsldSSEditMissingValues(this->get(), nparams, params,
                init_estimates_n, init_estimates, prior_n, prior,
                simul_missing_vals_n, simul_missing_vals, estimates_n,
                estimates),
            "MKLSSTask::edit_missing_values", "::vsldSSEditMissingValues");
    }

    int edit_cor_parameterization_dispatch(const float *cor,
        const MKL_INT *cor_storage, const float *pcor,
        const MKL_INT *pcor_storage)
    {
        return internal::mkl_error_check(
            ::vslsSSEditCorParameterization(
                this->get(), cor, cor_storage, pcor, pcor_storage),
            "MKLSSTask::edit_cor_parameterization",
            "::vslsSSEditCorParameterization");
    }

    int edit_cor_parameterization_dispatch(const double *cor,
        const MKL_INT *cor_storage, const double *pcor,
        const MKL_INT *pcor_storage)
    {
        return internal::mkl_error_check(
            ::vsldSSEditCorParameterization(
                this->get(), cor, cor_storage, pcor, pcor_storage),
            "MKLSSTask::edit_cor_parameterization",
            "::vsldSSEditCorParameterization");
    }

    int compute_dispatch(
        unsigned MKL_INT64 estimates, MKL_INT method, const float *)
    {
        return internal::mkl_error_check(
            ::vslsSSCompute(this->get(), estimates, method),
            "MKLSSTask::compute", "::vslsSSCompute");
    }

    int compute_dispatch(
        unsigned MKL_INT64 estimates, MKL_INT method, const double *)
    {
        return internal::mkl_error_check(
            ::vsldSSCompute(this->get(), estimates, method),
            "MKLSSTask::compute", "::vsldSSCompute");
    }
}; // class MKLSSTask

/// \brief MKL `VSLConvTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLConvTask : public MKLBase<::VSLConvTaskPtr, MKLConvTask<ResultType>>
{
    static_assert(internal::is_one_of<ResultType, float, double, MKL_Complex8,
                      MKL_Complex16>::value,
        "**MKLConvTask** USED WITH ResultType OTHER THAN float, double, "
        "MKL_Complex8 OR MKL_Complex16");

    public:
    using result_type = ResultType;

    explicit MKLConvTask(::VSLConvTaskPtr ptr = nullptr)
    {
        this->reset_ptr(ptr);
    }

    /// \brief `vslConvNewTask`
    MKLConvTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        reset(mode, dims, xshape, yshape, zshape);
    }

    /// \brief `vslConvNewTask1D`
    MKLConvTask(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        reset(mode, xshape, yshape, zshape);
    }

    /// \brief `vslConvNewTaskX`
    MKLConvTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        reset(mode, dims, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslConvNewTaskX1D`
    MKLConvTask(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        reset(mode, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslConvCopyTask`
    MKLConvTask(const MKLConvTask<ResultType> &other)
    {
        ::VSLConvTaskPtr ptr = nullptr;
        if (internal::mkl_error_check(::vslConvCopyTask(&ptr, other.get()),
                "MKLConvTask::MKLConvTask",
                "::vslConvCopyTask") == internal::MKL_SUCCESS) {
            this->reset_ptr(ptr);
        }
    }

    /// \brief `vslConvCopyTask`
    MKLConvTask<ResultType> &operator=(const MKLConvTask<ResultType> &other)
    {
        if (this != &other) {
            ::VSLConvTaskPtr ptr = nullptr;
            if (internal::mkl_error_check(::vslConvCopyTask(&ptr, other.get()),
                    "MKLConvTask::operator=",
                    "::vslConvCopyTask") == internal::MKL_SUCCESS) {
                this->reset_ptr(ptr);
            }
        }

        return *this;
    }

    MKLConvTask(MKLConvTask<ResultType> &&) = default;
    MKLConvTask<ResultType> &operator=(MKLConvTask<ResultType> &&) = default;

    /// \brief `vslConvDeleteTask`
    static int release(::VSLConvTaskPtr ptr)
    {
        if (ptr == nullptr)
            return internal::MKL_SUCCESS;

        return internal::mkl_error_check(::vslConvDeleteTask(&ptr),
            "MKLConvTask::release", "::vslConvDeleteTask");
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
        int status = internal::mkl_error_check(
            ::vslsConvNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vslsConvNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, double *)
    {
        ::VSLConvTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vsldConvNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vsldConvNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex8 *)
    {
        ::VSLConvTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslcConvNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vslcConvNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex16 *)
    {
        ::VSLConvTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslzConvNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vslzConvNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, float *)
    {
        ::VSLConvTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslsConvNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vslsConvNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, double *)
    {
        ::VSLConvTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vsldConvNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vsldConvNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex8 *)
    {
        ::VSLConvTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslcConvNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vslcConvNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex16 *)
    {
        ::VSLConvTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslzConvNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLConvTask::reset", "::vslzConvNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const float *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslsConvNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vslsConvNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const double *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vsldConvNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vsldConvNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex8 *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslcConvNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vslcConvNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex16 *x,
        const MKL_INT *xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslzConvNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vslzConvNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const float *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslsConvNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vslsConvNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const double *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vsldConvNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vsldConvNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex8 *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslcConvNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vslcConvNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex16 *x,
        const MKL_INT xstride)
    {
        ::VSLConvTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslzConvNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLConvTask::reset", "::vslzConvNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }
}; // class MKLConvTask

/// \brief MKL `VSLCorrTaskPtr`
/// \ingroup MKL
template <typename ResultType = double>
class MKLCorrTask : public MKLBase<::VSLCorrTaskPtr, MKLCorrTask<ResultType>>
{
    static_assert(internal::is_one_of<ResultType, float, double, MKL_Complex8,
                      MKL_Complex16>::value,
        "**MKLCorrTask** USED WITH ResultType OTHER THAN float, double, "
        "MKL_Complex8 OR MKL_Complex16");

    public:
    using result_type = ResultType;

    explicit MKLCorrTask(::VSLCorrTaskPtr ptr = nullptr)
    {
        this->reset_ptr(ptr);
    }

    /// \brief `vslCorrNewTask`
    MKLCorrTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape)
    {
        reset(mode, dims, xshape, yshape, zshape);
    }

    /// \brief `vslCorrNewTask1D`
    MKLCorrTask(
        MKL_INT mode, const MKL_INT xshape, MKL_INT yshape, MKL_INT zshape)
    {
        reset(mode, xshape, yshape, zshape);
    }

    /// \brief `vslCorrNewTaskX`
    MKLCorrTask(MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const result_type *x,
        const MKL_INT *xstride)
    {
        reset(mode, dims, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslCorrNewTaskX1D`
    MKLCorrTask(MKL_INT mode, MKL_INT xshape, MKL_INT yshape, MKL_INT zshape,
        const result_type *x, const MKL_INT xstride)
    {
        reset(mode, xshape, yshape, zshape, x, xstride);
    }

    /// \brief `vslCorrCopyTask`
    MKLCorrTask(const MKLCorrTask<ResultType> &other)
    {
        ::VSLCorrTaskPtr ptr = nullptr;
        if (internal::mkl_error_check(::vslCorrCopyTask(&ptr, other.get()),
                "MKLCorrTask::MKLCorrTask",
                "::vslCorrCopyTask") == internal::MKL_SUCCESS) {
            this->reset_ptr(ptr);
        }
    }

    /// \brief `vslCorrCopyTask`
    MKLCorrTask<ResultType> &operator=(const MKLCorrTask<ResultType> &other)
    {
        if (this != &other) {
            ::VSLCorrTaskPtr ptr = nullptr;
            if (internal::mkl_error_check(::vslCorrCopyTask(&ptr, other.get()),
                    "MKLCorrTask::operator=",
                    "::vslCorrCopyTask") == internal::MKL_SUCCESS) {
                this->reset_ptr(ptr);
            }
        }

        return *this;
    }

    MKLCorrTask(MKLCorrTask<ResultType> &&) = default;
    MKLCorrTask<ResultType> &operator=(MKLCorrTask<ResultType> &&) = default;

    /// \brief `vslCorrDeleteTask`
    static int release(::VSLCorrTaskPtr ptr)
    {
        if (ptr == nullptr)
            return internal::MKL_SUCCESS;

        return internal::mkl_error_check(::vslCorrDeleteTask(&ptr),
            "MKLCorrTask::release", "::vslCorrDeleteTask");
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
        int status = internal::mkl_error_check(
            ::vslsCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vslsCorrNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, double *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vsldCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vsldCorrNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex8 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslcCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vslcCorrNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, MKL_Complex16 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslzCorrNewTask(&ptr, mode, dims, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vslzCorrNewTask");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, float *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslsCorrNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vslsCorrNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, double *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vsldCorrNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vsldCorrNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex8 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslcCorrNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vslcCorrNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, MKL_Complex16 *)
    {
        ::VSLCorrTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::vslzCorrNewTask1D(&ptr, mode, xshape, yshape, zshape),
            "MKLCorrTask::reset", "::vslzCorrNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const float *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslsCorrNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vslsCorrNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const double *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vsldCorrNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vsldCorrNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex8 *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslcCorrNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vslcCorrNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, MKL_INT dims, const MKL_INT *xshape,
        const MKL_INT *yshape, const MKL_INT *zshape, const MKL_Complex16 *x,
        const MKL_INT *xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslzCorrNewTaskX(&ptr, mode, dims,
                                          xshape, yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vslzCorrNewTaskX");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const float *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslsCorrNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vslsCorrNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const double *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vsldCorrNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vsldCorrNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex8 *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslcCorrNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vslcCorrNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(const MKL_INT mode, const MKL_INT xshape,
        const MKL_INT yshape, const MKL_INT zshape, const MKL_Complex16 *x,
        const MKL_INT xstride)
    {
        ::VSLCorrTaskPtr ptr;
        int status =
            internal::mkl_error_check(::vslzCorrNewTaskX1D(&ptr, mode, xshape,
                                          yshape, zshape, x, xstride),
                "MKLCorrTask::reset", "::vslzCorrNewTaskX1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }
}; // class MKLCorrTask

/// \brief MKL `DFTaskPtr`
/// \ingroup MKL
template <typename RealType = double>
class MKLDFTask : public MKLBase<::DFTaskPtr, MKLDFTask<RealType>>
{
    static_assert(internal::is_one_of<RealType, float, double>::value,
        "**MKLDFTask** USED WITH RealType OTHER THAN float OR double");

    public:
    using result_type = RealType;

    explicit MKLDFTask(::DFTaskPtr ptr = nullptr) { this->reset_ptr(ptr); }

    MKLDFTask(MKL_INT nx, const result_type *x, MKL_INT xhint, MKL_INT ny,
        const result_type *y, MKL_INT yhint)
    {
        reset(nx, x, xhint, ny, y, yhint);
    }

    static int release(::DFTaskPtr ptr)
    {
        if (ptr == nullptr)
            return internal::MKL_SUCCESS;

        return internal::mkl_error_check(
            ::dfDeleteTask(&ptr), "MKLDFTask::release", "::dfDeleteTask");
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
        int status = internal::mkl_error_check(
            ::dfsNewTask1D(&ptr, nx, x, xhint, ny, y, yhint),
            "MKLDFTask::reset", "::dfsNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }

    int reset_dispatch(MKL_INT nx, const double *x, MKL_INT xhint, MKL_INT ny,
        const double *y, MKL_INT yhint)
    {
        ::DFTaskPtr ptr;
        int status = internal::mkl_error_check(
            ::dfdNewTask1D(&ptr, nx, x, xhint, ny, y, yhint),
            "MKLDFTask::reset", "::dfdNewTask1D");
        if (status == internal::MKL_SUCCESS)
            this->reset_ptr(ptr);

        return status;
    }
}; // class MKLDFTask

} // namespace vsmc

#endif // VSMC_UTILITY_MKL_HPP
