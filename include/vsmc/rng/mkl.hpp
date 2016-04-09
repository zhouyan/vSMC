//============================================================================
// vSMC/include/vsmc/rng/mkl.hpp
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

#ifndef VSMC_RNG_MKL_HPP
#define VSMC_RNG_MKL_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/uniform_real_distribution.hpp>
#include <mkl.h>

#define VSMC_RUNTIME_WARNING_RNG_MKL_OFFSET                                   \
    VSMC_RUNTIME_WARNING((offset < MaxOffset),                                \
        "**MKLEngine** EXCESS MAXIMUM NUMBER OF INDEPDENT RNG STREAMS")

#define VSMC_RUNTIME_ASSERT_RNG_MKL_DISCARD                                   \
    VSMC_RUNTIME_ASSERT(                                                      \
        (nskip >= 0), "**MKLEngine::discard** INPUT IS NEGATIVE")

namespace vsmc
{

namespace internal
{

#if VSMC_NO_RUNTIME_ASSERT
inline int mkl_error_check(int status, const char *, const char *)
{
    return status;
}
#else  // VSMC_NO_RUNTIME_ASSERT
inline int mkl_error_check(int status, const char *cpp, const char *c)
{
    if (status == VSL_ERROR_OK)
        return status;

    std::string msg;
    msg += "**";
    msg += cpp;
    msg += "**";
    msg += " failed";
    msg += "; MKL function: ";
    msg += c;
    msg += "; Error code: ";
    msg += std::to_string(status);

    VSMC_RUNTIME_ASSERT((status == VSL_ERROR_OK), msg.c_str());

    return status;
}
#endif // VSMC_NO_RUNTIME_ASSERT

} // namespace vsmc::internal

/// \brief MKL `VSLStreamStatePtr` wrapper
/// \ingroup MKLRNG
class MKLStream
{
    public:
    explicit MKLStream(::VSLStreamStatePtr ptr = nullptr) : ptr_(nullptr)
    {
        reset(ptr);
    }

    /// \brief `vslNewStream`
    MKLStream(MKL_INT brng, MKL_UINT seed) : ptr_(nullptr)
    {
        reset(brng, seed);
    }

    /// \brief `vslNewStreamEx`
    MKLStream(MKL_INT brng, MKL_INT n, unsigned *params) : ptr_(nullptr)
    {
        reset(brng, n, params);
    }

    /// \brief `vslCopyStream`
    MKLStream(const MKLStream &other)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        if (internal::mkl_error_check(::vslCopyStream(&ptr, other.ptr_),
                "MKLStream::MKLStream", "::vslCopyStream") == VSL_ERROR_OK) {
            reset(ptr);
        }
    }

    /// \brief `vslCopyStream`/`vslCopySreamState`
    MKLStream &operator=(const MKLStream &other)
    {
        if (this != &other) {
            ::VSLStreamStatePtr ptr = nullptr;
            if (internal::mkl_error_check(::vslCopyStream(&ptr, other.ptr_),
                    "MKLStream::operator=",
                    "::vslCopyStream") == VSL_ERROR_OK) {
                reset(ptr);
            }
        }

        return *this;
    }

    MKLStream(MKLStream &&other) : ptr_(other.ptr_) { other.ptr_ = nullptr; }

    MKLStream &operator=(MKLStream &&other)
    {
        if (this != &other) {
            release();
            ptr_ = other.ptr_;
            other.ptr_ = nullptr;
        }

        return *this;
    }

    /// \brief `vslDeleteStream`
    ~MKLStream() { release(); }

    int reset(::VSLStreamStatePtr ptr)
    {
        int status = release();
        ptr_ = ptr;

        return status;
    }

    /// \brief `vslNewStream`
    int reset(MKL_INT brng, MKL_UINT seed)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status =
            internal::mkl_error_check(::vslNewStream(&ptr, brng, seed),
                "MKLStream::reset", "::vslNewStream");
        if (status == VSL_ERROR_OK)
            reset(ptr);

        return status;
    }

    /// \brief `vslNewStreamEx`
    int reset(MKL_INT brng, MKL_INT n, unsigned *params)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status =
            internal::mkl_error_check(::vslNewStreamEx(&ptr, brng, n, params),
                "MKLStream::reset", "::vslNewStreamEx");
        if (status == VSL_ERROR_OK)
            reset(ptr);

        return status;
    }

    /// \brief `vslDeleteStream`
    int release()
    {
        if (ptr_ == nullptr)
            return VSL_ERROR_OK;

        int status = internal::mkl_error_check(::vslDeleteStream(&ptr_),
            "MKLStream::release", "::vslDeleteStream");
        ptr_ = nullptr;

        return status;
    }

    bool empty() const { return ptr_ == nullptr; }

    /// \brief `vslSaveStreamF`
    int save_f(const std::string &fname) const
    {
        return internal::mkl_error_check(::vslSaveStreamF(ptr_, fname.c_str()),
            "MKLStream::save_f", "::vslSaveStreamF");
    }

    /// \brief `vslSaveStreamF`
    int load_f(const std::string &fname)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status =
            internal::mkl_error_check(::vslSaveStreamF(&ptr, fname.c_str()),
                "MKLStream::load_f", "::vslSaveStreamF");
        if (status == VSL_ERROR_OK)
            reset(ptr);

        return status;
    }

    /// \brief `vslSaveStreamM`
    int save_m(char *memptr) const
    {
        return internal::mkl_error_check(::vslSaveStreamM(ptr_, memptr),
            "MKLStream::save_m", "::vslSaveStreamM");
    }

    /// \brief `vslLoadStreamM`
    int load_m(const char *memptr)
    {
        ::VSLStreamStatePtr ptr = nullptr;
        int status = internal::mkl_error_check(::vslLoadStreamM(&ptr, memptr),
            "MKLStream::load_m", "::vslLoadStreamM");
        if (status == VSL_ERROR_OK)
            reset(ptr);

        return status;
    }

    /// \brief `vslGetStreamSize`
    int get_size() const { return ::vslGetStreamSize(ptr_); }

    /// \brief `vslLeapfrogStream`
    int leapfrog(MKL_INT k, MKL_INT nstreams)
    {
        return internal::mkl_error_check(
            ::vslLeapfrogStream(ptr_, k, nstreams), "MKLStream::leapfrog",
            "::vslLeapfrogStream");
    }

    /// \brief `vslSkipAheadStream`
    int skip_ahead(long long nskip)
    {
        return internal::mkl_error_check(::vslSkipAheadStream(ptr_, nskip),
            "MKLStream::skip_ahead", "::vslSkipAheadStream");
    }

    /// \brief `vslGetStreamStateBrng`
    int get_brng() const { return ::vslGetStreamStateBrng(ptr_); }

    /// \brief `vslGetNumRegBrngs`
    static int get_num_reg_brngs() { return ::vslGetNumRegBrngs(); }

    /// \brief `vslGetBrngProperties`
    static int get_brng_properties(
        MKL_INT brng, ::VSLBRngProperties *properties)
    {
        return internal::mkl_error_check(
            ::vslGetBrngProperties(brng, properties),
            "MKLStream::get_brng_properties", "::vslGetBrngProperties");
    }

    /// \brief Test if `vslLeapfrogStream` is supported
    static bool has_leap_frog(MKL_INT brng)
    {
        MKLStream stream(brng, 1);

        return ::vslLeapfrogStream(stream.ptr_, 1, 2) == VSL_ERROR_OK;
    }

    /// \brief Test if `vslSkipAheadStream` is supported
    static bool has_skip_ahead(MKL_INT brng)
    {
        MKLStream stream(brng, 1);

        return ::vslSkipAheadStream(stream.ptr_, 1) == VSL_ERROR_OK;
    }

    /// \brief Test if `viRngUniformBits32` is supported
    static bool has_uniform_bits32(
        MKL_INT brng, MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS32_STD)
    {
        MKLStream stream(brng, 1);
        unsigned r;

        return ::viRngUniformBits32(method, stream.ptr_, 1, &r) ==
            VSL_ERROR_OK;
    }

    /// \brief Test if `viRngUniformBits64` is supported
    static bool has_uniform_bits64(
        MKL_INT brng, MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS64_STD)
    {
        MKLStream stream(brng, 1);
        unsigned MKL_INT64 r;

        return ::viRngUniformBits64(method, stream.ptr_, 1, &r) ==
            VSL_ERROR_OK;
    }

    /// \brief `vsRngUniform`
    int uniform(MKL_INT n, float *r, float a, float b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        return internal::mkl_error_check(
            ::vsRngUniform(method, ptr_, n, r, a, b), "MKLStream::uniform",
            "::vsRngUniform");
    }

    /// \brief `vdRngUniform`
    int uniform(MKL_INT n, double *r, double a, double b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        return internal::mkl_error_check(
            ::vdRngUniform(method, ptr_, n, r, a, b), "MKLStream::uniform",
            "::vdRngUniform");
    }

    /// \brief `vsRngGaussian`
    int gaussian(MKL_INT n, float *r, float a, float sigma,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vsRngGaussian(method, ptr_, n, r, a, sigma),
            "MKLStream::gaussian", "::vsRngGaussian");
    }

    /// \brief `vdRngGaussian`
    int gaussian(MKL_INT n, double *r, double a, double sigma,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIAN_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vdRngGaussian(method, ptr_, n, r, a, sigma),
            "MKLStream::gaussian", "::vdRngGaussian");
    }

    /// \brief `vsRngGaussianMV`
    int gaussian_mv(MKL_INT n, float *r, MKL_INT dimen, MKL_INT mstorage,
        const float *a, const float *t,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vsRngGaussianMV(method, ptr_, n, r, dimen, mstorage, a, t),
            "MKLStream::gaussian_mv", "::vsRngGaussianMV");
    }

    /// \brief `vdRngGaussianMV`
    int gaussian_mv(MKL_INT n, double *r, MKL_INT dimen, MKL_INT mstorage,
        const double *a, const double *t,
        MKL_INT method = VSL_RNG_METHOD_GAUSSIANMV_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vdRngGaussianMV(method, ptr_, n, r, dimen, mstorage, a, t),
            "MKLStream::gaussian_mv", "::vdRngGaussianMV");
    }

    /// \brief `vsRngExponential`
    int exponential(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngExponential(method, ptr_, n, r, a, beta),
            "MKLStream::exponential", "::vsRngExponential");
    }

    /// \brief `vdRngExponential`
    int exponential(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_EXPONENTIAL_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngExponential(method, ptr_, n, r, a, beta),
            "MKLStream::exponential", "::vdRngExponential");
    }

    /// \brief `vsRngLaplace`
    int laplace(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngLaplace(method, ptr_, n, r, a, beta), "MKLStream::laplace",
            "::vsRngLaplace");
    }

    /// \brief `vdRngLaplace`
    int laplace(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_LAPLACE_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngLaplace(method, ptr_, n, r, a, beta), "MKLStream::laplace",
            "::vdRngLaplace");
    }

    /// \brief `vsRngWeibull`
    int weibull(MKL_INT n, float *r, float alpha, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngWeibull(method, ptr_, n, r, alpha, a, beta),
            "MKLStream::weibull", "::vsRngWeibull");
    }

    /// \brief `vdRngWeibull`
    int weibull(MKL_INT n, double *r, double alpha, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_WEIBULL_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngWeibull(method, ptr_, n, r, alpha, a, beta),
            "MKLStream::weibull", "::vdRngWeibull");
    }

    /// \brief `vsRngCauchy`
    int cauchy(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngCauchy(method, ptr_, n, r, a, beta), "MKLStream::cauchy",
            "::vsRngCauchy");
    }

    /// \brief `vdRngCauchy`
    int cauchy(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_CAUCHY_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngCauchy(method, ptr_, n, r, a, beta), "MKLStream::cauchy",
            "::vdRngCauchy");
    }

    /// \brief `vsRngRayleigh`
    int rayleigh(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngRayleigh(method, ptr_, n, r, a, beta),
            "MKLStream::rayleigh", "::vsRngRayleigh");
    }

    /// \brief `vdRngRayleigh`
    int rayleigh(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_RAYLEIGH_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngRayleigh(method, ptr_, n, r, a, beta),
            "MKLStream::rayleigh", "::vdRngRayleigh");
    }

    /// \brief `vsRngLognormal`
    int lognormal(MKL_INT n, float *r, float a, float sigma, float b,
        float beta, MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vsRngLognormal(method, ptr_, n, r, a, sigma, b, beta),
            "MKLStream::lognormal", "::vsRngLognormal");
    }

    /// \brief `vdRngLognormal`
    int lognormal(MKL_INT n, double *r, double a, double sigma, double b,
        double beta, MKL_INT method = VSL_RNG_METHOD_LOGNORMAL_BOXMULLER2)
    {
        return internal::mkl_error_check(
            ::vdRngLognormal(method, ptr_, n, r, a, sigma, b, beta),
            "MKLStream::lognormal", "::vdRngLognormal");
    }

    /// \brief `vsRngGumbel`
    int gumbel(MKL_INT n, float *r, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF)
    {
        return internal::mkl_error_check(
            ::vsRngGumbel(method, ptr_, n, r, a, beta), "MKLStream::gumbel",
            "::vsRngGumbel");
    }

    /// \brief `vdRngGumbel`
    int gumbel(MKL_INT n, double *r, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_GUMBEL_ICDF)
    {
        return internal::mkl_error_check(
            ::vdRngGumbel(method, ptr_, n, r, a, beta), "MKLStream::gumbel",
            "::vdRngGumbel");
    }

    /// \brief `vsRngGamma`
    int gamma(MKL_INT n, float *r, float alpha, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM)
    {
        return internal::mkl_error_check(
            ::vsRngGamma(method, ptr_, n, r, alpha, a, beta),
            "MKLStream::gamma", "::vsRngGamma");
    }

    /// \brief `vdRngGamma`
    int gamma(MKL_INT n, double *r, double alpha, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_GAMMA_GNORM)
    {
        return internal::mkl_error_check(
            ::vdRngGamma(method, ptr_, n, r, alpha, a, beta),
            "MKLStream::gamma", "::vdRngGamma");
    }

    /// \brief `vsRngBeta`
    int beta(MKL_INT n, float *r, float p, float q, float a, float beta,
        MKL_INT method = VSL_RNG_METHOD_BETA_CJA)
    {
        return internal::mkl_error_check(
            ::vsRngBeta(method, ptr_, n, r, p, q, a, beta), "MKLStream::beta",
            "::vsRngBeta");
    }

    /// \brief `vdRngBeta`
    int beta(MKL_INT n, double *r, double p, double q, double a, double beta,
        MKL_INT method = VSL_RNG_METHOD_BETA_CJA)
    {
        return internal::mkl_error_check(
            ::vdRngBeta(method, ptr_, n, r, p, q, a, beta), "MKLStream::beta",
            "::vdRngBeta");
    }

    /// \brief `viRngUniform`
    int uniform(MKL_INT n, int *r, int a, int b,
        MKL_INT method = VSL_RNG_METHOD_UNIFORM_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniform(method, ptr_, n, r, a, b), "MKLStream::uniform",
            "::viRngUniform");
    }

    /// \brief `viRngUniform`
    int uniform_bits(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniformBits(method, ptr_, n, r), "MKLStream::uniform_bits",
            "::viRngUniformBits");
    }

    /// \brief `viRngUniform32`
    int uniform_bits32(MKL_INT n, unsigned *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS32_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniformBits32(method, ptr_, n, r),
            "MKLStream::uniform_bits32", "::viRngUniformBits32");
    }

    /// \brief `viRngUniform64`
    int uniform_bits64(MKL_INT n, unsigned MKL_INT64 *r,
        MKL_INT method = VSL_RNG_METHOD_UNIFORMBITS64_STD)
    {
        return internal::mkl_error_check(
            ::viRngUniformBits64(method, ptr_, n, r),
            "MKLStream::uniform_bits64", "::viRngUniformBits64");
    }

    /// \brief `viRngBernoulli`
    int bernoulli(MKL_INT n, int *r, double p,
        MKL_INT method = VSL_RNG_METHOD_BERNOULLI_ICDF)
    {
        return internal::mkl_error_check(
            ::viRngBernoulli(method, ptr_, n, r, p), "MKLStream::bernoulli",
            "::viRngBernoulli");
    }

    /// \brief `viRngGeometric`
    int geometric(MKL_INT n, int *r, double p,
        MKL_INT method = VSL_RNG_METHOD_GEOMETRIC_ICDF)
    {
        return internal::mkl_error_check(
            ::viRngGeometric(method, ptr_, n, r, p), "MKLStream::geometric",
            "::viRngGeometric");
    }

    /// \brief `viRngBinomial`
    int binomial(MKL_INT n, int *r, int ntrial, double p,
        MKL_INT method = VSL_RNG_METHOD_BINOMIAL_BTPE)
    {
        return internal::mkl_error_check(
            ::viRngBinomial(method, ptr_, n, r, ntrial, p),
            "MKLStream::binomial", "::viRngBinomial");
    }

    /// \brief `viRngHypergeometric`
    int hypergeometric(MKL_INT n, int *r, int l, int s, int m,
        MKL_INT method = VSL_RNG_METHOD_HYPERGEOMETRIC_H2PE)
    {
        return internal::mkl_error_check(
            ::viRngHypergeometric(method, ptr_, n, r, l, s, m),
            "MKLStream::hypergeometric", "::viRngHypergeometric");
    }

    /// \brief `viRngPoisson`
    int poisson(MKL_INT n, int *r, double lambda,
        MKL_INT method = VSL_RNG_METHOD_POISSON_PTPE)
    {
        return internal::mkl_error_check(
            ::viRngPoisson(method, ptr_, n, r, lambda), "MKLStream::poisson",
            "::viRngPoisson");
    }

    /// \brief `viRngPoissonV`
    int poisson_v(MKL_INT n, int *r, const double *lambda,
        MKL_INT method = VSL_RNG_METHOD_POISSONV_POISNORM)
    {
        return internal::mkl_error_check(
            ::viRngPoissonV(method, ptr_, n, r, lambda),
            "MKLStream::poisson_v", "::viRngPoissonV");
    }

    /// \brief `viRngNegbinomial`
    int neg_binomial(MKL_INT n, int *r, double a, double p,
        MKL_INT method = VSL_RNG_METHOD_NEGBINOMIAL_NBAR)
    {
        return internal::mkl_error_check(
            ::viRngNegbinomial(method, ptr_, n, r, a, p),
            "MKLStream::neg_binomial", "::viRngNegbinomial");
    }

    private:
    ::VSLStreamStatePtr ptr_;
}; // class MKLStream

/// \brief Equality comparison of MKLStream
/// \ingroup MKLRNG
inline bool operator==(const MKLStream &stream1, const MKLStream &stream2)
{
    if (stream1.get_brng() != stream2.get_brng())
        return false;

    std::size_t n = static_cast<std::size_t>(stream1.get_size());
    Vector<char> s1(n);
    Vector<char> s2(n);
    stream1.save_m(s1.data());
    stream2.save_m(s2.data());
    if (s1 != s2)
        return false;

    return true;
}

/// \brief Inequality comparison of MKLStream
/// \ingroup MKLRNG
inline bool operator!=(const MKLStream &stream1, const MKLStream &stream2)
{
    return !(stream1 == stream2);
}

/// \brief Output of MKLStream
/// \ingroup MKLRNG
template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const MKLStream &stream)
{
    if (!os)
        return os;

    std::size_t n = static_cast<std::size_t>(stream.get_size());
    std::size_t m = sizeof(std::uintmax_t);
    if (n % m != 0)
        n += m - n % m;
    n /= m;
    Vector<std::uintmax_t> s(n);
    stream.save_m(reinterpret_cast<char *>(s.data()));

    os << stream.get_brng() << ' ';
    os << s;

    return os;
}

/// \brief Input of MKLStream
/// \ingroup MKLRNG
template <typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits> &operator>>(
    std::basic_istream<CharT, Traits> &is, MKLStream &stream)
{
    if (!is)
        return is;

    MKL_INT brng;
    Vector<std::uintmax_t> s;
    is >> std::ws >> brng;
    is >> std::ws >> s;

    if (static_cast<bool>(is)) {
        MKLStream tmp;
        tmp.load_m(reinterpret_cast<const char *>(s.data()));
        stream = std::move(tmp);
    }

    return is;
}

namespace internal
{

template <MKL_INT BRNG>
class MKLMaxOffset : public std::integral_constant<MKL_INT, 0>
{
}; // MKLMaxOffset;

template <>
class MKLMaxOffset<VSL_BRNG_MT2203>
    : public std::integral_constant<MKL_INT, 6024>
{
}; // MKLMaxOffset

template <>
class MKLMaxOffset<VSL_BRNG_WH> : public std::integral_constant<MKL_INT, 273>
{
}; // MKLMaxOffset

template <MKL_INT BRNG, MKL_INT MaxOffset = MKLMaxOffset<BRNG>::value>
class MKLOffset
{
    public:
    static MKL_INT eval(MKL_INT offset)
    {
        VSMC_RUNTIME_WARNING_RNG_MKL_OFFSET;

        return BRNG + offset % MaxOffset;
    }
}; // class MKLOffset

template <MKL_INT BRNG>
class MKLOffset<BRNG, 0>
{
    public:
    static MKL_INT eval(MKL_INT) { return BRNG; }
}; // class MKLOffset

template <int>
class MKLResultTypeTrait;

template <>
class MKLResultTypeTrait<32>
{
    public:
    using type = unsigned;
}; // class MKLResultTypeTrait

template <>
class MKLResultTypeTrait<64>
{
    public:
    using type = unsigned MKL_INT64;
}; // class MKLResultTypeTrait

template <int Bits>
using MKLResultType = typename MKLResultTypeTrait<Bits>::type;

template <int>
class MKLUniformBits;

template <>
class MKLUniformBits<32>
{
    public:
    static void eval(MKLStream &stream, MKL_INT n, unsigned *r)
    {
        stream.uniform_bits32(n, r);
    }
}; // class MKLUniformBits

template <>
class MKLUniformBits<64>
{
    public:
    static void eval(MKLStream &stream, MKL_INT n, unsigned MKL_INT64 *r)
    {
        stream.uniform_bits64(n, r);
    }
}; // class MKLUniformBits

} // namespace vsmc::internal

/// \brief MKL RNG C++11 engine
/// \ingroup MKLRNG
///
/// \tparam BRNG A MKL BRNG. It need to support either `viRngUniformBits32` or
/// `viRngUniformBits64`.
/// \tparam Bits The number of bits in the output unsigned integer
/// - It can be 32 if and only if `viRngUniformBits32` is supported
/// - It can be 64 if and only if `viRngUniformBits64` is supported
template <MKL_INT BRNG, int Bits>
class MKLEngine
{
    static_assert(Bits == 32 || Bits == 64,
        "**MKLEngine** USED WITH Bits OTHER THAN 32, OR 64");

    public:
    using result_type = internal::MKLResultType<Bits>;

    explicit MKLEngine(result_type s = 1) : index_(M_) { seed(s); }

    template <typename SeedSeq>
    explicit MKLEngine(SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_INT,
            result_type, MKLEngine<BRNG, Bits>>::value>::type * = nullptr)
        : index_(M_)
    {
        seed(seq);
    }

    MKLEngine(MKL_INT offset, result_type s) : index_(M_)
    {
        static_assert(internal::MKLMaxOffset<BRNG>::value > 0,
            "**MKLEngine** DOES NOT SUPPORT OFFSETING");

        seed(offset, s);
    }

    template <typename SeedSeq>
    explicit MKLEngine(MKL_INT offset, SeedSeq &seq,
        typename std::enable_if<internal::is_seed_seq<SeedSeq, MKL_INT,
            MKL_UINT, MKLEngine<BRNG, Bits>>::value>::type * = nullptr)
        : index_(M_)
    {
        static_assert(internal::MKLMaxOffset<BRNG>::value > 0,
            "**MKLEngine** DOES NOT SUPPORT OFFSETING");

        seed(offset, seq);
    }

    void seed(result_type s)
    {
        s %= static_cast<result_type>(std::numeric_limits<MKL_UINT>::max());
        MKL_INT brng = stream_.empty() ? BRNG : stream_.get_brng();
        stream_.reset(brng, static_cast<MKL_UINT>(s));
        index_ = M_;
    }

    template <typename SeedSeq>
    void seed(
        SeedSeq &seq, typename std::enable_if<internal::is_seed_seq<SeedSeq,
                          MKL_INT, result_type>::value>::type * = nullptr)
    {
        MKL_INT brng = stream_.empty() ? BRNG : stream_.get_brng();
        Vector<MKL_UINT> params;
        MKL_INT n = seed_params(brng, seq, params);
        stream_.reset(brng, n, params.data());
        index_ = M_;
    }

    void seed(MKL_INT offset, result_type s)
    {
        static_assert(internal::MKLMaxOffset<BRNG>::value > 0,
            "**MKLEngine** DOES NOT SUPPORT OFFSETING");

        s %= static_cast<result_type>(std::numeric_limits<MKL_UINT>::max());
        MKL_INT brng = internal::MKLOffset<BRNG>::eval(offset);
        stream_.reset(brng, static_cast<MKL_UINT>(s));
        index_ = M_;
    }

    template <typename SeedSeq>
    void seed(MKL_INT offset, SeedSeq &seq,
        typename std::enable_if<
            internal::is_seed_seq<SeedSeq, MKL_UINT>::value>::type * = nullptr)
    {
        static_assert(internal::MKLMaxOffset<BRNG>::value > 0,
            "**MKLEngine** DOES NOT SUPPORT OFFSETING");

        MKL_INT brng = internal::MKLOffset<BRNG>::eval(offset);
        Vector<unsigned> params;
        MKL_INT n = seed_params(brng, seq, params);
        stream_.reset(brng, n, params.data());
        index_ = M_;
    }

    result_type operator()()
    {
        if (index_ == M_) {
            generate();
            index_ = 0;
        }

        return buffer_[index_++];
    }

    void operator()(std::size_t n, result_type *r)
    {
        internal::size_check<MKL_INT>(n, "MKLEngine::operator()");

        const std::size_t remain = M_ - index_;

        if (n < remain) {
            std::copy_n(buffer_.data() + index_, n, r);
            index_ += n;
            return;
        }

        std::copy_n(buffer_.data() + index_, remain, r);
        r += remain;
        n -= remain;
        index_ = M_;

        const std::size_t m = n / M_ * M_;
        const std::size_t l = n % M_;
        internal::MKLUniformBits<Bits>::eval(
            stream_, static_cast<MKL_INT>(m), r);
        r += m;

        generate();
        std::copy_n(buffer_.data(), l, r);
        index_ = l;
    }

    /// \brief Discard the buffer
    std::size_t discard()
    {
        const std::size_t remain = M_ - index_;
        index_ = M_;

        return remain;
    }

    void discard(long long nskip)
    {
        VSMC_RUNTIME_ASSERT_RNG_MKL_DISCARD;

        if (nskip == 0)
            return;

        const long long remain = static_cast<long long>(M_ - index_);
        if (nskip <= remain) {
            index_ += static_cast<std::size_t>(nskip);
            return;
        }
        nskip -= remain;
        index_ = M_;

        ::VSLBRngProperties properties;
        MKLStream::get_brng_properties(stream_.get_brng(), &properties);
        int bits = properties.NBits;
        long long M = static_cast<long long>(M_);
        long long m = nskip / M * M;
        if (Bits >= bits)
            m *= Bits / bits + (Bits % bits == 0 ? 0 : 1);
        switch (stream_.get_brng()) {
            case VSL_BRNG_MCG31: stream_.skip_ahead(m); break;
            case VSL_BRNG_MRG32K3A: stream_.skip_ahead(m); break;
            case VSL_BRNG_MCG59: stream_.skip_ahead(m); break;
            case VSL_BRNG_WH: stream_.skip_ahead(m); break;
            case VSL_BRNG_MT19937: stream_.skip_ahead(m); break;
            case VSL_BRNG_SFMT19937: stream_.skip_ahead(m); break;
            case VSL_BRNG_SOBOL: stream_.skip_ahead(m); break;
            case VSL_BRNG_NIEDERR: stream_.skip_ahead(m); break;
#if INTEL_MKL_VERSION >= 110300
            case VSL_BRNG_PHILOX4X32X10: stream_.skip_ahead(m); break;
            case VSL_BRNG_ARS5: stream_.skip_ahead(m); break;
#endif
            default:
                if (MKLStream::has_skip_ahead(stream_.get_brng())) {
                    stream_.skip_ahead(m);
                    break;
                }
                while (nskip > M) {
                    generate();
                    nskip -= M;
                }
        };
        generate();
        index_ = static_cast<std::size_t>(nskip % M);
    }

    static constexpr result_type min()
    {
        return std::numeric_limits<result_type>::min();
    }

    static constexpr result_type max()
    {
        return std::numeric_limits<result_type>::max();
    }

    MKLStream &stream() { return stream_; }
    const MKLStream &stream() const { return stream_; }

    friend bool operator==(
        const MKLEngine<BRNG, Bits> &gen1, const MKLEngine<BRNG, Bits> &gen2)
    {
        if (gen1.stream_ != gen2.stream_)
            return false;
        if (gen1.buffer_ != gen2.buffer_)
            return false;
        if (gen1.index_ != gen2.index_)
            return false;
        return true;
    }

    friend bool operator!=(
        const MKLEngine<BRNG, Bits> &gen1, const MKLEngine<BRNG, Bits> &gen2)
    {
        return !(gen1 == gen2);
    }

    template <typename CharT, typename Traits>
    friend std::basic_ostream<CharT, Traits> &operator<<(
        std::basic_ostream<CharT, Traits> &os,
        const MKLEngine<BRNG, Bits> &gen)
    {
        if (!os)
            return os;

        os << gen.stream_ << ' ';
        os << gen.buffer_ << ' ';
        os << gen.index_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend std::basic_istream<CharT, Traits> &operator>>(
        std::basic_istream<CharT, Traits> &is, MKLEngine<BRNG, Bits> &gen)
    {
        if (!is)
            return is;

        MKLStream stream;
        Vector<result_type> buffer;
        std::size_t index;
        is >> std::ws >> stream;
        is >> std::ws >> buffer;
        is >> std::ws >> index;

        if (static_cast<bool>(is)) {
            gen.stream_ = std::move(stream);
            gen.buffer_ = std::move(buffer);
            gen.index_ = index;
        }

        return is;
    }

    private:
    static constexpr std::size_t M_ = internal::BufferSize<result_type>::value;

    Vector<result_type> buffer_;
    std::size_t index_;
    MKLStream stream_;

    void generate()
    {
        buffer_.resize(M_);
        internal::MKLUniformBits<Bits>::eval(
            stream_, static_cast<MKL_INT>(M_), buffer_.data());
    }

    template <typename SeedSeq>
    MKL_INT seed_params(MKL_INT brng, SeedSeq &seq, Vector<unsigned> &params)
    {
        ::VSLBRngProperties properties;
        MKLStream::get_brng_properties(brng, &properties);
        MKL_INT n = properties.NSeeds;
        params.resize(static_cast<std::size_t>(n));
        seq.generate(params.begin(), params.end());

        return n;
    }
}; // class MKLEngine

/// \brief A 59-bit multiplicative congruential generator
/// \ingroup MKLRNG
using MKL_MCG59 = MKLEngine<VSL_BRNG_MCG59, 32>;

/// \brief A 59-bit multiplicative congruential generator (64-bit)
/// \ingroup MKLRNG
using MKL_MCG59_64 = MKLEngine<VSL_BRNG_MCG59, 64>;

/// \brief A Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
using MKL_MT19937 = MKLEngine<VSL_BRNG_MT19937, 32>;

/// \brief A Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
using MKL_MT19937_64 = MKLEngine<VSL_BRNG_MT19937, 64>;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor
/// \ingroup MKLRNG
using MKL_MT2203 = MKLEngine<VSL_BRNG_MT2203, 32>;

/// \brief A set of 6024 Mersenne-Twister pseudoranom number genertor (64-bit)
/// \ingroup MKLRNG
using MKL_MT2203_64 = MKLEngine<VSL_BRNG_MT2203, 64>;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number
/// genertor
/// \ingroup MKLRNG
using MKL_SFMT19937 = MKLEngine<VSL_BRNG_SFMT19937, 32>;

/// \brief A SIMD-oriented fast Mersenne-Twister pseudoranom number genertor
/// (64-bit)
/// \ingroup MKLRNG
using MKL_SFMT19937_64 = MKLEngine<VSL_BRNG_SFMT19937, 64>;

/// \brief A non-determinstic random number generator
/// \ingroup MKLRNG
using MKL_NONDETERM = MKLEngine<VSL_BRNG_NONDETERM, 32>;

/// \brief A non-determinstic random number generator (64-bit)
/// \ingroup MKLRNG
using MKL_NONDETERM_64 = MKLEngine<VSL_BRNG_NONDETERM, 64>;

#if INTEL_MKL_VERSION >= 110300

/// \brief A counter-based random number generator
/// \ingroup MKLRNG
using MKL_ARS5 = MKLEngine<VSL_BRNG_ARS5, 32>;

/// \brief A counter-based random number generator (64-bit)
/// \ingroup MKLRNG
using MKL_ARS5_64 = MKLEngine<VSL_BRNG_ARS5, 64>;

/// \brief A counter-based random number generator
/// \ingroup MKLRNG
using MKL_PHILOX4X32X10 = MKLEngine<VSL_BRNG_PHILOX4X32X10, 32>;

/// \brief A counter-based random number generator (64-bit)
/// \ingroup MKLRNG
using MKL_PHILOX4X32X10_64 = MKLEngine<VSL_BRNG_PHILOX4X32X10, 64>;

#endif // INTEL_MKL_VERSION >= 110300

template <MKL_INT BRNG, int Bits>
inline void rng_rand(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    typename MKLEngine<BRNG, Bits>::result_type *r)
{
    rng(n, r);
}

template <MKL_INT BRNG, int Bits>
inline void beta_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float alpha, float beta)
{
    internal::size_check<MKL_INT>(n, "beta_distribution)");
    rng.stream().beta(static_cast<MKL_INT>(n), r, alpha, beta, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void beta_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double alpha, double beta)
{
    internal::size_check<MKL_INT>(n, "beta_distribution)");
    rng.stream().beta(static_cast<MKL_INT>(n), r, alpha, beta, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "cauchy_distribution)");
    rng.stream().cauchy(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void cauchy_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "cauchy_distribution)");
    rng.stream().cauchy(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float lambda)
{
    internal::size_check<MKL_INT>(n, "exponential_distribution)");
    rng.stream().exponential(static_cast<MKL_INT>(n), r, 0, 1 / lambda);
}

template <MKL_INT BRNG, int Bits>
inline void exponential_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double lambda)
{
    internal::size_check<MKL_INT>(n, "exponential_distribution)");
    rng.stream().exponential(static_cast<MKL_INT>(n), r, 0, 1 / lambda);
}

template <MKL_INT BRNG, int Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "extreme_value_distribution)");
    rng.stream().gumbel(static_cast<MKL_INT>(n), r, a, b);
    sub(n, 2 * a, r, r);
}

template <MKL_INT BRNG, int Bits>
inline void extreme_value_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "extreme_value_distribution)");
    rng.stream().gumbel(static_cast<MKL_INT>(n), r, a, b);
    sub(n, 2 * a, r, r);
}

template <MKL_INT BRNG, int Bits>
inline void gamma_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float alpha, float beta)
{
    internal::size_check<MKL_INT>(n, "gamma_distribution)");
    rng.stream().gamma(static_cast<MKL_INT>(n), r, alpha, 0, beta);
}

template <MKL_INT BRNG, int Bits>
inline void gamma_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double alpha, double beta)
{
    internal::size_check<MKL_INT>(n, "gamma_distribution)");
    rng.stream().gamma(static_cast<MKL_INT>(n), r, alpha, 0, beta);
}

template <MKL_INT BRNG, int Bits>
inline void laplace_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float location, float scale)
{
    internal::size_check<MKL_INT>(n, "lapace_distribution)");
    rng.stream().laplace(static_cast<MKL_INT>(n), r, location, scale);
}

template <MKL_INT BRNG, int Bits>
inline void laplace_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double location, double scale)
{
    internal::size_check<MKL_INT>(n, "lapace_distribution)");
    rng.stream().laplace(static_cast<MKL_INT>(n), r, location, scale);
}

template <MKL_INT BRNG, int Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float m, float s)
{
    internal::size_check<MKL_INT>(n, "lognormal_distribution)");
    rng.stream().lognormal(static_cast<MKL_INT>(n), r, m, s, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void lognormal_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double m, double s)
{
    internal::size_check<MKL_INT>(n, "lognormal_distribution)");
    rng.stream().lognormal(static_cast<MKL_INT>(n), r, m, s, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void normal_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, float mean, float stddev)
{
    internal::size_check<MKL_INT>(n, "normal_distribution)");
    rng.stream().gaussian(static_cast<MKL_INT>(n), r, mean, stddev);
}

template <MKL_INT BRNG, int Bits>
inline void normal_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, double mean, double stddev)
{
    internal::size_check<MKL_INT>(n, "normal_distribution)");
    rng.stream().gaussian(static_cast<MKL_INT>(n), r, mean, stddev);
}

template <MKL_INT BRNG, int Bits>
inline void normal_mv_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    float *r, std::size_t m, const float *mean, const float *chol)
{
    internal::size_check<MKL_INT>(n, "normal_mv_distribution)");
    internal::size_check<MKL_INT>(m, "normal_mv_distribution)");
    rng.stream().gaussian_mv(static_cast<MKL_INT>(n), r,
        static_cast<MKL_INT>(m), VSL_MATRIX_STORAGE_PACKED, mean, chol);
}

template <MKL_INT BRNG, int Bits>
inline void normal_mv_distribution(MKLEngine<BRNG, Bits> &rng, std::size_t n,
    double *r, std::size_t m, const double *mean, const double *chol)
{
    internal::size_check<MKL_INT>(n, "normal_mv_distribution)");
    internal::size_check<MKL_INT>(m, "normal_mv_distribution)");
    rng.stream().gaussian_mv(static_cast<MKL_INT>(n), r,
        static_cast<MKL_INT>(m), VSL_MATRIX_STORAGE_PACKED, mean, chol);
}

template <MKL_INT BRNG, int Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float sigma)
{
    internal::size_check<MKL_INT>(n, "rayleigh_distribution)");
    rng.stream().rayleigh(
        static_cast<MKL_INT>(n), r, 0, const_sqrt_2<float>() * sigma);
}

template <MKL_INT BRNG, int Bits>
inline void rayleigh_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double sigma)
{
    internal::size_check<MKL_INT>(n, "rayleigh_distribution)");
    rng.stream().rayleigh(
        static_cast<MKL_INT>(n), r, 0, const_sqrt_2<double>() * sigma);
}

template <MKL_INT BRNG, int Bits>
inline void u01_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r)
{
    internal::size_check<MKL_INT>(n, "u01_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void u01_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r)
{
    internal::size_check<MKL_INT>(n, "u01_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void u01_co_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r)
{
    internal::size_check<MKL_INT>(n, "u01_co_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void u01_co_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r)
{
    internal::size_check<MKL_INT>(n, "u01_co_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, 0, 1);
}

template <MKL_INT BRNG, int Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "uniform_real_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void uniform_real_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "uniform_real_distribution)");
    rng.stream().uniform(static_cast<MKL_INT>(n), r, a, b);
}

template <MKL_INT BRNG, int Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, float *r, float a, float b)
{
    internal::size_check<MKL_INT>(n, "weibull_distribution)");
    rng.stream().weibull(static_cast<MKL_INT>(n), r, a, 0, b);
}

template <MKL_INT BRNG, int Bits>
inline void weibull_distribution(
    MKLEngine<BRNG, Bits> &rng, std::size_t n, double *r, double a, double b)
{
    internal::size_check<MKL_INT>(n, "weibull_distribution)");
    rng.stream().weibull(static_cast<MKL_INT>(n), r, a, 0, b);
}

namespace internal
{

template <typename RNGType>
class MKLStreamState
{
    public:
    unsigned reserved1[2];
    unsigned reserved2[2];
    RNGType rng;
}; // class MKLStreamState

template <typename RNGType>
inline constexpr int mkl_nseeds(std::false_type)
{
    return (sizeof(typename RNGType::ctr_type) +
               sizeof(typename RNGType::key_type)) /
        sizeof(unsigned);
}

template <typename RNGType>
inline constexpr int mkl_nseeds(std::true_type)
{
    return 1;
}

template <typename RNGType>
inline constexpr int mkl_nseeds()
{
    return mkl_nseeds<RNGType>(std::integral_constant<
        bool, (std::is_same<CtrType<RNGType>, NullType>::value ||
                  std::is_same<KeyType<RNGType>, NullType>::value)>());
}

template <typename RNGType>
inline int mkl_init(
    RNGType &rng, int n, const unsigned *param, std::false_type)
{
    int nc = static_cast<int>(
        sizeof(typename RNGType::ctr_type) / sizeof(unsigned));
    int nk = static_cast<int>(
        sizeof(typename RNGType::key_type) / sizeof(unsigned));
    new (static_cast<void *>(&rng)) RNGType();

    if (n > 0) {
        std::size_t size =
            static_cast<std::size_t>(std::min(n, nk)) * sizeof(unsigned);
        typename RNGType::key_type key;
        std::fill(key.begin(), key.end(), 0);
        std::memcpy(key.data(), param, size);
        rng.key(key);
    }

    if (n > nk) {
        n -= nk;
        param += nk;
        std::size_t size =
            static_cast<std::size_t>(std::min(n, nc)) * sizeof(unsigned);
        typename RNGType::ctr_type ctr;
        std::fill(ctr.begin(), ctr.end(), 0);
        std::memcpy(ctr.data(), param, size);
        rng.ctr(ctr);
    }

    return 0;
}

template <typename RNGType>
inline int mkl_init(RNGType &rng, int n, const unsigned *param, std::true_type)
{
    if (n == 0) {
        new (static_cast<void *>(&rng)) RNGType();
    } else {
        new (static_cast<void *>(&rng))
            RNGType(static_cast<typename RNGType::result_type>(param[0]));
    }

    return 0;
}

template <typename RNGType>
inline int mkl_init(
    int method, ::VSLStreamStatePtr stream, int n, const unsigned *param)
{
    RNGType &rng = (*reinterpret_cast<MKLStreamState<RNGType> *>(stream)).rng;

    if (method == VSL_INIT_METHOD_STANDARD) {
        return mkl_init(
            rng, n, param,
            std::integral_constant<
                bool, (std::is_same<CtrType<RNGType>, NullType>::value ||
                          std::is_same<KeyType<RNGType>, NullType>::value)>());
    }

    if (method == VSL_INIT_METHOD_LEAPFROG)
        return VSL_RNG_ERROR_LEAPFROG_UNSUPPORTED;

    if (method == VSL_INIT_METHOD_SKIPAHEAD)
        rng.discard(static_cast<unsigned>(n));

    return 0;
}

template <typename RNGType, typename RealType>
inline int mkl_uniform_real(
    ::VSLStreamStatePtr stream, int n, RealType *r, RealType a, RealType b)
{
    RNGType &rng = (*reinterpret_cast<MKLStreamState<RNGType> *>(stream)).rng;
    uniform_real_distribution(rng, static_cast<std::size_t>(n), r, a, b);

    return 0;
}

template <typename RNGType>
inline int mkl_uniform_int(::VSLStreamStatePtr stream, int n, unsigned *r)
{
    RNGType &rng = (*reinterpret_cast<MKLStreamState<RNGType> *>(stream)).rng;
    uniform_bits_distribution(rng, static_cast<std::size_t>(n), r);

    return 0;
}

} // namespace vsmc::internal

/// \brief Register a C++11 RNG as MKL BRNG
/// \ingroup MKLRNG
///
/// \details
/// Only engines defined in this library and the standard library are
/// specialized. This function requires the C runtime of the library.
template <typename RNGType>
int mkl_brng()
{
    static ::VSLBRngProperties properties = {
        sizeof(internal::MKLStreamState<RNGType>),
        internal::mkl_nseeds<RNGType>(), 1, 4, 32, internal::mkl_init<RNGType>,
        internal::mkl_uniform_real<RNGType, float>,
        internal::mkl_uniform_real<RNGType, double>,
        internal::mkl_uniform_int<RNGType>};
    static int brng = ::vslRegisterBrng(&properties);

    return brng;
}

} // namespace vsmc

#endif // VSMC_RNG_MKL_HPP
