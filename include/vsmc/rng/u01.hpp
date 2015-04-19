//============================================================================
// vSMC/include/vsmc/rng/u01.hpp
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

#ifndef VSMC_RNG_U01_HPP
#define VSMC_RNG_U01_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01.h>

#define VSMC_RUNTIME_ASSERT_RNG_U01_U01_SEQUENCE(Method) \
    VSMC_RUNTIME_ASSERT((n < N_ && (n == n_ || n == n_ + 1 || n_ == N_)),    \
            ("**U01Sequence"#Method"::operator[]** INVALID INDEX"))

#define VSMC_DEFINE_RNG_U01(FPType, Left, Right, left, right, UBits, FBits) \
template <> struct U01<Left, Right, uint##UBits##_t, FPType>                 \
{                                                                            \
    FPType operator() (uint##UBits##_t u) const                              \
    {return ::u01_##left##_##right##_##UBits##_##FBits(u);}                  \
                                                                             \
    static FPType uint2fp (uint##UBits##_t u)                                \
    {return ::u01_##left##_##right##_##UBits##_##FBits(u);}                  \
};

namespace vsmc {

/// \brief Parameter type for open interval
/// \ingroup U01
struct Open {};

/// \brief Parameter type for closed interval
/// \ingroup U01
struct Closed {};

template <typename, typename, typename, typename> struct U01;

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Closed, closed, closed, 32, 24)

/// \brief Converting 32-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Open, closed, open, 32, 24)

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Closed, open, closed, 32, 24)

/// \brief Converting 32-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Open, open, open, 32, 24)

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Closed, closed, closed, 32, 53)

/// \brief Converting 32-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Open, closed, open, 32, 53)

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Closed, open, closed, 32, 53)

/// \brief Converting 32-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Open, open, open, 32, 53)

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Closed, closed, closed, 64, 24)

/// \brief Converting 64-bits unsigned to single precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Closed, Open, closed, open, 64, 24)

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Closed, open, closed, 64, 24)

/// \brief Converting 64-bits unsigned to single precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(float, Open, Open, open, open, 64, 24)

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Closed, closed, closed, 64, 53)

/// \brief Converting 64-bits unsigned to double precision uniform \f$[0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Closed, Open, closed, open, 64, 53)

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1]\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Closed, open, closed, 64, 53)

/// \brief Converting 64-bits unsigned to double precision uniform \f$(0,1)\f$
/// \ingroup U01
VSMC_DEFINE_RNG_U01(double, Open, Open, open, open, 64, 53)

/// \brief Generate a fixed length sequence of uniform \f$[0,1)\f$ random
/// variates by sorting.
///
/// \details
/// This is primarily used in resampling algorithms within the library but can
/// be used for other purposes. It deals with the usage case similar to the
/// following,
/// ~~~{.cpp}
/// const std::size_t N = 1000;
/// std::mt19937 rng;
/// std::uniform_real_distribution<double> runif(0, 1);
/// std::vector<double> u01(N);
/// for (std::size_t i = 0; i != N; ++i)
///     u01[i] = runif(rng);
/// std::sort(u01.begin(), u01.end());
/// for (std::size_t i = 0; i != N; ++i)
///     do_something_with_u01(u01[i]);
/// ~~~
/// In the above example, one want N uniform random variates, and having them
/// sorted. The sorted sequence is accessed sequentially. There are two
/// undesired effects. First the program has a runtime cost \f$O(N\log N)\f$.
/// Second, it has a memory cost \f$O(N)\f$. The presented class can be used in
/// place of the temporary vector in the following fashion.
/// ~~~{.cpp}
/// const std::size_t N = 1000;
/// std::mt19937 rng;
/// U01SequenceSorted<std::mt19937> u01(N, rng);
/// for (std::size_t i = 0; i != N; ++i)
///     do_something_with_u01(u01[i]);
/// ~~~
/// The above program will has a runtime cost \f$O(N)\f$ and a memory cost
/// \f$O(1)\f$. However, the overloaded `operator[]` has a few restrictions.
/// Most importantly, it is a forward one-pass operation. More specifically,
/// let `n` be the calling argument and `nlast` be the calling argument of the
/// last time it is called.
/// - `n` can only be either `0`, `nlast`, or `nlast + 1`
/// - `n` can be at most `N - 1`.
template <typename RngType>
class U01SequenceSorted
{
    public :

    U01SequenceSorted (std::size_t N, RngType &rng) :
        N_(N), n_(N), u_(0), lmax_(0), rng_(rng), runif_(0, 1) {}

    /// \brief Generate the n-th random variates
    /// 
    double operator[] (std::size_t n)
    {
        using std::exp;
        using std::log;

        VSMC_RUNTIME_ASSERT_RNG_U01_U01_SEQUENCE(Sorted)

        if (n == n_)
            return u_;

        lmax_ += log(1 - runif_(rng_)) / (N_ - n);
        n_ = n;
        u_ = 1 - exp(lmax_);

        return u_;
    }

    double operator() (std::size_t n) {return operator[](n);}

    private :

    std::size_t N_;
    std::size_t n_;
    double u_;
    double lmax_;
    RngType &rng_;
    std::uniform_real_distribution<double> runif_;
}; // U01SequenceSorted

/// \brief Generate a fixed length sequence of uniform \f$[0,1)\f$ random
/// variates by stratified sampling.
///
/// \details
/// This is similar to U01SequenceSorted except that, instead of generating the
/// sequence as if by sorting. It is done by generating
/// \f$u_i = U_i / N + (i - 1) / N\f$ where \f$U_i\f$ is uniform \f$[0,1)\f$
/// random variates.
template <typename RngType>
class U01SequenceStratified
{
    public :

    U01SequenceStratified (std::size_t N, RngType &rng) :
        N_(N), n_(N), u_(0), delta_(1.0 / N), rng_(rng), runif_(0, 1) {}

    double operator[] (std::size_t n)
    {
        VSMC_RUNTIME_ASSERT_RNG_U01_U01_SEQUENCE(Stratified)

        if (n == n_)
            return u_;

        n_ = n;
        u_ = runif_(rng_) * delta_ + n * delta_;

        return u_;
    }

    double operator() (std::size_t n) {return operator[](n);}

    private :

    std::size_t N_;
    std::size_t n_;
    double u_;
    double delta_;
    RngType &rng_;
    std::uniform_real_distribution<double> runif_;
}; // class U01SequenceStratified

/// \brief Generate a fixed length sequence of uniform \f$[0,1)\f$ random
/// variates by systematic sampling.
///
/// \details
/// This is similar to U01SequenceSorted except that, instead of generating the
/// sequence as if by sorting. It is done by generating
/// \f$u_i = U / N + (i - 1) / N\f$ where \f$U\f$ is uniform \f$[0,1)\f$
/// random variates and it is generated only once for all \f$i\f$.
template <typename RngType>
class U01SequenceSystematic
{
    public :

    U01SequenceSystematic (std::size_t N, RngType &rng) :
        N_(N), n_(N), u_(0), u0_(0), delta_(1.0 / N)
    {
        std::uniform_real_distribution<double> runif(0, 1);
        u0_ = runif(rng) * delta_;
    }

    double operator[] (std::size_t n)
    {
        VSMC_RUNTIME_ASSERT_RNG_U01_U01_SEQUENCE(Systematic)

        if (n == n_)
            return u_;

        n_ = n;
        u_ = u0_ + n * delta_;

        return u_;
    }

    double operator() (std::size_t n) {return operator[](n);}

    private :

    std::size_t N_;
    std::size_t n_;
    double u_;
    double u0_;
    double delta_;
}; // class U01SequenceSystematic

} // namespace vsmc

#endif // VSMC_RNG_U01_HPP
