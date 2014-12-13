//============================================================================
// vSMC/include/vsmc/rng/uniform_real_distribution.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP
#define VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01.hpp>

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(a, b) \
    VSMC_RUNTIME_ASSERT((a <= b),                                            \
            ("**UniformRealDistribution** CONSTRUCTED WITH INVALID "         \
             "MINIMUM AND MAXIMUM PARAMTER VALUES"))

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MIN(eng_min) \
    VSMC_RUNTIME_ASSERT((eng_min == 0),                                      \
            ("**UniformRealDistribution::operator()** "                      \
             "ENGINE MEMBER FUNCTION min() RETURN A VALUE OTHER THAN ZERO"))

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MAX(eng_max) \
    VSMC_RUNTIME_ASSERT((eng_max == uint32_t_max_ || eng_max == uint64_t_max_),\
            ("**UniformRealDistribution::operator()** "                      \
             "ENGINE MEMBER FUNCTION max() RETURN A VALUE OTHER THAN "       \
             "THE MAXIMUM OF uint32_t OR uint64_t"))

namespace vsmc {

namespace internal {

template <uint64_t, uint64_t>
struct UniformRealDistributionFRIntType;

template <>
struct UniformRealDistributionFRIntType<0,
    static_cast<uint64_t>(static_cast<uint32_t>(~(static_cast<uint32_t>(0))))>
{typedef uint32_t type;};

template <>
struct UniformRealDistributionFRIntType<0,
    static_cast<uint64_t>(~(static_cast<uint64_t>(0)))>
{typedef uint64_t type;};

template <typename FPType, typename Left, typename Right, typename Eng, bool>
class UniformRealDistributionOp
{
    static VSMC_CONSTEXPR const uint64_t uint32_t_max_ = static_cast<uint64_t>(
            static_cast<uint32_t>(~(static_cast<uint32_t>(0))));

    static VSMC_CONSTEXPR const uint64_t uint64_t_max_ = static_cast<uint64_t>(
            ~(static_cast<uint64_t>(0)));

    public :

    static FPType uint2fp (Eng &eng)
    {
        static const uint64_t eng_max = static_cast<uint64_t>(
                eng.max VSMC_MNE ());

        VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MIN(
                (eng.min VSMC_MNE ()));
        VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MAX(eng_max);

        if (eng_max == uint32_t_max_)
            return U01<Left, Right, uint32_t, FPType>::uint2fp(
                    static_cast<uint32_t>(eng()));

        if (eng_max == uint64_t_max_)
            return U01<Left, Right, uint64_t, FPType>::uint2fp(
                    static_cast<uint64_t>(eng()));

        return 0;
    }
}; // class UniformRealDistributionOp

#if VSMC_HAS_CXX11_CONSTEXPR
template <typename FPType, typename Left, typename Right, typename Eng>
class UniformRealDistributionOp<FPType, Left, Right, Eng, true>
{
    typedef typename internal::UniformRealDistributionFRIntType<
        Eng::min VSMC_MNE(), Eng::max VSMC_MNE ()>::type eng_uint_t;

    public :

    static FPType uint2fp (Eng &eng)
    {
        return U01<Left, Right, eng_uint_t, FPType>::uint2fp(
                static_cast<eng_uint_t>(eng()));
    }
}; // class UniformRealDistributionOp
#endif

} // namespace vsmc::interal

/// \brief Uniform real distribution with variants open/closed variants
/// \ingroup Distribution
///
/// \details
/// This distribution is almost identical to C++11
/// `std::uniform_real_distribution`. But it differs in two important aspects
/// - It allows the interval to be either open or closed on both sides.
/// - It requires that the uniform random number generator to produce integers
///   on the full range of either `uint32_t` or `uint64_t`.
/// \tparam FPType The floating points type of results
/// \tparam Left Shall the left side of the interval be Open or Closed
/// \tparam Right Shall the right side of the interval be Open or Closed
/// \tparam MinMaxIsConstexpr Whether or not UniformRealDistribution shall
/// expect RNG engines that will be used with it has their `min` and `max`
/// member functions defined as contant expresssion. For example,
/// ~~~{.cpp}
/// struct Engine1
/// {
///     static constexpr uint32_t min ();
///     static constexpr uint32_t max ();
///     uint32_t operator() ();
/// };
///
/// struct Engine2
/// {
///     uint32_t min ();
///     uint32_t max ();
///     uint32_t operator() ();
/// };
///
/// Engine1 eng1;
/// Engine2 eng2;
/// UniformRealDistribution<double, true>  runif1(0, 1);
/// UniformRealDistribution<double, false> runif2(0, 1);
///
/// runif1(eng1); // OK
/// runif1(eng2); // ERROR! Engine2 does not have contant expression min/max
/// runif2(eng1); // OK, but less efficient than runif1(eng1)
/// runif2(eng2); // OK
/// ~~~
/// Without C++11 support, this template parameter has no effect and the
/// distribution will always determine whether the RNG engine generates (full
/// range) 32-bits or 64-bits unsigned integers, and raise a runtime assertion
/// failure if it does neither. With C++11 support, and when this template
/// parameter is set to `true`, then the distribution will make the decision at
/// compile time, and will rais a static assertion failure if the generated
/// integers does not cover the full range of either 32-bits or 64-bits
/// unsigned integers.
template <typename FPType = double,
         typename Left = Closed, typename Right = Open,
         bool MinMaxIsConstexpr = false>
class UniformRealDistribution
{
    public :

    typedef FPType result_type;

    struct param_type
    {
        typedef FPType result_type;

        typedef UniformRealDistribution<FPType, Left, Right, MinMaxIsConstexpr>
            distribution_type;

        explicit param_type (result_type a = 0, result_type b = 1) :
            a_(a), b_(b) {}

        result_type a () const {return a_;}
        result_type b () const {return b_;}

        friend inline bool operator== (
                const param_type &param1, const param_type &param2)
        {
            if (param1.a_ < param2.a_ || param1.a_ > param2.a_)
                return false;
            if (param1.b_ < param2.b_ || param1.b_ > param2.b_)
                return false;
            return true;
        }

        friend inline bool operator!= (
                const param_type param1, const param_type param2)
        {return !(param1 == param2);}

        template <typename CharT, typename Traits>
        friend inline std::basic_ostream<CharT, Traits> &operator<< (
                std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            if (!os.good())
                return os;

            os << param.a_ << ' ' << param.b_;

            return os;
        }

        template <typename CharT, typename Traits>
        friend inline std::basic_istream<CharT, Traits> &operator>> (
                std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            if (!is.good())
                return is;

            result_type a = 1;
            result_type b = 0;
            is >> std::ws >> a;
            is >> std::ws >> b;

            if (is.good()) {
                if (a <= b) {
                    param.a_ = a;
                    param.b_ = b;
                } else {
                    is.setstate(std::ios_base::failbit);
                }
            }

            return is;
        }

        private :

        result_type a_;
        result_type b_;
    }; // class param_type

    explicit UniformRealDistribution (result_type a = 0, result_type b = 1) :
        a_(a), b_(b)
    {VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(a_, b_);}

    explicit UniformRealDistribution (const param_type &param) :
        a_(param.a()), b_(param.b())
    {VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_PARAM_CHECK(a_, b_);}

    param_type param () const {return param_type(a_, b_);}

    void param (const param_type &param)
    {
        a_ = param.a();
        b_ = param.b();
    }

    void reset () const {}

    result_type a () const {return a_;}
    result_type b () const {return b_;}
    result_type min VSMC_MNE () const {return a_;}
    result_type max VSMC_MNE () const {return b_;}

    /// \brief Generate uniform random variates
    ///
    /// \tparam Eng Requirement:
    /// ~~~{.cpp}
    /// Eng::min() == 0 && (
    /// Eng::max() == std::numeric_limits<uint32_t>::max() ||
    /// Eng::max() == std::numeric_limits<uint64_t>::max()
    /// )
    /// ~~~
    template <typename Eng>
    result_type operator() (Eng &eng) const
    {
        return internal::UniformRealDistributionOp<
            FPType, Left, Right, Eng, MinMaxIsConstexpr
            >::uint2fp(eng) * (b_ - a_) + a_;
    }

    friend inline bool operator== (
            const UniformRealDistribution<
            FPType, Left, Right, MinMaxIsConstexpr> &runif1,
            const UniformRealDistribution<
            FPType, Left, Right, MinMaxIsConstexpr> &runif2)
    {
        if (runif1.a_ < runif2.a_ ||runif1.a_ > runif1.a_)
            return false;
        if (runif1.b_ < runif2.b_ ||runif1.b_ > runif1.b_)
            return false;
        return true;
    }

    friend inline bool operator!= (
            const UniformRealDistribution<
            FPType, Left, Right, MinMaxIsConstexpr> &runif1,
            const UniformRealDistribution<
            FPType, Left, Right, MinMaxIsConstexpr> &runif2)
    {return !(runif1 == runif2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const UniformRealDistribution<
            FPType, Left, Right, MinMaxIsConstexpr> &runif)
    {
        if (!os.good())
            return os;

        os << runif.a_ << ' ' << runif.b_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            UniformRealDistribution<
            FPType, Left, Right, MinMaxIsConstexpr> &runif)
    {
        if (!is.good())
            return is;

        result_type a = 1;
        result_type b = 0;
        is >> std::ws >> a;
        is >> std::ws >> b;
        if (is.good()) {
            if (a <= b) {
                runif.a_ = a;
                runif.b_ = b;
            } else {
                is.setstate(std::ios_base::failbit);
            }
        }

        return is;
    }

    private :

    result_type a_;
    result_type b_;
}; // class UniformRealDistribution

} // namespace vsmc

#endif // VSMC_RNG_UNIFORM_REAL_DISTRIBUTION_HPP
