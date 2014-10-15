//============================================================================
// include/vsmc/rng/uniform_real_distribution.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UNIFORM_REAL_DISTRIBUTION_HPP
#define VSMC_UNIFORM_REAL_DISTRIBUTION_HPP

#include <vsmc/rng/internal/common.hpp>
#include <vsmc/rng/u01.h>

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUITON_PARAM_CHECK(a, b) \
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

template<uint64_t, uint64_t> struct UniformRealDistributionFullRangeIntgerType;

template<>
struct UniformRealDistributionFullRangeIntgerType<0,
    static_cast<uint64_t>(static_cast<uint32_t>(~(static_cast<uint32_t>(0))))>
{typedef uint32_t type;};

template<>
struct UniformRealDistributionFullRangeIntgerType<0,
    static_cast<uint64_t>(~(static_cast<uint64_t>(0)))>
{typedef uint64_t type;};

} // namespace vsmc::interal

/// \brief Parameter type for open interval
/// \ingroup RNG
struct Open {};

/// \brief Parameter type for closed interval
/// \ingroup RNG
struct Closed {};

/// \brief Uniform real distribution with variants open/closed variants
/// \ingroup RNG
///
/// \details
/// This distribution is almost identical to C++11
/// `std::uniform_real_distribution`. But it differs in two important aspects
/// - It allows the interval to be either open or closed on both sides.
/// - It requires that the uniform random number generator to produce integers
///   on the full range of either `uint32_t` or `uint64_t`.
template <typename FPType, typename Left, typename Right>
class UniformRealDistribution
{
    private :

    typedef cxx11::integral_constant<std::size_t, sizeof(float)> f24;
    typedef cxx11::integral_constant<std::size_t, sizeof(double)> f53;
    typedef cxx11::integral_constant<std::size_t, sizeof(uint32_t)> u32;
    typedef cxx11::integral_constant<std::size_t, sizeof(uint64_t)> u64;

    public :

    typedef FPType result_type;

    class param_type
    {
        public :

        typedef FPType result_type;

        typedef UniformRealDistribution<FPType, Left, Right> distribution_type;

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
    {VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUITON_PARAM_CHECK(a_, b_);}

    UniformRealDistribution (const param_type &param) :
        a_(param.a()), b_(param.b())
    {VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUITON_PARAM_CHECK(a_, b_);}

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
    result_type operator() (Eng &eng)
    {
#if VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
	operator()(eng, cxx11::true_type());
#else
	operator()(eng, cxx11::false_type());
#endif
    }

    /// \brief static dispatch based on `Eng::min()` and `Eng::max()`, both
    /// need to be constant expression
    template <typename Eng>
    result_type operator() (Eng &eng, cxx11::true_type) const
    {
        typedef cxx11::integral_constant<std::size_t, sizeof(result_type)>
            fbits;
        typedef typename internal::UniformRealDistributionFullRangeIntgerType<
            Eng::min VSMC_MNE (), Eng::max VSMC_MNE ()>::type eng_uint_t;
        typedef cxx11::integral_constant<std::size_t, sizeof(eng_uint_t)>
            ubits;

        result_type u = u01(static_cast<eng_uint_t>(eng()), Left(), Right(),
                ubits(), fbits());

        return u * (b_ - a_) + a_;
    }

    /// \brief Dynamic dispatch based on `eng.min()` and `eng.max()`
    template <typename Eng>
    result_type operator() (Eng &eng, cxx11::false_type) const
    {
        typedef cxx11::integral_constant<std::size_t, sizeof(result_type)>
            fbits;

        static const uint64_t eng_max = static_cast<uint64_t>(
                eng.max VSMC_MNE ());

        VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MIN(
                (eng.min VSMC_MNE ()));
        VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MAX(eng_max);

        result_type u = 0;
        switch (eng_max) {
            case uint32_t_max_ :
                u = u01(static_cast<uint32_t>(eng()), Left(), Right(),
                        u32(), fbits());
                break;
            case uint64_t_max_ :
                u = u01(static_cast<uint64_t>(eng()), Left(), Right(),
                        u64(), fbits());
                break;
            default :
                return 0;
        }

        return u * (b_ - a_) + a_;
    }

    friend inline bool operator== (
            const UniformRealDistribution<FPType, Left, Right> &runif1,
            const UniformRealDistribution<FPType, Left, Right> &runif2)
    {
        if (runif1.a_ < runif2.a_ ||runif1.a_ > runif1.a_)
            return false;
        if (runif1.b_ < runif2.b_ ||runif1.b_ > runif1.b_)
            return false;
        return true;
    }

    friend inline bool operator!= (
            const UniformRealDistribution<FPType, Left, Right> &runif1,
            const UniformRealDistribution<FPType, Left, Right> &runif2)
    {return !(runif1 == runif2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const UniformRealDistribution<FPType, Left, Right> &runif)
    {
        if (!os.good())
            return os;

        os << runif.a_ << ' ' << runif.b_;

        return os;
    }

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            UniformRealDistribution<FPType, Left, Right> &runif)
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

#if !VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
    static VSMC_CONSTEXPR const uint64_t uint32_t_max_ = static_cast<uint64_t>(
            static_cast<uint32_t>(~(static_cast<uint32_t>(0))));
    static VSMC_CONSTEXPR const uint64_t uint64_t_max_ = static_cast<uint64_t>(
            ~(static_cast<uint64_t>(0)));
#endif

    static float u01(uint32_t i, Open, Open, u32, f24)
    {return ::u01_open_open_32_24(i);}

    static float u01(uint32_t i, Open, Closed, u32, f24)
    {return ::u01_open_closed_32_24(i);}

    static float u01(uint32_t i, Closed, Open, u32, f24)
    {return ::u01_closed_open_32_24(i);}

    static float u01(uint32_t i, Closed, Closed, u32, f24)
    {return ::u01_closed_closed_32_24(i);}

    static double u01(uint32_t i, Open, Open, u32, f53)
    {return ::u01_open_open_32_53(i);}

    static double u01(uint32_t i, Open, Closed, u32, f53)
    {return ::u01_open_closed_32_53(i);}

    static double u01(uint32_t i, Closed, Open, u32, f53)
    {return ::u01_closed_open_32_53(i);}

    static double u01(uint32_t i, Closed, Closed, u32, f53)
    {return ::u01_closed_closed_32_53(i);}

    static float u01(uint64_t i, Open, Open, u64, f24)
    {return static_cast<float>(::u01_open_open_64_53(i));}

    static float u01(uint64_t i, Open, Closed, u64, f24)
    {return static_cast<float>(::u01_open_closed_64_53(i));}

    static float u01(uint64_t i, Closed, Open, u64, f24)
    {return static_cast<float>(::u01_closed_open_64_53(i));}

    static float u01(uint64_t i, Closed, Closed, u64, f24)
    {return static_cast<float>(::u01_closed_closed_64_53(i));}

    static double u01(uint64_t i, Open, Open, u64, f53)
    {return ::u01_open_open_64_53(i);}

    static double u01(uint64_t i, Open, Closed, u64, f53)
    {return ::u01_open_closed_64_53(i);}

    static double u01(uint64_t i, Closed, Open, u64, f53)
    {return ::u01_closed_open_64_53(i);}

    static double u01(uint64_t i, Closed, Closed, u64, f53)
    {return ::u01_closed_closed_64_53(i);}
}; // class UniformRealDistributionBase

} // namespace vsmc

#endif // VSMC_UNIFORM_REAL_DISTRIBUTION_HPP
