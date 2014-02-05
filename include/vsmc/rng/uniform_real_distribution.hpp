#ifndef VSMC_UNIFORM_REAL_DISTRIBUTION_HPP
#define VSMC_UNIFORM_REAL_DISTRIBUTION_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/u01.h>

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MIN(eng_min) \
    VSMC_RUNTIME_ASSERT((eng_min == 0),                                      \
            ("**vsmc::UniformRealDistribution::operator()** "                \
             "ENGINE MEMBER FUNCTION min() RETURN A VALUE OTHER THAN ZERO"))

#define VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MAX \
    VSMC_RUNTIME_ASSERT(false,                                               \
            ("**vsmc::UniformRealDistribution::operator()** "                \
             "ENGINE MEMBER FUNCTION min() RETURN A VALUE OTHER THAN "       \
             "THE MAXIMUM OF uint32_t OR uint64_t"))

namespace vsmc {

namespace traits {

template<uint64_t, uint64_t> struct IntegerRangeTypeTrait;

template<>
struct IntegerRangeTypeTrait<0, ~((uint32_t)0)> {typedef uint32_t type;};

template<>
struct IntegerRangeTypeTrait<0, ~((uint64_t)0)> {typedef uint64_t type;};

} // namespace vsmc::traits

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

        typedef UniformRealDistribution<FPType, Left, Right>
            distribution_type;

        explicit param_type (result_type a = 0, result_type b = 1) :
            a_(a), b_(b) {}

        result_type a () const {return a_;}
        result_type b () const {return b_;}

        friend inline bool operator== (
                const param_type &param1, const param_type &param2)
        {return (param1.a() == param2.a()) && (param1.b() == param2.b());}

        friend inline bool operator!= (
                const param_type param1, const param_type param2)
        {return (param1.a() != param2.a()) || (param1.b() != param2.b());}

        template <typename CharT, typename Traits>
        friend inline std::basic_ostream<CharT, Traits> &operator<< (
                std::basic_ostream<CharT, Traits> &os, const param_type &param)
        {
            os << param.a() << ' ' << param.b();

            return os;
        }

        template <typename CharT, typename Traits>
        friend inline std::basic_istream<CharT, Traits> &operator>> (
                std::basic_istream<CharT, Traits> &is, param_type &param)
        {
            result_type a;
            result_type b;
            if (is >> a >> std::ws >> b) {
                if (a <= b)
                    param = param_type(param_type(a, b));
                else
                    is.setstate(std::ios_base::failbit);
            }

            return is;
        }

        private :

        result_type a_;
        result_type b_;
    }; // class param_type

    explicit UniformRealDistribution (result_type a = 0, result_type b = 1) :
        a_(a), b_(b) {}

    UniformRealDistribution (const param_type &param) :
        a_(param.a()), b_(param.b()) {}

    param_type param () const {return param_type(a_, b_);}

    void param (const param_type &param)
    {
        a_ = param.a();
        b_ = param.b();
    }

    void reset () const {}

    result_type a () const {return a_;}
    result_type b () const {return b_;}
    result_type min VSMC_MACRO_NO_EXPANSION () const {return a_;}
    result_type max VSMC_MACRO_NO_EXPANSION () const {return b_;}

    template <typename Eng>
    result_type operator() (Eng &eng) const
    {
        typedef cxx11::integral_constant<std::size_t, sizeof(result_type)>
            fp_bits;

#if VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
        typedef typename traits::IntegerRangeTypeTrait<
            Eng::min VSMC_MACRO_NO_EXPANSION (),
            Eng::max VSMC_MACRO_NO_EXPANSION ()>::type eng_uint_t;
        typedef cxx11::integral_constant<std::size_t, sizeof(eng_uint_t)>
            u_bits;

        result_type u = u01(static_cast<eng_uint_t>(eng()), Left(), Right(),
                u_bits(), fp_bits());
#else // VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
        static VSMC_CONSTEXPR const uint64_t eng_min = static_cast<uint64_t>(
                eng.min VSMC_MACRO_NO_EXPANSION ());
        VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MIN(eng_min);

        result_type u = 0;
        static VSMC_CONSTEXPR const uint64_t eng_max = static_cast<uint64_t>(
                eng.max VSMC_MACRO_NO_EXPANSION ());
        switch (eng_max) {
            case uint32_t_max_ :
                u = u01(static_cast<uint32_t>(eng()), Left(), Right(),
                        u32(), fp_bits());
                break;
            case uint64_t_max_ :
                u = u01(static_cast<uint64_t>(eng()), Left(), Right(),
                        u64(), fp_bits());
                break;
            default :
                VSMC_RUNTIME_ASSERT_RNG_UNIFORM_REAL_DISTRIBUTION_ENG_MAX;
        }
#endif // VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX

        return u * (b_ - a_) + a_;
    }

    private :

    result_type a_;
    result_type b_;

#if !VSMC_HAS_CXX11LIB_RANDOM_CONSTEXPR_MINMAX
    static VSMC_CONSTEXPR const uint64_t uint32_t_max_ = ~((uint32_t)0);
    static VSMC_CONSTEXPR const uint64_t uint64_t_max_ = ~((uint64_t)0);
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

/// \brief UniformRealDistribution operator==
/// \ingroup RNG
template <typename FPType, typename Left, typename Right>
inline bool operator== (
        const UniformRealDistribution<FPType, Left, Right> &runif1,
        const UniformRealDistribution<FPType, Left, Right> &runif2)
{
    return (runif1.a() == runif2.a()) && (runif1.b() == runif2.b());
}

/// \brief UniformRealDistribution operator!=
/// \ingroup RNG
template <typename FPType, typename Left, typename Right>
inline bool operator!= (
        const UniformRealDistribution<FPType, Left, Right> &runif1,
        const UniformRealDistribution<FPType, Left, Right> &runif2)
{
    return (runif1.a() != runif2.a()) || (runif1.b() != runif2.b());
}

/// \brief UniformRealDistribution operator<<
/// \ingroup RNG
template <typename CharT, typename Traits,
         typename FPType, typename Left, typename Right>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os,
        const UniformRealDistribution<FPType, Left, Right> &runif)
{
    os << runif.a() << ' ' << runif.b();

    return os;
}

/// \brief UniformRealDistribution operator>>
/// \ingroup RNG
template <typename CharT, typename Traits,
         typename FPType, typename Left, typename Right>
inline std::basic_istream<CharT, Traits> &operator>> (
        std::basic_istream<CharT, Traits> &is,
        UniformRealDistribution<FPType, Left, Right> &runif)
{
    typedef typename UniformRealDistribution<FPType, Left, Right>::param_type
        param_type;

    FPType a;
    FPType b;
    if (is >> a >> std::ws >> b) {
        if (a <= b)
            runif.param(param_type(a, b));
        else
            is.setstate(std::ios_base::failbit);
    }

    return is;
}

} // namespace vsmc

#endif // VSMC_UNIFORM_REAL_DISTRIBUTION_HPP
