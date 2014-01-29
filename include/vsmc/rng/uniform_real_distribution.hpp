#ifndef VSMC_UNIFORM_REAL_DISTRIBUTION_HPP
#define VSMC_UNIFORM_REAL_DISTRIBUTION_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/u01.h>

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
///
/// Also note that the results of RNG engine's `min` and `max` member functions
/// has to be compile time constants and can thus be used as template
/// argumments. Because the range of the `result_type` of many RNG engines
/// (e.g., `std::mt19937`) is not necessary the same as the between of `min`
/// and `max`. If UniformRealDistribution::operator() cannot determine the
/// proper type that holds the range between `min` and `max` at compile time,
/// then it has to resolve to (much) slower runtime decisions. Though
/// considering that cost of generating random integers is likely to be much
/// higher than the cost of runtime decision of the range, it might be just
/// easier to use functions defined in `<vsmc/rng/u01.h>`.
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
        typedef typename Eng::result_type eng_result_type;
        typedef typename traits::IntegerRangeTypeTrait<
            Eng::min VSMC_MACRO_NO_EXPANSION (),
            Eng::max VSMC_MACRO_NO_EXPANSION ()>::type eng_uint;
        typedef cxx11::integral_constant<std::size_t, sizeof(eng_uint)>
            u_bits;
        typedef cxx11::integral_constant<std::size_t, sizeof(result_type)>
            fp_bits;

        result_type u = u01(static_cast<eng_uint>(eng()), Left(), Right(),
                u_bits(), fp_bits());

        return u * (b_ - a_) + a_;
    }

    private :

    result_type a_;
    result_type b_;

    static const uint32_t uint32_t_max_ = ~((uint32_t)0);
    static const uint64_t uint64_t_max_ = ~((uint64_t)0);

    static float u01(uint32_t i, Open, Open, u32, f24)
    {return ::u01_open_open_32_24(i);}

    static float u01(uint32_t i, Open, Closed, u32, f24)
    {return ::u01_open_closed_32_24(i);}

    static float u01(uint32_t i, Closed, Open, u32, f24)
    {return ::u01_closed_open_32_24(i);}

    static float u01(uint32_t i, Closed, Closed, u32, f24)
    {return ::u01_closed_closed_32_24(i);}

    static float u01(uint32_t i, Open, Open, u32, f53)
    {return ::u01_open_open_32_53(i);}

    static float u01(uint32_t i, Open, Closed, u32, f53)
    {return ::u01_open_closed_32_53(i);}

    static float u01(uint32_t i, Closed, Open, u32, f53)
    {return ::u01_closed_open_32_53(i);}

    static float u01(uint32_t i, Closed, Closed, u32, f53)
    {return ::u01_closed_closed_32_53(i);}

    static float u01(uint64_t i, Open, Open, u64, f53)
    {return ::u01_open_open_64_53(i);}

    static float u01(uint64_t i, Open, Closed, u64, f53)
    {return ::u01_open_closed_64_53(i);}

    static float u01(uint64_t i, Closed, Open, u64, f53)
    {return ::u01_closed_open_64_53(i);}

    static float u01(uint64_t i, Closed, Closed, u64, f53)
    {return ::u01_closed_closed_64_53(i);}
}; // class UniformRealDistributionBase

} // namespace vsmc

#endif // VSMC_UNIFORM_REAL_DISTRIBUTION_HPP
