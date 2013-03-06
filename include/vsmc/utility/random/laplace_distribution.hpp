#ifndef VSMC_UTILITY_RANDOM_LAPALCE_DISTRIBUTION_HPP
#define VSMC_UTILITY_RANDOM_LAPALCE_DISTRIBUTION_HPP

#include <vsmc/utility/random/common.hpp>

namespace vsmc { namespace cxx11 {

template <typename RealType = double>
class laplace_distribution
{
    public :

    typedef RealType result_type;

    class param_type
    {
        public :

        typedef RealType result_type;
        typedef laplace_distribution<RealType> distribution_type;

        param_type (RealType location = 0, RealType scale = 1) :
            location_(location), scale_(scale)
        {
            VSMC_RUNTIME_ASSERT((scale > 0),
                    ("**vsmc::cxx11::laplace_distribution** INVALID scale"));
        }

        RealType location () const {return location_;}

        RealType scale () const {return scale_;}

        private :

        RealType location_;
        RealType scale_;
    }; // param_type

    laplace_distribution (RealType location = 0, RealType scale = 1) :
        param_(param_type(location, scale)),
        runif_(static_cast<RealType>(-0.5), static_cast<RealType>(0.5)) {}

    laplace_distribution (const param_type &par) :
        param_(par),
        runif_(static_cast<RealType>(-0.5), static_cast<RealType>(0.5)) {}

    template <typename Eng>
    RealType operator() (Eng &eng)
    {
        using std::log;

        RealType u = runif_(eng);

        return u > 0 ?
            param_.location() - param_.scale() * log(1 - 2 * u):
            param_.location() + param_.scale() * log(1 + 2 * u);
    }

    void reset () {};

    RealType location () const {return param_.location();}

    RealType scale () const {return param_.scale();}

    param_type param () const {return param_;}

    void param (const param_type &par) {param_ = par;}

    static VSMC_CONSTEXPR RealType min VSMC_MACRO_NO_EXPANSION ()
    {return std::numeric_limits<RealType>::min VSMC_MACRO_NO_EXPANSION ();}

    static VSMC_CONSTEXPR RealType max VSMC_MACRO_NO_EXPANSION ()
    {return std::numeric_limits<RealType>::max VSMC_MACRO_NO_EXPANSION ();}

    private :

    param_type param_;
    uniform_real_distribution<RealType> runif_;
}; // class laplace_distribution

template <typename RealType>
inline bool operator== (
        const typename laplace_distribution<RealType>::param_type &p1,
        const typename laplace_distribution<RealType>::param_type &p2)
{return p1.location() == p2.location() && p1.scale() == p2.scale();}

template <typename RealType>
inline bool operator!= (
        const typename laplace_distribution<RealType>::param_type &p1,
        const typename laplace_distribution<RealType>::param_type &p2)
{return !(p1 == p2);}

template <typename RealType>
inline bool operator== (
        const laplace_distribution<RealType> &d1,
        const laplace_distribution<RealType> &d2)
{return d1.location() == d2.location() && d1.scale() == d2.scale();}

template <typename RealType>
inline bool operator!= (
        const laplace_distribution<RealType> &d1,
        const laplace_distribution<RealType> &d2)
{return !(d1 == d2);}

template <typename CharT, typename Traits, typename RealType>
inline std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os,
        const laplace_distribution<RealType> &dist)
{
    internal::SaveIOFlags<CharT, Traits> flags(os);
    os.flags(std::ios_base::dec |
            std::ios_base::left |
            std::ios_base::fixed |
            std::ios_base::scientific);
    CharT sp = os.widen(' ');
    os.fill(sp);
    os << dist.location() << sp << dist.scale();

    return os;
}

template <typename CharT, typename Traits, typename RealType>
inline std::basic_istream<CharT, Traits> &operator<< (
        std::basic_istream<CharT, Traits> &is,
        const laplace_distribution<RealType> &dist)
{
    internal::SaveIOFlags<CharT, Traits> flags(is);
    is.flags(std::ios_base::dec | std::ios_base::skipws);
    RealType p1;
    RealType p2;
    is >> p1 >> p2;
    if (!is.fail())
        dist.param(
                typename laplace_distribution<RealType>::param_type(p1, p2));

    return is;
}

} } // namespace vsmc::cxx11

#endif // VSMC_UTILITY_RANDOM_LAPALCE_DISTRIBUTION_HPP
