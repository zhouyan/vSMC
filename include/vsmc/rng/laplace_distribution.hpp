#ifndef VSMC_RNG_LAPLACE_DISTRIBUTION_HPP
#define VSMC_RNG_LAPLACE_DISTRIBUTION_HPP

#include <vsmc/rng/common.hpp>

namespace vsmc { namespace cxx11 {

/// \brief Laplace distribution
/// \ingroup RNG
///
/// \tparam RealType A floating point type
///
/// Instantiations of class template laplace_distribution model a
/// random distribiton. Such a distribution produces random numbers \c x
/// distributed with probability density function
/// \f$\displaystyle p(x) =
///   \frac{1}{2b}\exp(-|x-\mu|/b)
/// \f$,
/// where location and scale are the paramters of the distributions.
template <typename RealType = double>
class laplace_distribution
{
    public :

    typedef RealType input_type;
    typedef RealType result_type;

    class param_type
    {
        public :

        explicit param_type (RealType location = 0, RealType scale = 1) :
            location_(location), scale_(scale) {}

        RealType location () const
        {
            return location_;
        }

        RealType scale () const
        {
            return scale_;
        }

        private :

        RealType location_;
        RealType scale_;
    }; // class param_type

    explicit laplace_distribution (RealType location = 0, RealType scale = 1) :
        location_(location), scale_(scale), runif_(0, 1)
    {
        assert(scale_ >= 0);
    }

    explicit laplace_distribution (const param_type &parm) :
        location_(parm.location()), scale_(parm.scale()), runif_(0, 1)
    {
        assert(scale_ >= 0);
    }

    RealType location () const
    {
        return location_;
    }

    RealType scale () const
    {
        return scale_;
    }

    RealType min VSMC_PREVENT_MIN_MAX () const
    {
        return -std::numeric_limits<RealType>::infinity();
    }

    RealType max VSMC_PREVENT_MIN_MAX () const
    {
        return std::numeric_limits<RealType>::infinity();
    }

    param_type param () const
    {
        return param_type(location_, scale_);
    }

    void param (const param_type &parm)
    {
        location_ = parm.location();
        scale_ = parm.scale();
    }

    void reset ()
    {
        runif_.reset();
    }

    template <typename URNG>
    result_type operator() (URNG &urng)
    {
        using std::abs;
        using std::log;

        RealType u = runif_(urng) - 0.5;
        RealType r = -2 * std::abs(u);

        r = log(1 + r);
        r = u > 0 ? r : -r;

        return location_ - scale_ * r;
    }

    template <typename URNG>
    result_type operator() (URNG &urng, const param_type &parm)
    {
        return laplace_distribution(parm)(urng);
    }

    private :

    RealType location_;
    RealType scale_;
    uniform_real_distribution<RealType> runif_;
}; // class laplace_distribution

template<typename RealType>
bool operator== (
        laplace_distribution<RealType> &lhs,
        laplace_distribution<RealType> &rhs)
{
    return lhs.location() == rhs.location() && lhs.scale() == rhs.scale();
}

template<typename RealType>
bool operator!= (
        laplace_distribution<RealType> &lhs,
        laplace_distribution<RealType> &rhs)
{
    return lhs.location() != rhs.location() || lhs.scale() != rhs.scale();
}

template<typename T, typename CharT, typename Traits, typename RealType>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os,
        laplace_distribution<RealType> &dist)
{
    os << dist.location() << " " << dist.scale() << " ";

    return os;
}

template<typename T, typename CharT, typename Traits, typename RealType>
std::basic_istream<CharT, Traits> &operator<< (
        std::basic_istream<CharT, Traits> &is,
        laplace_distribution<RealType> &dist)
{
    RealType location = 0;
    RealType scale = 1;
    is >> std::ws >> location >> std::ws  >> scale;
    dist.param(typename laplace_distribution<RealType>::param_type(
                location, scale));

    return is;
}

} } // namespace vsmc::cxx11

#endif // VSMC_RNG_LAPLACE_DISTRIBUTION_HPP
