#ifndef V_SMC_RNG_BETA_DISTRIBUTION_HPP
#define V_SMC_RNG_BETA_DISTRIBUTION_HPP

#include <vSMC/rng/common.hpp>

namespace vSMC { namespace rng {

/// \brief Beta distribution
///
/// Instantiations of class template beta_distribution model a random
/// distribiton. Such a distribution produces random numbers \c x distributed
/// with probability density function
/// \f$\displaystyle p(x) =
///   \frac{1}{B(\alpha,\beta)}x^{\alpha-1}(1-x)^{\beta-1}
/// \f$,
/// where shape1 and shape2 are the paramters of the distributions.
template <typename RealType = double>
class beta_distribution
{
    public :

    typedef RealType input_type;
    typedef RealType result_type;

    class param_type
    {
        public :

        explicit param_type (RealType shape1 = 1, RealType shape2 = 1) :
            shape1_(shape1), shape2_(shape2) {}

        RealType shape1 () const
        {
            return shape1_;
        }

        RealType shape2 () const
        {
            return shape2_;
        }

        private :

        RealType shape1_;
        RealType shape2_;
    };

    explicit beta_distribution (RealType shape1 = 1, RealType shape2 = 1) :
        shape1_(shape1), shape2_(shape2)
    {
        assert(shape1_ > 0);
        assert(shape2_ > 0);
    }

    explicit beta_distribution (const param_type &parm) :
        shape1_(parm.shape1()), shape2_(parm.shape2())
    {
        assert(shape1_ > 0);
        assert(shape2_ > 0);
    }

    RealType shape1 () const
    {
        return shape1_;
    }

    RealType shape2 () const
    {
        return shape2_;
    }

    RealType min V_SMC_PREVENT_MIN_MAX () const
    {
        return 0;
    }

    RealType max V_SMC_PREVENT_MIN_MAX () const
    {
        return 1;
    }

    param_type param () const
    {
        return param_type(shape1_, shape2_);
    }

    void param (const param_type &parm)
    {
        shape1_ = parm.shape1();
        shape2_ = parm.shape2();
    }

    void reset () {}

    template <typename URNG>
    result_type operator() (URNG &urng)
    {
        RealType x = gamma_distribution<RealType>(shape1_, 1)(urng);
        RealType y = gamma_distribution<RealType>(shape2_, 1)(urng);

        return x / (x + y);
    }

    template <typename URNG>
    result_type operator() (URNG &urng, const param_type &parm)
    {
        return beta_distribution(parm)(urng);
    }

    private :

    RealType shape1_;
    RealType shape2_;
}; // class beta_distribution

template<typename RealType>
bool operator== (
        beta_distribution<RealType> &lhs,
        beta_distribution<RealType> &rhs)
{
    return lhs.shape1() == rhs.shape1() && lhs.shape2() == rhs.shape2();
}

template<typename RealType>
bool operator!= (
        beta_distribution<RealType> &lhs,
        beta_distribution<RealType> &rhs)
{
    return lhs.shape1() != rhs.shape1() || lhs.shape2() != rhs.shape2();
}

template<typename T, typename CharT, typename Traits, typename RealType>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os,
        beta_distribution<RealType> &dist)
{
    os << dist.shape1() << " " << dist.shape2() << " ";

    return os;
}

template<typename T, typename CharT, typename Traits, typename RealType>
std::basic_istream<CharT, Traits> &operator<< (
        std::basic_istream<CharT, Traits> &is,
        beta_distribution<RealType> &dist)
{
    RealType shape1 = 0;
    RealType shape2 = 1;
    is >> std::ws >> shape1 >> std::ws >> shape2;
    dist.param(typename beta_distribution<RealType>::param_type(
                shape1, shape2));

    return is;
}

} } // namespace vSMC::rng

#endif // V_SMC_RNG_BETA_DISTRIBUTION_HPP
