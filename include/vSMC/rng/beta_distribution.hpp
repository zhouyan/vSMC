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

    // FIXME
    // /// Writes a @c beta_distribution to a @c std::ostream.
    // BOOST_RANDOM_DETAIL_OSTREAM_OPERATOR(os, beta_distribution, nd)
    // {
    //     os << nd._shape1 << " " << nd._shape2 << " ";
    //     return os;
    // }

    // /// Reads a @c beta_distribution from a @c std::istream.
    // BOOST_RANDOM_DETAIL_ISTREAM_OPERATOR(is, beta_distribution, nd)
    // {
    //     is >> std::ws >> nd._shape1 >> std::ws >> nd._shape2;
    //     return is;
    // }

    bool operator== (beta_distribution<RealType> &other)
    {
        return shape1_ == other.shape1_ && shape2_ == other.shape2_;
    }

    bool operator!= (beta_distribution<RealType> &other)
    {
        return shape1_ != other.shape1_ || shape2_ != other.shape2_;
    }

    private :

    RealType shape1_;
    RealType shape2_;
}; // class beta_distribution

} } // namespace vSMC::rng

#endif // V_SMC_RNG_BETA_DISTRIBUTION_HPP
