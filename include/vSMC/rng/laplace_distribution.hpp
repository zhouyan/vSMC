#ifndef V_SMC_RNG_LAPLACE_DISTRIBUTION_HPP
#define V_SMC_RNG_LAPLACE_DISTRIBUTION_HPP

#include <vSMC/rng/common.hpp>

namespace vSMC { namespace rng {

/// \brief Laplace distribution
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
        location_(location), scale_(scale)
    {
        assert(scale_ >= 0);
    }

    explicit laplace_distribution (const param_type &parm) :
        location_(parm.location()), scale_(parm.scale())
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

    RealType min V_SMC_PREVENT_MIN_MAX () const
    {
        return -std::numeric_limits<RealType>::infinity();
    }

    RealType max V_SMC_PREVENT_MIN_MAX () const
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

    void reset () {}

    template <typename URNG>
    result_type operator() (URNG &urng)
    {
        using std::abs;
        using std::log;

        RealType u = uniform_real_distribution<RealType>()(urng) - 0.5;
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

    // FIXME
    // /** Writes a \c laplace_distribution to a \c std::ostream. */
    // BOOST_RANDOM_DETAIL_OSTREAM_OPERATOR(os, laplace_distribution, nd)
    // {
    //     os << nd._location << " " << nd._scale << " ";
    //     return os;
    // }

    // /** Reads a \c laplace_distribution from a \c std::istream. */
    // BOOST_RANDOM_DETAIL_ISTREAM_OPERATOR(is, laplace_distribution, nd)
    // {
    //     is >> std::ws >> nd._location >> std::ws >> nd._scale;
    //     return is;
    // }

    bool operator== (laplace_distribution<RealType> &other)
    {
        return location_ == other.location_ && scale_ == other.scale_;
    }

    bool operator!= (laplace_distribution<RealType> &other)
    {
        return location_ != other.location_ || scale_ != other.scale_;
    }

    private :

    RealType location_;
    RealType scale_;
}; // class laplace_distribution

} } // namespace vSMC::rng

#endif // V_SMC_RNG_LAPLACE_DISTRIBUTION_HPP
