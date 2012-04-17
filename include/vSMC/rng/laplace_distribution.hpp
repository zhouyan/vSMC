#ifndef V_SMC_RNG_LAPLACE_DISTRIBUTION_HPP
#define V_SMC_RNG_LAPLACE_DISTRIBUTION_HPP

#include <limits>
#include <cassert>
#include <cmath>
#include <boost/math/special_functions/log1p.hpp>
#include <boost/random/uniform_01.hpp>

namespace vSMC {

template <typename RealType = double>
class laplace_distribution
{
    public :

    typedef RealType input_type;
    typedef RealType result_type;

    class param_type
    {
        public :

        typedef laplace_distribution distribution_type;

        explicit param_type (RealType location = 0, RealType scale = 1) :
            location_(location), scale_(scale) {}

        RealType location () const {return location_;}
        RealType scale () const {return scale_;}

        private :

        RealType location_;
        RealType scale_;
    }; // class param_type

    explicit laplace_distribution (
            RealType location = 0, RealType  scale = 1) :
        location_(location), scale_(scale)
    {
        assert(scale_ > 0);
    }

    explicit laplace_distribution (const param_type &param) :
        location_(param.location()), scale_(param.scale())
    {
        assert(scale_ > 0);
    }

    RealType min () const {return -std::numeric_limits<RealType>::infinity();}
    RealType max () const {return std::numeric_limits<RealType>::infinity();}
    RealType location () const {return location_;}
    RealType scale () const {return scale_;}
    param_type param () const {return param_type(location_, scale_);}
    void param (const param_type &param)
    {
        location_ = param.location();
        scale_ = param.scale();
    }

    void reset () {}

    template <typename Engine>
    result_type operator() (Engine &eng)
    {
        double u = boost::random::uniform_01<RealType>()(eng) - 0.5;
        double r = -2 * std::abs(u);

        r = boost::math::log1p(r);
        r = u > 0 ? r : -r;

        return location_ - scale_ * r;
    }

    template <typename Engine>
    result_type operator() (Eigen &eng, const param_type &param)
    {
        return normal_distribution(param)(eng);
    }

    private :

    RealType location_;
    RealType scale_;
}; // class laplace_distribution

} // namespace vSMC

#endif // V_SMC_RNG_LAPLACE_DISTRIBUTION_HPP
