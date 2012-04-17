#ifndef V_SMC_RNG_BETA_DISTRIBUTION_HPP
#define V_SMC_RNG_BETA_DISTRIBUTION_HPP

#include <limits>
#include <cassert>
#include <cmath>
#include <boost/math/special_functions/log1p.hpp>
#include <boost/random/gamma_distribuiton.hpp>

namespace vSMC {

template <typename RealType = double>
class beta_distribution
{
    public :

    typedef RealType input_type;
    typedef RealType result_type;

    class param_type
    {
        public :

        typedef beta_distribution distribution_type;

        explicit param_type (RealType shape1 = 0, RealType shape2 = 1) :
            shape1_(shape1), shape2_(shape2) {}

        RealType shape1 () const {return shape1_;}
        RealType shape2 () const {return shape2_;}

        private :

        RealType shape1_;
        RealType shape2_;
    }; // class param_type

    explicit beta_distribution (RealType shape1 = 0, RealType  shape2 = 1) :
        shape1_(shape1), shape2_(shape2)
    {
        assert(shape1_ > 0);
        assert(shape2_ > 0);
    }

    explicit beta_distribution (const param_type &param) :
        shape1_(param.shape1()), shape2_(param.shape2())
    {
        assert(shape1_ > 0);
        assert(shape2_ > 0);
    }

    RealType min () const {return 0;}
    RealType max () const {return 1;}
    RealType shape1 () const {return shape1_;}
    RealType shape2 () const {return shape2_;}
    param_type param () const {return param_type(shape1_, shape2_);}
    void param (const param_type &param)
    {
        shape1_ = param.shape1();
        shape2_ = param.shape2();
    }

    void reset () {}

    template <typename Engine>
    result_type operator() (Engine &eng)
    {
        RealType x =
            boost::random::gamma_distribution<RealType>(shape1_, 1)(eng);
        RealType y =
            boost::random::gamma_distribution<RealType>(shape2_, 1)(eng);

        return x / (x + y);
    }

    template <typename Engine>
    result_type operator() (Eigen &eng, const param_type &param)
    {
        return beta_distribution(param)(eng);
    }

    private :

    RealType shape1_;
    RealType shape2_;
}; // class beta_distribution

} // namespace vSMC

#endif // V_SMC_RNG_BETA_DISTRIBUTION_HPP
