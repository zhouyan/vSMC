#ifndef BOOST_RANDOM_BETA_DISTRIBUTION_HPP
#define BOOST_RANDOM_BETA_DISTRIBUTION_HPP

#include <istream>
#include <iosfwd>
#include <boost/assert.hpp>
#include <boost/static_assert.hpp>
#include <boost/random/detail/config.hpp>
#include <boost/random/detail/operators.hpp>
#include <boost/random/gamma_distribution.hpp>

namespace boost { namespace random {

/**
 * Instantiations of class template beta_distribution model a
 * \random_distribiton. Such a distribution produces random numbers @c x
 * distributed with probability density function
 * \f$\displaystyle p(x) =
 *   \frac{1}{B(\alpha,\beta)}x^{\alpha-1}(1-x)^{\beta-1}
 * \f$,
 * where shape1 and shape2 are the paramters of the distributions.
 */
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

        /**
         * Constructs a @c param_type with a given shape1 and shape2.
         *
         * Requires: shape1 > 0 && shape2 > 0
         */
        explicit param_type (
                RealType shape1_arg = RealType(1.0),
                RealType shape2_arg = RealType(1.0)) :
            _shape1(shape1_arg), _shape2(shape2_arg) {}

        /** Returns the shape1 of the distribution. */
        RealType shape1 () const {return _shape1;}

        /** Returns the shape2 of the distribution. */
        RealType shape2 () const {return _shape2;}

        /** Writes a @c param_type to a @c std::ostream. */
        BOOST_RANDOM_DETAIL_OSTREAM_OPERATOR(os, param_type, parm)
        { os << parm._shape1 << " " << parm._shape2 ; return os; }

        /** Reads a @c param_type from a @c std::istream. */
        BOOST_RANDOM_DETAIL_ISTREAM_OPERATOR(is, param_type, parm)
        { is >> parm._shape1 >> std::ws >> parm._shape2; return is; }

        /** Returns true if the two sets of parameters are the same. */
        BOOST_RANDOM_DETAIL_EQUALITY_OPERATOR(param_type, lhs, rhs)
        { return lhs._shape1 == rhs._shape1 && lhs._shape2 == rhs._shape2; }

        /** Returns true if the two sets of parameters are the different. */
        BOOST_RANDOM_DETAIL_INEQUALITY_OPERATOR(param_type)
        private :

        RealType _shape1;
        RealType _shape2;
    }; // class param_type

    /**
     * Constructs a @c beta_distribution objet. @c shape1 and @c shape2 are
     * the parameters for the distribution.
     *
     * Requires: shape1 > 0 && shape2 > 0
     */
    explicit beta_distribution (
            const RealType &shape1_arg = RealType(1.0),
            const RealType &shape2_arg = RealType(1.0)) :
        _shape1(shape1_arg), _shape2(shape2_arg)
    {
        BOOST_ASSERT(_shape1 > RealType(0.0));
        BOOST_ASSERT(_shape2 > RealType(0.0));
    }

    /**
     * Constructs a @c beta_distribution object from its parameters.
     */
    explicit beta_distribution (const param_type &parm) :
        _shape1(parm.shape1()), _shape2(parm.shape2())
    {
        BOOST_ASSERT(_shape1 > RealType(0.0));
        BOOST_ASSERT(_shape2 > RealType(0.0));
    }

    /** Returns the shape1 of the distribution. */
    RealType shape1 () const {return _shape1;}

    /** Returns the shape2 of the distribution. */
    RealType shape2 () const {return _shape2;}

    /** Returns the smallest value that the distribution can produce. */
    RealType min BOOST_PREVENT_MACRO_SUBSTITUTION () const
    {return RealType(0.0);}

    /** Returns the largest value that the distribution can produce. */
    RealType max BOOST_PREVENT_MACRO_SUBSTITUTION () const
    {return RealType(1.0);}

    /** Returns the parameters of the distribution. */
    param_type param () const {return param_type(_shape1, _shape2);}

    /** Sets the parameters of the distribution. */
    void param (const param_type &parm)
    {
        _shape1 = parm.shape1();
        _shape2 = parm.shape2();
    }

    /**
     * Effects: Subsequent uses of the distribution do not depend
     * on values produced by any engine prior to invoking reset.
     */
    void reset () {}

    /** Returns a beta variate. */
    template <typename Engine>
    result_type operator() (Engine &eng)
    {
        RealType x =
            boost::random::gamma_distribution<RealType>(_shape1, 1)(eng);
        RealType y =
            boost::random::gamma_distribution<RealType>(_shape2, 1)(eng);

        return x / (x + y);
    }

    /** Returns a beta variate with parameters specified by @c param. */
    template <typename URNG>
    result_type operator() (URNG &urng, const param_type &parm)
    {
        return beta_distribution(parm)(urng);
    }

    /** Writes a @c beta_distribution to a @c std::ostream. */
    BOOST_RANDOM_DETAIL_OSTREAM_OPERATOR(os, beta_distribution, nd)
    {
        os << nd._shape1 << " " << nd._shape2 << " ";
        return os;
    }

    /** Reads a @c beta_distribution from a @c std::istream. */
    BOOST_RANDOM_DETAIL_ISTREAM_OPERATOR(is, beta_distribution, nd)
    {
        is >> std::ws >> nd._shape1 >> std::ws >> nd._shape2;
        return is;
    }

    /**
     * Returns true if the two instances of @c beta_distribution will
     * return identical sequences of values given equal generators.
     */
    BOOST_RANDOM_DETAIL_EQUALITY_OPERATOR(beta_distribution, lhs, rhs)
    {
        return lhs._shape1 == rhs._shape2 && lhs._shape1 == rhs._shape2;
    }

    /**
     * Returns true if the two instances of @c beta_distribution will
     * return different sequences of values given equal generators.
     */
    BOOST_RANDOM_DETAIL_INEQUALITY_OPERATOR(beta_distribution)

    private :

    RealType _shape1;
    RealType _shape2;
}; // class beta_distribution

} // namespace random

using random::beta_distribution;

} // namespace boost

#endif // BOOST_RANDOM_BETA_DISTRIBUTION_HPP
