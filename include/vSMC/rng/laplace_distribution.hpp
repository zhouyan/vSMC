#ifndef BOOST_RANDOM_LAPLACE_DISTRIBUTION_HPP
#define BOOST_RANDOM_LAPLACE_DISTRIBUTION_HPP

#include <boost/config/no_tr1/cmath.hpp>
#include <istream>
#include <iosfwd>
#include <boost/assert.hpp>
#include <boost/limits.hpp>
#include <boost/static_assert.hpp>
#include <boost/math/special_functions/log1p.hpp>
#include <boost/random/detail/config.hpp>
#include <boost/random/detail/operators.hpp>
#include <boost/random/uniform_01.hpp>

namespace boost { namespace random {

/**
 * Instantiations of class template laplace_distribution model a
 * \random_distribiton. Such a distribution produces random numbers @c x
 * distributed with probability density function
 * \f$\displaystyle p(x) =
 *   \frac{1}{2b}\exp(-|x-\mu|/b)
 * \f$,
 * where location and scale are the paramters of the distributions.
 */
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

        /**
         * Constructs a @c param_type with a given location and scale.
         *
         * Requires: scale >= 0
         */
        explicit param_type (
                RealType location_arg = RealType(0.0),
                RealType scale_arg = RealType(1.0)) :
            _location(location_arg), _scale(scale_arg) {}

        /** Returns the location of the distribution. */
        RealType location () const {return _location;}

        /** Returns the scale of the distribution. */
        RealType scale () const {return _scale;}

        /** Writes a @c param_type to a @c std::ostream. */
        BOOST_RANDOM_DETAIL_OSTREAM_OPERATOR(os, param_type, parm)
        { os << parm._location << " " << parm._scale ; return os; }

        /** Reads a @c param_type from a @c std::istream. */
        BOOST_RANDOM_DETAIL_ISTREAM_OPERATOR(is, param_type, parm)
        { is >> parm._location >> std::ws >> parm._scale; return is; }

        /** Returns true if the two sets of parameters are the same. */
        BOOST_RANDOM_DETAIL_EQUALITY_OPERATOR(param_type, lhs, rhs)
        { return lhs._location == rhs._location && lhs._scale == rhs._scale; }
        
        /** Returns true if the two sets of parameters are the different. */
        BOOST_RANDOM_DETAIL_INEQUALITY_OPERATOR(param_type)
        private :

        RealType _location;
        RealType _scale;
    }; // class param_type

    /**
     * Constructs a @c laplace_distribution objet. @c location and @c scale
     * are the parameters for the distribution.
     *
     * Requires: scale > 0
     */
    explicit laplace_distribution (
            const RealType &location_arg = RealType(1.0),
            const RealType &scale_arg = RealType(1.0)) :
        _location(location_arg), _scale(scale_arg)
    {
        BOOST_ASSERT(_scale >= RealType(0.0));
    }

    /**
     * Constructs a @c laplace_distribution object from its parameters.
     */
    explicit laplace_distribution (const param_type &parm) :
        _location(parm.location()), _scale(parm.scale())
    {
        BOOST_ASSERT(_scale > RealType(0.0));
    }

    /** Returns the location of the distribution. */
    RealType location () const {return _location;}

    /** Returns the scale of the distribution. */
    RealType scale () const {return _scale;}

    /** Returns the smallest value that the distribution can produce. */
    RealType min BOOST_PREVENT_MACRO_SUBSTITUTION () const
    {return -std::numeric_limits<RealType>::infinity();}

    /** Returns the largest value that the distribution can produce. */
    RealType max BOOST_PREVENT_MACRO_SUBSTITUTION () const
    {return std::numeric_limits<RealType>::infinity();}

    /** Returns the parameters of the distribution. */
    param_type param () const {return param_type(_location, _scale);}

    /** Sets the parameters of the distribution. */
    void param (const param_type &parm)
    {
        _location = parm.location();
        _scale = parm.scale();
    }

    /**
     * Effects: Subsequent uses of the distribution do not depend
     * on values produced by any engine prior to invoking reset.
     */
    void reset () {}

    /** Returns a laplace variate. */
    template <typename Engine>
    result_type operator() (Engine &eng)
    {
        using std::abs;

        RealType u = boost::random::uniform_01<RealType>()(eng) - 0.5;
        RealType r = -2 * std::abs(u);

        r = boost::math::log1p(r);
        r = u > 0 ? r : -r;

        return _location - _scale * r;
    }

    /** Returns a laplace variate with parameters specified by @c param. */
    template <typename URNG>
    result_type operator() (URNG &urng, const param_type &parm)
    {
        return laplace_distribution(parm)(urng);
    }

    /** Writes a @c laplace_distribution to a @c std::ostream. */
    BOOST_RANDOM_DETAIL_OSTREAM_OPERATOR(os, laplace_distribution, nd)
    {
        os << nd._location << " " << nd._scale << " ";
        return os;
    }

    /** Reads a @c laplace_distribution from a @c std::istream. */
    BOOST_RANDOM_DETAIL_ISTREAM_OPERATOR(is, laplace_distribution, nd)
    {
        is >> std::ws >> nd._location >> std::ws >> nd._scale;
        return is;
    }

    /**
     * Returns true if the two instances of @c laplace_distribution will
     * return identical sequences of values given equal generators.
     */
    BOOST_RANDOM_DETAIL_EQUALITY_OPERATOR(laplace_distribution, lhs, rhs)
    {
        return lhs._location == rhs._location && lhs._scale == rhs._scale;
    }

    /**
     * Returns true if the two instances of @c laplace_distribution will
     * return different sequences of values given equal generators.
     */
    BOOST_RANDOM_DETAIL_INEQUALITY_OPERATOR(laplace_distribution)

    private :

    RealType _location;
    RealType _scale;
}; // class laplace_distribution

} // namespace random

using random::laplace_distribution;

} // namespace boost

#endif // BOOST_RANDOM_LAPLACE_DISTRIBUTION_HPP
