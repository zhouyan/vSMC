#ifndef V_SMC_RANDOM_BETA_DISTRIBUTION_HPP
#define V_SMC_RANDOM_BETA_DISTRIBUTION_HPP

#include <vSMC/rng/common.hpp>

namespace vSMC { namespace rng {

/// \brief Laplace distribution
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

        typedef beta_distribution distribution_type;

        /// \brief Constructs a \c param_type with given shape1 and shape2
        ///
        /// Requires: shape1 > 0 && shape2 > 0
        ///
        explicit param_type (RealType shape1 = 1.0, RealType shape2 = 1.0) :
            shape1_(shape1), shape2_(shape2) {}

        /// The shape1 parameter of the distribution
        RealType shape1 () const
        {
            return shape1_;
        }

        /// The shape2 parameter of the distribution
        RealType shape2 () const
        {
            return shape2_;
        }

        // FIXME
        // /// Writes a @c param_type to a @c std::ostream.
        // BOOST_RANDOM_DETAIL_OSTREAM_OPERATOR(os, param_type, parm)
        // { os << parm._shape1 << " " << parm._shape2 ; return os; }

        // /// Reads a @c param_type from a @c std::istream.
        // BOOST_RANDOM_DETAIL_ISTREAM_OPERATOR(is, param_type, parm)
        // { is >> parm._shape1 >> std::ws >> parm._shape2; return is; }

        // /// Returns true if the two sets of parameters are the same.
        // BOOST_RANDOM_DETAIL_EQUALITY_OPERATOR(param_type, lhs, rhs)
        // { return lhs._shape1 == rhs._shape1 && lhs._shape2 == rhs._shape2; }

        // /// Returns true if the two sets of parameters are the different.
        // BOOST_RANDOM_DETAIL_INEQUALITY_OPERATOR(param_type)

        private :

        RealType shape1_;
        RealType shape2_;
    };

    /// \brief Constructs a \c beta_distribution from the \c shape1 and \c
    /// shape2 are the parameters
    ///
    /// Requires: shape1 > 0 && shape2 > 0
    explicit beta_distribution (RealType shape1 = 1.0, RealType shape2 = 1.0) :
        shape1_(shape1), shape2_(shape2)
    {
        assert(shape1_ > 0);
        assert(shape2_ > 0);
    }

    /// \brief Constructs a \c beta_distribution from its parameters
    ///
    explicit beta_distribution (const param_type &parm) :
        shape1_(parm.shape1()), shape2_(parm.shape2())
    {
        assert(shape1_ > 0);
        assert(shape2_ > 0);
    }

    /// The shape1 parameter of the distribution
    RealType shape1 () const
    {
        return shape1_;
    }

    /// The shape2 parameter of the distribution
    RealType shape2 () const
    {
        return shape2_;
    }

    /// The smallest value that the distribution can produce
    RealType min V_SMC_PREVENT_MIN_MAX () const
    {
        return 0;
    }

    /// The largest value that the distribution can produce
    RealType max V_SMC_PREVENT_MIN_MAX () const
    {
        return 1;
    }

    /// The parameters of the distribution.
    param_type param () const
    {
        return param_type(shape1_, shape2_);
    }

    /// Sets the parameters of the distribution
    void param (const param_type &parm)
    {
        shape1_ = parm.shape1();
        shape2_ = parm.shape2();
    }

    /// Effects: Subsequent uses of the distribution do not depend
    /// on values produced by any engine prior to invoking reset.
    ///
    void reset () {}

    /// Returns a beta variate.
    template <typename Engine>
    result_type operator() (Engine &eng)
    {
        RealType x = gamma_distribution<RealType>(shape1_, 1)(eng);
        RealType y = gamma_distribution<RealType>(shape2_, 1)(eng);

        return x / (x + y);
    }

    /// Returns a beta variate with parameters specified by \c param.
    template <typename Engine>
    result_type operator() (Engine &eng, const param_type &parm)
    {
        return beta_distribution(parm)(eng);
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

    // ///
    // /// Returns true if the two instances of @c beta_distribution will
    // /// return identical sequences of values given equal generators.
    // ///
    // BOOST_RANDOM_DETAIL_EQUALITY_OPERATOR(beta_distribution, lhs, rhs)
    // {
    //     return lhs._shape1 == rhs._shape2 && lhs._shape1 == rhs._shape2;
    // }

    // ///
    // /// Returns true if the two instances of @c beta_distribution will
    // /// return different sequences of values given equal generators.
    // BOOST_RANDOM_DETAIL_INEQUALITY_OPERATOR(beta_distribution)

    private :

    RealType shape1_;
    RealType shape2_;
}; // class beta_distribution

} } // namespace vSMC::rng

#endif // V_SMC_RANDOM_BETA_DISTRIBUTION_HPP
