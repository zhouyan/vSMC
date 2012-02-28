#ifndef V_SMC_CORE_RNG_HPP
#define V_SMC_CORE_RNG_HPP

#include <limits>
#include <cmath>
#include <boost/math/special_functions/log1p.hpp>
#include <Random123/aes.h>
#include <Random123/ars.h>
#include <Random123/philox.h>
#include <Random123/threefry.h>

/// The Parallel RNG (based on Rand123) seed, unsigned
#ifndef V_SMC_RNG_SEED
#define V_SMC_RNG_SEED 0xdeadbeefL
#endif // V_SMC_RNG_SEED

/// The Parallel RNG (based on Rand123) type, philox or threefry
#ifndef V_SMC_RNG_TYPE
#define V_SMC_RNG_TYPE Threefry4x64
#endif // V_SMC_RNG_TYPE

/// The type used to extract random bits
#ifndef V_SMC_RNG_UINT_TYPE
#define V_SMC_RNG_UINT_TYPE unsigned
#endif // V_SMC_RNG_UINT_TYPE

#define V_SMC_RNG_IDX_MAX sizeof(rng_type::ctr_type) / sizeof(uint_type)

namespace vSMC {

class Rng
{
    public :

    /// \brief Type of the internal RNG (Random123 C++ type)
    typedef r123::V_SMC_RNG_TYPE rng_type;
    /// \brief Type of the internal unsigned random integer
    typedef V_SMC_RNG_UINT_TYPE uint_type;

    Rng (
            unsigned ctr_seed = V_SMC_RNG_SEED,
            unsigned key_seed = V_SMC_RNG_SEED,
            unsigned step = 1) :
        ctr_seed_(ctr_seed), key_seed_(key_seed), step_(step),
        base_(std::numeric_limits<uint_type>::max()),
        index_(0), index_max_(V_SMC_RNG_IDX_MAX)
    {
        rng_type::ctr_type c = {{}};
        rng_type::key_type k = {{}};
        c[0] = ctr_seed;
        k[0] = key_seed;
        ctr_ = c;
        key_ = k;
        state_.c = crng_(ctr_, key_);
    }

    unsigned ctr_seed () const
    {
        return ctr_seed_;
    }

    void ctr_seed (unsigned seed)
    {
        ctr_seed_ = seed;
        ctr_[0] = seed;
        state_.c = crng_(ctr_, key_);
    }

    unsigned key_seed () const
    {
        return key_seed_;
    }

    void key_seed (unsigned seed)
    {
        key_seed_ = seed;
        key_[0] = seed;
        state_.c = crng_(ctr_, key_);
    }

    void seed (unsigned c_seed, unsigned k_seed)
    {
        ctr_seed_ = c_seed;
        key_seed_ = k_seed;
        ctr_[0] = c_seed;
        key_[0] = k_seed;
        state_.c = crng_(ctr_, key_);
    }

    unsigned step () const
    {
        return step_;
    }

    void step (unsigned s)
    {
        step_ = s;
    }

    /// \brief Generate an uniform random integer
    uint_type ruint ()
    {
        if (index_ == index_max_) {
            index_ = 0;
            ctr_[0] += step_;
            state_.c = crng_(ctr_, key_);
        }

        return state_.n[index_++];
    }

    /// \brief Generate an [0,1] uniform random variate
    double runif ()
    {
        return ruint() / base_;
    }

    double runif (double min, double max)
    {
        return runif() * (max - min) + min;
    }

    double rnorm (double mean, double sd)
    {
        static const double pi2 = 6.283185307179586476925286766559005768394;

        double u1 = runif();
        double u2 = runif();

        return std::sqrt(-2 * std::log(u1)) * std::sin(pi2 * u2) * sd + mean;
    }

    double rlnorm (double meanlog, double sdlog)
    {
        return std::exp(rnorm(meanlog, sdlog));
    }

    double rcauchy (double location, double scale)
    {
        static const double pi = 3.141592653589793238462643383279502884197;

        return scale * std::tan(pi * (runif() - 0.5)) + location;
    }

    double rexp (double scale)
    {
        return -scale * boost::math::log1p(-runif());
    }

    double rlaplace (double location, double scale)
    {
        double u = runif() - 0.5;

        return u > 0 ?
            location - scale * boost::math::log1p(-2 * u):
            location + scale * boost::math::log1p(2 * u);
    }

    double rweibull (double shape, double scale)
    {
        return scale * std::pow(
                -boost::math::log1p(-runif()), 1 / shape);
    }

    // TODO GAMMA
    double rgamma (double shape, double scale)
    {
        return 0;
    }

    double rchisq (double df)
    {
        return rgamma(0.5 * df, 2);
    }

    double rf (double df1, double df2)
    {
        return rchisq(df1) / rchisq(df2) * df2 / df1;
    }

    double rt (double df)
    {
        return rnorm(0, 1) / std::sqrt(rchisq(df) / df);
    }

    // TODO BETA
    double rbeta (double shape1, double shape2)
    {
        return 0;
    }

    private :

    rng_type crng_;
    rng_type::ctr_type ctr_;
    rng_type::key_type key_;
    unsigned ctr_seed_;
    unsigned key_seed_;
    unsigned step_;
    double base_;
    unsigned index_;
    unsigned index_max_;
    union {rng_type::ctr_type c; uint_type n[V_SMC_RNG_IDX_MAX];} state_;
};

} // namespace vSMC

#endif // V_SMC_CORE_RNG_HPP
