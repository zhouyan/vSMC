//============================================================================
// include/vsmc/rng/rng_set.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_RNG_RNG_SET_HPP
#define VSMC_RNG_RNG_SET_HPP

#include <vsmc/rng/seed.hpp>
#include <vsmc/rng/threefry.hpp>

/// \brief Default RNG set type
/// \ingroup Config
#ifndef VSMC_RNG_SET_TYPE
#define VSMC_RNG_SET_TYPE ::vsmc::RngSet< ::vsmc::Threefry4x64, ::vsmc::Vector>
#endif

namespace vsmc {

template <typename, typename> class RngSet;

/// \brief Scalar RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, Scalar>
{
    public :

    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSet (size_type N) : size_(N) {seed();}

    size_type size () const {return size_;}

    void resize (std::size_t) {}

    void seed () {rng_.seed(Seed::instance().get());}

    rng_type &operator[] (size_type) {return rng_;}

    private :

    std::size_t size_;
    rng_type rng_;
}; // class RngSet

/// \brief Vector RNG set
/// \ingroup RNG
template <typename RngType>
class RngSet<RngType, Vector>
{
    public :

    typedef RngType rng_type;
    typedef typename std::vector<rng_type>::size_type size_type;

    explicit RngSet (size_type N) : rng_(N, rng_type()) {seed();}

    size_type size () const {return rng_.size();}

    void resize (std::size_t n)
    {
        if (n == rng_.size())
            return;

        if (n < rng_.size())
            rng_.resize(n);

        rng_.reserve(n);
        rng_type rng;
        for (std::size_t i = rng_.size(); i != n; ++i) {
            rng.seed(Seed::instance().get());
            rng_.push_back(rng);
        }
    }

    void seed ()
    {
        for (size_type i = 0; i != rng_.size(); ++i)
            rng_[i].seed(Seed::instance().get());
    }

    rng_type &operator[] (size_type id) {return rng_[id];}

    private :

    std::vector<rng_type> rng_;
}; // class RngSet

namespace traits {

/// \brief Particle::rng_set_type trait
/// \ingroup Traits
VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RngSetType, rng_set_type, VSMC_RNG_SET_TYPE)

} // namespace vsmc::traits

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_HPP
