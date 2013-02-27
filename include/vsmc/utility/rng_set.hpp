#ifndef VSMC_UTILITY_RNG_SET_HPP
#define VSMC_UTILITY_RNG_SET_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/seed.hpp>

namespace vsmc {

/// \brief Scalar RNG set
/// \ingroup Utility
template <typename RngType>
class RngSet<RngType, ScalarRng>
{
    public :

    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSet (size_type) : rng_(static_cast<
            typename rng_type::result_type>(Seed::instance().get())) {}

    rng_type &rng (size_type id) {return rng_;}

    private :

    rng_type rng_;
}; // class RngSet

/// \brief Vector RNG set
/// \ingroup Utility
template <typename RngType>
class RngSet<RngType, VectorRng>
{
    public :

    typedef RngType rng_type;
    typedef typename std::vector<rng_type>::size_type size_type;

    explicit RngSet (size_type N)
    {
        Seed &seed = Seed::instance();
        for (size_type i = 0; i != N; ++i) {
            rng_.push_back(rng_type(static_cast<
                        typename rng_type::result_type>(seed.get())));
        }
    }

    rng_type &rng (size_type id) {return rng_[id];}

    private :

    std::vector<rng_type> rng_;
}; // class RngSet

} // namespace vsmc

#endif // VSMC_UTILITY_RNG_SET_HPP
