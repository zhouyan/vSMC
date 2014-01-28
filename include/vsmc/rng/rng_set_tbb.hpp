#ifndef VSMC_RNG_RNG_SET_TBB_HPP
#define VSMC_RNG_RNG_SET_TBB_HPP

#include <vsmc/rng/rng_set.hpp>
#include <tbb/tbb.h>

namespace vsmc {

/// \brief Thread local RNG set using Intel TBB
/// \ingroup RNG
template <typename RngType>
class RngSetTBB
{
    public :

    typedef RngType rng_type;
    typedef std::size_t size_type;

    explicit RngSetTBB (size_type N) : size_(N), rng_(rng_init) {}

    size_type size () const {return size_;}

    rng_type &rng (size_type id = 0)
    {
        std::pair<rng_type, bool> &rng_flag = rng_.local();

        if (rng_flag.second) return rng_flag.first;

        tbb::mutex::scoped_lock lock(mtx_);
        rng_flag.first.seed(Seed::instance().get());
        shift_(rng_flag.first);
        rng_flag.second = true;

        return rng_flag.first;
    }

    private :

    std::size_t size_;
    traits::RngShift<rng_type> shift_;
    tbb::mutex mtx_;
    tbb::combinable<std::pair<rng_type, bool> > rng_;

    static std::pair<rng_type, bool> rng_init ()
    {return std::make_pair(rng_type(), false);}
}; // class RngSetTBB

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_TBB_HPP
