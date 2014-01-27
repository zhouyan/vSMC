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

    explicit RngSetTBB (size_type N = 1) : size_(N) {init_rng();}

    size_type size () const {return size_;}

    rng_type &rng (size_type id = 0) {return rng_.local();}

    private :

    std::size_t size_;
    traits::RngShift<rng_type> shift_;
    tbb::combinable<rng_type> rng_;
    tbb::mutex mtx_;

    void init_rng ()
    {
        tbb::mutex::scoped_lock lock(mtx_);
        rng_.local().seed(Seed::instance().get());
        shift_(rng_.local());
    }
}; // class RngSetTBB

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_TBB_HPP
