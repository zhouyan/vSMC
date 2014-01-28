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

    explicit RngSetTBB (size_type N) : size_(N), flag_(flag_init) {}

    size_type size () const {return size_;}

    rng_type &rng (size_type id = 0)
    {
        init_rng();
        return rng_.local();
    }

    private :

    std::size_t size_;
    traits::RngShift<rng_type> shift_;
    tbb::mutex mtx_;
    tbb::combinable<rng_type> rng_;
    tbb::combinable<bool> flag_;

    void init_rng ()
    {
        if (flag_.local()) return;

        tbb::mutex::scoped_lock lock(mtx_);
        rng_.local().seed(Seed::instance().get());
        shift_(rng_.local());
        flag_.local() = true;
    }

    static bool flag_init () {return false;}
}; // class RngSetTBB

} // namespace vsmc

#endif // VSMC_RNG_RNG_SET_TBB_HPP
