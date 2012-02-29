#ifndef V_SMC_CORE_RNG_HPP
#define V_SMC_CORE_RNG_HPP

#include <limits>
#include <cmath>
#include <boost/cstdint.hpp>
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
#define V_SMC_RNG_TYPE vSMC::threefry4x64_64
#endif // V_SMC_RNG_TYPE

/// Maximum of results index used by the Boost.Random eigen
#define BOOST_EIGEN_IDX_MAX \
    sizeof(typename rng_type::ctr_type) / sizeof(result_type)

namespace vSMC {

template <typename Random123Type, typename UIntType>
class random123_eigen
{
    public :
    
    typedef Random123Type rng_type;
    typedef UIntType result_type;

    random123_eigen () :
        index_max_(BOOST_EIGEN_IDX_MAX), index_(index_max_), step_(1)
    {
        seed();
    }

    explicit random123_eigen (typename rng_type::ctr_type::value_type k0) :
        index_max_(BOOST_EIGEN_IDX_MAX), index_(index_max_), step_(1)
    {
        seed(k0);
    }

    template <typename CtrIter, typename KeyIter>
    random123_eigen (CtrIter &first_ctr, CtrIter last_ctr,
            KeyIter &first_key, KeyIter last_key) :
        index_max_(BOOST_EIGEN_IDX_MAX), index_(index_max_), step_(1)
    {
        seed(first_ctr, last_ctr, first_key, last_key);
    }

    void seed ()
    {
        ctr_.fill(1);
        key_.fill(1);
    }

    void seed (typename rng_type::key_type::value_type k0)
    {
        seed();
        key_[0] = k0;
    }

    void seed (
            typename rng_type::ctr_type::value_type c0,
            typename rng_type::key_type::value_type k0)
    {
        seed();
        ctr_[0] = c0;
        key_[0] = k0;
    }

    template <typename CtrIter, typename KeyIter>
    void seed (CtrIter &first_ctr, CtrIter last_ctr,
            KeyIter &first_key, KeyIter last_key)
    {
        seed();
        for (typename rng_type::ctr_type::iterator iter_ctr = ctr_.begin();
                iter_ctr != ctr_.end && first_ctr != last_ctr;
                ++iter_ctr, ++first_ctr) {
            *iter_ctr = *first_ctr;
        }
        for (typename rng_type::key_type::iterator iter_key = key_.begin();
                iter_key != key_.end && first_key != last_key;
                ++iter_key, ++first_key) {
            *iter_key = *first_key;
        }
    }

    void step_size (unsigned s)
    {
        step_ = s;
    }

    static result_type min ()
    {
        return std::numeric_limits<result_type>::min();
    }

    static result_type max ()
    {
        return std::numeric_limits<result_type>::max();
    }

    result_type operator () ()
    {
        if (index_ == index_max_) {
            index_ = 0;
            ctr_[0] += step_;
            state_.c = crng_(ctr_, key_);
        }

        return state_.n[index_++];
    }

    private :

    union {
        typename rng_type::ctr_type c;
        result_type n[BOOST_EIGEN_IDX_MAX];
    } state_;
    rng_type crng_;
    typename rng_type::ctr_type ctr_;
    typename rng_type::key_type key_;
    unsigned index_max_;
    unsigned index_; 
    unsigned step_;
};

typedef random123_eigen<r123::Philox2x32, uint32_t> philox2x32_32;
typedef random123_eigen<r123::Philox2x32, uint64_t> philox2x32_64;
typedef random123_eigen<r123::Philox4x32, uint32_t> philox4x32_32;
typedef random123_eigen<r123::Philox4x32, uint64_t> philox4x32_64;
typedef random123_eigen<r123::Philox2x64, uint32_t> philox2x64_32;
typedef random123_eigen<r123::Philox2x64, uint64_t> philox2x64_64;
typedef random123_eigen<r123::Philox4x64, uint32_t> philox4x64_32;
typedef random123_eigen<r123::Philox4x64, uint64_t> philox4x64_64;

typedef random123_eigen<r123::Threefry2x32, uint32_t> threefry2x32_32;
typedef random123_eigen<r123::Threefry2x32, uint64_t> threefry2x32_64;
typedef random123_eigen<r123::Threefry4x32, uint32_t> threefry4x32_32;
typedef random123_eigen<r123::Threefry4x32, uint64_t> threefry4x32_64;
typedef random123_eigen<r123::Threefry2x64, uint32_t> threefry2x64_32;
typedef random123_eigen<r123::Threefry2x64, uint64_t> threefry2x64_64;
typedef random123_eigen<r123::Threefry4x64, uint32_t> threefry4x64_32;
typedef random123_eigen<r123::Threefry4x64, uint64_t> threefry4x64_64;

} // namespace vSMC

#endif // V_SMC_CORE_RNG_HPP
