#ifndef V_SMC_CORE_RNG_HPP
#define V_SMC_CORE_RNG_HPP

#include <limits>
#include <climits>
#include <cmath>
#include <boost/cstdint.hpp>
#include <boost/math/special_functions/log1p.hpp>
#include <Random123/aes.h>
#include <Random123/ars.h>
#include <Random123/philox.h>
#include <Random123/threefry.h>

/// The Parallel RNG (based on Rand123) seed, unsigned
#ifndef V_SMC_RNG_SEED
#define V_SMC_RNG_SEED 0xdeadbeefU
#endif // V_SMC_RNG_SEED

/// The Parallel RNG (based on Rand123) type, philox or threefry
#ifndef V_SMC_RNG_TYPE
#define V_SMC_RNG_TYPE vSMC::threefry4x64_64
#endif // V_SMC_RNG_TYPE

/// Maximum of results index used by the Boost.Random eigen
#define RANDOM123_EIGEN_IDX_MAX \
    sizeof(typename cbrng_type::ctr_type) / sizeof(result_type)

/// Initializer list for random123_eigen
#define RANDOM123_EIGEN_INIT \
    index_max_(RANDOM123_EIGEN_IDX_MAX), index_(index_max_), \
    step_size_(1), ctr_index_(0), \
    ctr_max_(std::numeric_limits<ctr_value_type>::max())

namespace vSMC {

template <typename Random123Type, typename UIntType>
class random123_eigen
{
    public :
    
    typedef Random123Type cbrng_type;
    typedef UIntType result_type;
    typedef typename cbrng_type::key_type::value_type seed_type;
    typedef typename cbrng_type::ctr_type::value_type ctr_value_type;

    random123_eigen () : RANDOM123_EIGEN_INIT
    {
        ctr_.fill(0);
        seed();
    }

    explicit random123_eigen (seed_type k0) : RANDOM123_EIGEN_INIT
    {
        ctr_.fill(0);
        seed(k0);
    }

    template <typename SeedSeq>
    random123_eigen (SeedSeq &seed_seq) : RANDOM123_EIGEN_INIT
    {
        ctr_.fill(0);
        seed(seed_seq);
    }

    template <typename Iter>
    random123_eigen (Iter &first, Iter last) : RANDOM123_EIGEN_INIT
    {
        ctr_.fill(0);
        seed(first, last);
    }

    void seed ()
    {
        ctr_.fill(0);
        seed(static_cast<seed_type>(V_SMC_RNG_SEED));
    }

    void seed (seed_type k0)
    {
        key_[0] = k0;
        for (unsigned i = 1; i != key_.size(); ++i) {
            key_[i] = (key_[i-1]>>1) |
                (key_[i-1]<<(sizeof(key_[0]) * CHAR_BIT - 1));
        }
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seed_seq)
    {
        key_ = cbrng_type::key_type::seed(seed_seq);
    }

    template <typename Iter>
    void seed (Iter &first, Iter last)
    {
        seed();
        for (unsigned i = 0; i != key_.size() && first != last; ++i, ++first)
            key_[i] = *first;
    }

    void step_size (unsigned size)
    {
        step_size_ = size;
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
            advance_ctr();
            state_.c = cbrng_(ctr_, key_);
        }

        return state_.n[index_++];
    }

    void dicard (unsigned step)
    {
        index_ = (index_ + step) % index_max_;
        advance_ctr(step / index_max_);
        state_.c = cbrng_(ctr_, key_);
    }

    void advance_ctr (unsigned step = 1)
    {
        ctr_value_type inc = step * step_size_;

        if (ctr_max_ - ctr_[ctr_index_] < inc)
            ++ctr_index_;

        if (ctr_index_ == ctr_.size()) {
            ctr_.fill(0);
            ctr_index_ = 0;
        }

        ctr_[ctr_index_] += inc;
    }

    private :

    union {
        typename cbrng_type::ctr_type c;
        result_type n[RANDOM123_EIGEN_IDX_MAX];
    } state_;
    cbrng_type cbrng_;
    typename cbrng_type::ctr_type ctr_;
    typename cbrng_type::key_type key_;
    unsigned index_max_;
    unsigned index_; 
    unsigned step_size_;
    unsigned ctr_index_;
    ctr_value_type ctr_max_;
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
