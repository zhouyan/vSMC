#ifndef VSMC_INTERNAL_RESAMPLING_HPP
#define VSMC_INTERNAL_RESAMPLING_HPP

namespace vsmc { namespace internal {

template <bool IsParallel, typename T>
class ParallelResampling
{
    public :

    static void pre_resampling (T &value) {}
    static void post_resampling (T &value) {}
};

template <typename T>
class ParallelResampling<true, T>
{
    public :

    static void pre_resampling (T &value)
    {
        value.pre_resampling();
    }

    static void post_resampling (T &value)
    {
        value.post_resampling();
    }
};


template <typename T>
inline void pre_resampling (T &value)
{
    ParallelResampling<internal::is_base_of<ParallelTag, T>::value, T>::
        pre_resampling(value);
}

template <typename T>
inline void post_resampling (T &value)
{
    ParallelResampling<internal::is_base_of<ParallelTag, T>::value, T>::
        post_resampling(value);
}

} } // namesapce vsmc::internal

#endif // VSMC_INTERNAL_RESAMPLING_HPP
