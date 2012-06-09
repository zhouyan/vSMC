#ifndef VSMC_INTERNAL_RESAMPLING_HPP
#define VSMC_INTERNAL_RESAMPLING_HPP

namespace vsmc { namespace internal {

template <bool IsCL, typename T>
class resampling_cl
{
    public :

    static void pre_resampling (T &value) {}
    static void post_resampling (T &value) {}
};

template <typename T>
class resampling_cl<true, T>
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

template <bool IsTBB, typename T>
class resampling_tbb
{
    public :

    static void pre_resampling (T &value) {}
    static void post_resampling (T &value) {}
};

template <typename T>
class resampling_tbb<true, T>
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
    resampling_tbb<internal::is_base_of<StateCLTrait, T>::value, T>::
        pre_resampling(value);
    resampling_tbb<internal::is_base_of<StateTBBTrait, T>::value, T>::
        pre_resampling(value);
}

template <typename T>
inline void post_resampling (T &value)
{
    resampling_tbb<internal::is_base_of<StateCLTrait, T>::value, T>::
        post_resampling(value);
    resampling_tbb<internal::is_base_of<StateTBBTrait, T>::value, T>::
        post_resampling(value);
}

} } // namesapce vsmc::internal

#endif // VSMC_INTERNAL_RESAMPLING_HPP
