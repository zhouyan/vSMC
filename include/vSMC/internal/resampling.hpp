#ifndef V_SMC_INTERNAL_RESAMPLING_HPP
#define V_SMC_INTERNAL_RESAMPLING_HPP

namespace vSMC { namespace internal {

template <bool if_cl, typename T>
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

template <bool if_tbb, typename T>
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

} } // namesapce vSMC::internal

#endif // V_SMC_INTERNAL_RESAMPLING_HPP
