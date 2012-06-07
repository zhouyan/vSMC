#ifndef V_SMC_INTERNAL_RESAMPLING_HPP
#define V_SMC_INTERNAL_RESAMPLING_HPP

namespace vSMC { namespace internal {

template <bool if_cl>
inline void pre_resampling_cl (void *value) {}

template <>
inline void pre_resampling_cl<true> (void *value)
{
    reinterpret_cast<StateCLTrait *>(value)->pre_resampling();
}

template <bool if_tbb>
inline void pre_resampling_tbb (void *value) {}

template <>
inline void pre_resampling_tbb<true> (void *value)
{
    reinterpret_cast<StateTBBTrait *>(value)->pre_resampling();
}

template <typename T>
inline void pre_resampling (T *value)
{
    pre_resampling_cl<internal::is_base_of<StateCLTrait, T>::value>(value);
    pre_resampling_tbb<internal::is_base_of<StateTBBTrait, T>::value>(value);
}

template <bool if_cl>
inline void post_resampling_cl (void *value) {}

template <>
inline void post_resampling_cl<true> (void *value)
{
    reinterpret_cast<StateCLTrait *>(value)->post_resampling();
}

template <bool if_tbb>
inline void post_resampling_tbb (void *value) {}

template <>
inline void post_resampling_tbb<true> (void *value)
{
    reinterpret_cast<StateTBBTrait *>(value)->post_resampling();
}

template <typename T>
inline void post_resampling (T *value)
{
    post_resampling_cl<internal::is_base_of<StateCLTrait, T>::value>(value);
    post_resampling_tbb<internal::is_base_of<StateTBBTrait, T>::value>(value);
}

} } // namesapce vSMC::internal

#endif // V_SMC_INTERNAL_RESAMPLING_HPP
