#ifndef VSMC_INTERNAL_RESAMPLING_HPP
#define VSMC_INTERNAL_RESAMPLING_HPP

#include <vsmc/internal/config.hpp>

namespace vsmc { namespace internal {

#if VSMC_HAS_CXX11_DECLTYPE && VSMC_HAS_CXX11_AUTO_TYPE

inline void pre_resampling (void *) {}

template <typename T>
inline auto pre_resampling (T *value) -> decltype(value->pre_resampling())
{
    value->pre_resampling();
}

inline void post_resampling (void *) {}

template <typename T>
inline auto post_resampling (T *value) -> decltype(value->post_resampling())
{
    value->post_resampling();
}

#else // VSMC_HAS_CXX11_DECLTYPE && VSMC_HAS_CXX11_AUTO_TYPE

template <bool IsParallel, typename T>
class ParallelResampling
{
    public :

    static void pre_resampling (T *value) {}
    static void post_resampling (T *value) {}
};

template <typename T>
class ParallelResampling<true, T>
{
    public :

    static void pre_resampling (T *value)
    {
        value->pre_resampling();
    }

    static void post_resampling (T *value)
    {
        value->post_resampling();
    }
};


template <typename T>
inline void pre_resampling (T *value)
{
    ParallelResampling<internal::is_base_of<internal::ParallelTag, T>::value,
        T>::pre_resampling(value);
}

template <typename T>
inline void post_resampling (T *value)
{
    ParallelResampling<internal::is_base_of<internal::ParallelTag, T>::value,
        T>::post_resampling(value);
}

#endif // VSMC_HAS_CXX11_DECLTYPE && VSMC_HAS_CXX11_AUTO_TYPE

} } // namesapce vsmc::internal

#endif // VSMC_INTERNAL_RESAMPLING_HPP
