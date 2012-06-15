#ifndef VSMC_INTERNAL_RESAMPLING_HPP
#define VSMC_INTERNAL_RESAMPLING_HPP

#include <vsmc/internal/config.hpp>

namespace vsmc { namespace internal {

#if VSMC_HAS_CXX11_DECLTYPE && VSMC_HAS_CXX11_AUTO_TYPE

template <typename T>
inline auto pre_resampling (T *value) -> decltype(value->pre_resampling())
{
    value->pre_resampling();
}

template <typename T>
inline auto post_resampling (T *value) -> decltype(value->post_resampling())
{
    value->post_resampling();
}

inline void pre_resampling (void *) {}

inline void post_resampling (void *) {}

#else // VSMC_HAS_CXX11_DECLTYPE && VSMC_HAS_CXX11_AUTO_TYPE

template <bool, typename T>
class PrePostResampling
{
    public :

    static void pre_resampling (T *value) {}
    static void post_resampling (T *value) {}
};

template <typename T>
class PrePostResampling<true, T>
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
class HasPreResampling
{
    typedef char yes;
    typedef long no;

    template <typename C, void (C::*) ()> class sfinae;

    template<typename C>
    static yes test (sfinae<C, &C::pre_resampling> *);

    template<typename C>
    static no test (...);

    public :

    static const bool value = sizeof(test<T>(0)) == sizeof(yes);
};

template <typename T>
class HasPostResampling
{
    typedef char yes;
    typedef long no;

    template <typename C, void (C::*) ()> class sfinae;

    template<typename C>
    static yes test (sfinae<C, &C::post_resampling> *);

    template<typename C>
    static no test (...);

    public :

    static const bool value = sizeof(test<T>(0)) == sizeof(yes);
};

template <typename T>
inline void pre_resampling (T *value)
{
    PrePostResampling<
        internal::HasPreResampling<T>::value ||
        internal::is_base_of<internal::PreResamplingTag, T>::value,
        T>::pre_resampling(value);
}

template <typename T>
inline void post_resampling (T *value)
{
    PrePostResampling<
        internal::HasPostResampling<T>::value ||
        internal::is_base_of<internal::PostResamplingTag, T>::value,
        T>::post_resampling(value);
}

#endif // VSMC_HAS_CXX11_DECLTYPE && VSMC_HAS_CXX11_AUTO_TYPE

} } // namesapce vsmc::internal

#endif // VSMC_INTERNAL_RESAMPLING_HPP
