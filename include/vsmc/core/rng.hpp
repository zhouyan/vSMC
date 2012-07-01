#ifndef VSMC_CORE_RNG_HPP
#define VSMC_CORE_RNG_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/random.hpp>

namespace vsmc {

/// \brief Sequential RNG set class
/// \ingroup Core
class RngSetSeq
{
    public :

    /// The type of the random number generator C++11 engine
    typedef VSMC_SEQRNG_TYPE rng_type;

    protected :

    typedef std::size_t size_type;

    RngSetSeq (size_type N) : rng_(N)
    {
        rng::Seed<rng_type> &seed = rng::Seed<rng_type>::create();
        rng_ = rng_type(static_cast<rng_type::result_type>(seed.get()));
    }

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
        return rng_;
    }

    private :

    rng_type rng_;
}; // class RngSetSeq

/// \brief Parallel RNG set class
/// \ingroup Core
class RngSetPrl
{
    protected :

    /// The type of the random number generator C++11 engine
    typedef VSMC_PRLRNG_TYPE rng_type;

    typedef std::vector<rng_type>::size_type size_type;

    RngSetPrl (size_type N) : rng_(N)
    {
        rng::Seed<rng_type> &seed = rng::Seed<rng_type>::create();
        for (size_type i = 0; i != N; ++i)
            rng_[i] = rng_type(static_cast<rng_type::result_type>(seed.get()));
    }

    /// \brief Get a C++11 RNG engine
    ///
    /// \param id The position of the particle, 0 to size() - 1
    ///
    /// \return A reference to a C++11 RNG engine unique to particle at
    /// position id, and independent of others
    rng_type &rng (size_type id)
    {
        return rng_[id];
    }

    private :

    std::vector<rng_type> rng_;
}; // class RngSetPrl

namespace internal {

template <typename T>
class HasRngSetType
{
    private :

    struct char2 {char c1; char c2;};

    template<typename S>
    static char test (typename S::rng_set_type *);

    template <typename S>
    static char2 test (...);

    public :

    static const bool value = sizeof(test<T>(VSMC_NULLPTR)) == sizeof(char);
};

template <typename T, bool>
class RngSetTypeDispatch;

template <typename T>
class RngSetTypeDispatch<T, true>
{
    public :

    typedef typename T::rng_set_type type;
};

template <typename T>
class RngSetTypeDispatch<T, false>
{
    public :

#if VSMC_USE_RANDOM123
    typedef RngSetPrl type;
#else
    typedef RngSetSeq type;
#endif
};

} // namespace vsmc::internal

/// \brief Trait class of rng_set_type
/// \ingroup Core
template <typename T>
class RngSetTypeTrait
{
    public :

    /// \brief Type of T::rng_set_type if it exist, otherwise RngSetSeq or
    /// RngSetPrl depending on \c VSMC_USE_RANDOM123
    typedef typename internal::RngSetTypeDispatch<T,
            internal::HasRngSetType<T>::value>::type type;
}; // class RngSetTypeTrait

} // namespace vsmc

#endif // VSMC_CORE_RNG_HPP
