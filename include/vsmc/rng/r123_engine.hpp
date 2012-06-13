#ifndef VSMC_RNG_R123_ENGINE_HPP
#define VSMC_RNG_R123_ENGINE_HPP

// #include <Random123/aes.h>
// #include <Random123/ars.h>
#include <Random123/philox.h>
#include <Random123/threefry.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4521)
#endif // _MSC_VER

#include <Random123/conventional/Engine.hpp>

#ifdef _MSC_VER
#pragma warning(pop)
#endif // _MSC_VER

namespace vsmc { namespace rng {

#ifdef VSMC_USE_CONSTEXPR_ENGINE

#if !VSMC_HAS_CXX11_CONSTEXPR
#error VSMC_HAS_CXX11_CONSTEXPR has to be defined to a\
    non-zero value to enable VSMC_USE_CONSTEXPR_ENGINE
#endif

/// \brief An Engine with constexpr min() and max()
/// \ingroup RNG
///
/// \details
/// vsmc::rng::Engine is an alias to r123::Engine unless the macro \c
/// VSMC_USE_CONSTEXPR_ENGINE is explicited defined before the including of
/// this header. One shall never define this macro unless it is absolutely
/// needed as explained below.
///
/// The Engine distributed by Random123 does not return \c min() and \c max()
/// as \c constexpr for a good reason. However this will be needed if \c libc++
/// is used. This mini Engine is derived from r123::Engine. The base type is
/// not actually intended to be used as a base class. As this derived class
/// does not have any data of its own. It shall be fine to use it even through
/// base type pointers, though that is certainly a bad practice. However when
/// \c min() and \c max() are of concern, one shall use the derived type to
/// invoke the static members. In furture we may write a entirely new class for
/// this case but that will be reinventing the wheel which Random123 has
/// already done. So that won't happen in any near future.
template <typename CBRNG>
class Engine : public r123::Engine<CBRNG>
{
    public :

    typedef typename r123:Engine<CBRNG> engine_type;

    typedef typename engine_type::cbrng_type  cbrng_type;
    typedef typename engine_type::ctr_type    ctr_type;
    typedef typename engine_type::key_type    key_type;
    typedef typename engine_type::ukey_type   ukey_type;
    typedef typename engine_type::result_type result_type;
    typedef typename engine_type::elem_type   elem_type;

    explicit r123_engine () : engine_type() {}
    explicit r123_engine (result_type r) : engine_type(r) {}

    template <typename SeedSeq>
    explicit r123_engine (SeedSeq &seed_seq) : engine_type(seed_seq) {}

    static constexpr result_type min VSMC_PREVENT_MIN_MAX ()
    {
        return 0;
    }

    static constexpr result_type max VSMC_PREVENT_MIN_MAX ()
    {
        return std::numeric_limits<result_type>::max VSMC_PREVENT_MIN_MAX ();
    }

}; // class r123_engine

#else // VSMC_USE_CONSTEXPR_ENGINE

using r123::Engine;

#endif // VSMC_USE_CONSTEXPR_ENGINE

} } // namespae vsmc::rng

#endif // VSMC_RNG_R123_ENGINE_HPP
