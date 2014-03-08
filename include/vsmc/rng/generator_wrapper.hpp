#ifndef VSMC_RNG_GENERATOR_WRAPPER_HPP
#define VSMC_RNG_GENERATOR_WRAPPER_HPP

#include <vsmc/rng/common.hpp>
#include <immintrin.h>

#define VSMC_STATIC_ASSERT_RNG_GENERATOR_WRAPPER_RESULT_TYPE(ResultType) \
    VSMC_STATIC_ASSERT((                                                     \
                cxx11::is_same<ResultType, uint16_t>::value ||               \
                cxx11::is_same<ResultType, uint32_t>::value ||               \
                cxx11::is_same<ResultType, uint64_t>::value),                \
            USE_GeneratorWrapper_WITH_RESULT_TYPE_OTHER_THAN_uint16_t_OR_uint32_t_OR_uint64_t)

#define VSMC_STATIC_ASSERT_RNG_GENERATOR_WRAPPER \
    VSMC_STATIC_ASSERT_RNG_GENERATOR_WRAPPER_RESULT_TYPE(ResultType);

namespace vsmc {

/// \brief A thin wrapper over any RNG Generator for use with C++11 API
/// \ingroup RNGWrapper
///
/// \details
/// Generator need to have a member function named `generate` which returns
/// uniform integers on the range of `ResultType`, which has to be one of
/// `uint16_t`, `uint32_t` and `uint64_t`. Note that, the generator does not
/// necessarily returns one of these types. For example, on some platforms,
/// some RNG returns `unsigned long long`, which is 64-bits, while `uint64_t`
/// is also 64-bits but a typedef of `unsigned long`. It is ok as long the
/// `static_cast` from the returned value to `uint64_t` does not change, add,
/// or lose any bits.
template <typename ResultType, class Generator>
class GeneratorWrapper
{
    public :

    typedef ResultType result_type;

    explicit GeneratorWrapper (result_type = 0)
    {VSMC_STATIC_ASSERT_RNG_GENERATOR_WRAPPER;}

    template <typename SeedSeq>
    explicit GeneratorWrapper (SeedSeq &, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) {VSMC_STATIC_ASSERT_RNG_GENERATOR_WRAPPER;}

    void seed (result_type) {}

    template <typename SeedSeq>
    void seed (SeedSeq &, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) {}

    result_type operator() ()
    {return static_cast<result_type>(generator_.generate());}

    void discard (std::size_t) {}

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const GeneratorWrapper<ResultType, Generator> &,
            const GeneratorWrapper<ResultType, Generator> &) {return false;}

    friend inline bool operator!= (
            const GeneratorWrapper<ResultType, Generator> &,
            const GeneratorWrapper<ResultType, Generator> &) {return true;}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const GeneratorWrapper<ResultType, Generator> &) {return os;}

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            GeneratorWrapper<ResultType, Generator> &) {return is;}

    private :

    Generator generator_;
}; // clss GeneratorWrapper

} // namespace vsmc

#endif // VSMC_RNG_GENERATOR_WRAPPER_HPP
