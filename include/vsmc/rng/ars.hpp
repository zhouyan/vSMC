#ifndef VSMC_RNG_ARS_HPP
#define VSMC_RNG_ARS_HPP

#include <vsmc/rng/aes_ni.hpp>

/// \brief ARSEngine default blocks
/// \ingroup Config
#ifndef VSMC_RNG_ARS_BLOCKS
#define VSMC_RNG_ARS_BLOCKS 1
#endif

/// \brief ARSEngine default rounds
/// \ingroup Config
#ifndef VSMC_RNG_ARS_ROUNDS
#define VSMC_RNG_ARS_ROUNDS 10
#endif

namespace vsmc {

namespace internal {

template <std::size_t> struct ARSWeylConstantValue;

template <> struct ARSWeylConstantValue<0> :
    public cxx11::integral_constant<uint64_t, UINT64_C(0xBB67AE8584CAA73B)> {};

template <> struct ARSWeylConstantValue<1> :
    public cxx11::integral_constant<uint64_t, UINT64_C(0x9E3779B97F4A7C15)> {};

} // namespace vsmc::internal

namespace traits {

/// \brief ARSEngine Weyl sequence constants
/// \ingroup Traits
///
/// \details
/// The two specializaiton (N = 0, 1) corresponds to lower and upper 64-bits or
/// the Weyl constant.
template <std::size_t N> struct ARSWeylConstantTrait :
    public ::vsmc::internal::ARSWeylConstantValue<N> {};

} // namespace vsmc::traits

/// \brief Default ARSEngine key sequence generator
/// \ingroup R123RNG
class ARSKeySeq
{
    public :

    template <std::size_t Rp1, typename Traits>
    static void generate (const __m128i &ukey,
            StaticVector<__m128i, Rp1, Traits> &key_seq)
    {
        key_seq.front() = ukey;
        __m128i weyl = _mm_set_epi64x(
                static_cast<int64_t>(traits::ARSWeylConstantTrait<0>::value),
                static_cast<int64_t>(traits::ARSWeylConstantTrait<1>::value));
        generate<1>(weyl, key_seq, cxx11::integral_constant<bool, 1 < Rp1>());
    }

    private :

    template <std::size_t, std::size_t Rp1, typename Traits>
    static void generate (const __m128i &,
            StaticVector<__m128i, Rp1, Traits> &, cxx11::false_type) {}

    template <std::size_t N, std::size_t Rp1, typename Traits>
    static void generate (const __m128i &weyl,
            StaticVector<__m128i, Rp1, Traits> &key_seq, cxx11::true_type)
    {
        key_seq[Position<N>()] = _mm_add_epi64(
                key_seq[Position<N - 1>()], weyl);
        generate<N + 1>(weyl, key_seq,
                cxx11::integral_constant<bool, N + 1 < Rp1>());
    }
}; // class ARSKeySeq

/// \brief ARS RNG engine reimplemented
/// \ingroup AESNIRNG
///
/// \details
/// This is a reimplementation of the algorithm ARS as described in [Parallel
/// Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// \sa ARSKeySeq.
template <typename ResultType,
         std::size_t R = VSMC_RNG_ARS_ROUNDS,
         std::size_t Blocks = VSMC_RNG_ARS_BLOCKS>
class ARSEngine : public AESNIEngine<ResultType, ARSKeySeq, R, Blocks>
{
    typedef AESNIEngine<ResultType, ARSKeySeq, R, Blocks> base;

    public :

    explicit ARSEngine (ResultType s = 0) : base(s) {}

    template <typename SeedSeq>
    explicit ARSEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base(seq) {}
}; // class ARSEngine

/// \brief ARS RNG engine returning 32-bits integers with default blocks and
/// default rounds
/// \ingroup R123RNG
typedef ARSEngine<uint32_t> ARS4x32;

/// \brief ARS RNG engine returning 64-bits integers with default blocks and
/// default rounds
/// \ingroup R123RNG
typedef ARSEngine<uint64_t> ARS2x64;

/// \brief ARS RNG engine returning 128-bits integers with default blocks and
/// default rounds
/// \ingroup R123RNG
typedef ARSEngine<__m128i>  ARS1x128;

} // namespace vsmc

#endif // VSMC_RNG_ARS_HPP
