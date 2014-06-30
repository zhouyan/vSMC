//============================================================================
// vsmc/rng/ars.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

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
#define VSMC_RNG_ARS_ROUNDS 7
#endif

namespace vsmc {

namespace traits {

namespace internal {

template <std::size_t> struct ARSWeylConstantValue;

template <> struct ARSWeylConstantValue<0> :
    public cxx11::integral_constant<uint64_t, UINT64_C(0xBB67AE8584CAA73B)> {};

template <> struct ARSWeylConstantValue<1> :
    public cxx11::integral_constant<uint64_t, UINT64_C(0x9E3779B97F4A7C15)> {};

} // namespace vsmc::traits::internal

/// \brief ARSEngine Weyl sequence constants
/// \ingroup Traits
///
/// \details
/// The two specializaiton (N = 0, 1) corresponds to lower and upper 64-bits or
/// the Weyl constant.
template <std::size_t I> struct ARSWeylConstantTrait :
    public internal::ARSWeylConstantValue<I> {};

} // namespace vsmc::traits

/// \brief Default ARSEngine key sequence generator
/// \ingroup AESNIRNG
template <typename ResultType>
class ARSKeySeq
{
    public :

    typedef Array<ResultType, 16 / sizeof(ResultType)> key_type;

    ARSKeySeq () : weyl_(_mm_set_epi64x(
                static_cast<int64_t>(traits::ARSWeylConstantTrait<0>::value),
                static_cast<int64_t>(traits::ARSWeylConstantTrait<1>::value)))
    {}

    template <std::size_t Rp1, typename Traits>
    void generate (const key_type &key, Array<__m128i, Rp1, Traits> &key_seq)
    {
        m128i_pack<0>(key, key_seq.front());
        generate_seq<1>(key_seq, cxx11::integral_constant<bool, 1 < Rp1>());
    }

    private :

    const __m128i weyl_;

    template <std::size_t, std::size_t Rp1, typename Traits>
    void generate_seq (Array<__m128i, Rp1, Traits> &, cxx11::false_type) {}

    template <std::size_t N, std::size_t Rp1, typename Traits>
    void generate_seq (Array<__m128i, Rp1, Traits> &key_seq, cxx11::true_type)
    {
        key_seq[Position<N>()] = _mm_add_epi64(
                key_seq[Position<N - 1>()], weyl_);
        generate_seq<N + 1>(key_seq,
                cxx11::integral_constant<bool, N + 1 < Rp1>());
    }
}; // class ARSKeySeq

/// \brief ARS RNG engine
/// \ingroup AESNIRNG
///
/// \details
/// This is a reimplementation of the ARS engine as described in
/// [Parallel Random Numbers: As Easy as 1, 2, 3][r123paper] and implemented in
/// [Random123][r123lib].
///
/// [r123paper]:http://sc11.supercomputing.org/schedule/event_detail.php?evid=pap274
/// [r123lib]: https://www.deshawresearch.com/resources_random123.html
///
/// \sa ARSKeySeq
/// \sa AESNIEngine
template <typename ResultType,
         std::size_t Rounds = VSMC_RNG_ARS_ROUNDS,
         std::size_t Blocks = VSMC_RNG_ARS_BLOCKS>
class ARSEngine :
    public AESNIEngine<ResultType, ARSKeySeq<ResultType>,
    false, Rounds, Blocks>
{
    public :

    typedef AESNIEngine<ResultType, ARSKeySeq<ResultType>,
            false, Rounds, Blocks> base_eng_type;

    explicit ARSEngine (ResultType s = 0) : base_eng_type(s) {}

    template <typename SeedSeq>
    explicit ARSEngine (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_seq<SeedSeq, ResultType>::value>::type * =
            VSMC_NULLPTR) : base_eng_type(seq) {}

    ARSEngine (const typename base_eng_type::ctr_type &c,
            const typename base_eng_type::key_type &k) : base_eng_type(c, k) {}
}; // class ARSEngine

/// \brief ARS RNG engine with 32-bits integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
typedef ARSEngine<uint32_t> ARS_32;

/// \brief ARS RNG engine with 64-bits integers output, default blocks and
/// default rounds
/// \ingroup AESNIRNG
typedef ARSEngine<uint64_t> ARS_64;

} // namespace vsmc

#endif // VSMC_RNG_ARS_HPP
