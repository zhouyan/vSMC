#ifndef VSMC_RNG_ENGINE_RESULT_WRAPPER_HPP
#define VSMC_RNG_ENGINE_RESULT_WRAPPER_HPP

#include <vsmc/rng/common.hpp>

#define VSMC_STATIC_ASSERT_RNG_ENGINE_RESULT_WRAPPER_SIZE(K) \
    VSMC_STATIC_ASSERT((K > 0),                                             \
            USE_EngineResultWrapper_WITH_A_TOO_LARGE_RESULT_TYPE)

#define VSMC_STATIC_ASSERT_RNG_ENGINE_RESULT_WRAPPER \
    VSMC_STATIC_ASSERT_RNG_ENGINE_RESULT_WRAPPER_SIZE(K_);

namespace vsmc {

/// \brief Wrapper one RNG Engine into another
/// \ingroup RNGWrapper
///
/// \details
/// Wrap one engine into another. The `result_type` of the original engine has
/// to be a multiple of the new `ResultType`. For example, if `Eng` generate
/// 64-bits random bits, and then `ResultType` can be `uint32_t` and 32-bits
/// are returned every time the `operator()` is called. And each call down to
/// the original engine's `operator()` generates two 32-bits integers.
template <typename ResultType, typename Eng>
class EngineResultWrapper
{
    static VSMC_CONSTEXPR const std::size_t K_ =
        sizeof(typename Eng::result_type) / sizeof(ResultType);

    public :

    typedef ResultType result_type;
    typedef Eng engine_type;

    explicit EngineResultWrapper (result_type s = 1) : eng_(s), remain_(0)
    {VSMC_STATIC_ASSERT_RNG_ENGINE_RESULT_WRAPPER;}

    template <typename SeedSeq>
    explicit EngineResultWrapper (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, result_type>::value>::type * =
            VSMC_NULLPTR) : eng_(seq), remain_(0)
    {VSMC_STATIC_ASSERT_RNG_ENGINE_RESULT_WRAPPER;}

    void seed (result_type s)
    {
        eng_.seed(s);
        remain_ = 0;
    }

    template <typename SeedSeq>
    void seed (SeedSeq &seq, typename cxx11::enable_if<
            !internal::is_seed_sequence<SeedSeq, result_type>::value>::type * =
            VSMC_NULLPTR)
    {
        eng_.seed(seq);
        remain_ = 0;
    }

    result_type operator() ()
    {
        if (remain_ == 0) {
            typename Eng::result_type *rp = reinterpret_cast<
                typename Eng::result_type *>(res_.data());
            *rp = eng_() ;
            remain_ = K_;
        }

        return res_[--remain_];
    }

    void discard (std::size_t nskip)
    {
        if (nskip <= remain_) {
            remain_ -= nskip;
            return;
        }

        nskip_ -= remain_;
        remain_ = 0;
        eng_.discard(nskip / K_);
        nskip -= (nskip / K_) * K_;

        for (std::size_t i = 0; i != nskip; ++i)
            operator()();
    }

    static VSMC_CONSTEXPR const result_type _Min = 0;
    static VSMC_CONSTEXPR const result_type _Max = static_cast<result_type>(
            ~(static_cast<result_type>(0)));

    static VSMC_CONSTEXPR result_type min VSMC_MNE () {return _Min;}
    static VSMC_CONSTEXPR result_type max VSMC_MNE () {return _Max;}

    friend inline bool operator== (
            const EngineResultWrapper<ResultType, Eng> &eng1,
            const EngineResultWrapper<ResultType, Eng> &eng2)
    {return eng1.eng_ == eng2.eng_;}

    friend inline bool operator!= (
            const EngineResultWrapper<ResultType, Eng> &eng1,
            const EngineResultWrapper<ResultType, Eng> &eng2)
    {return !(eng1 == eng2);}

    template <typename CharT, typename Traits>
    friend inline std::basic_ostream<CharT, Traits> &operator<< (
            std::basic_ostream<CharT, Traits> &os,
            const EngineResultWrapper<ResultType, Eng> &eng)
    {return (os << eng.eng_);}

    template <typename CharT, typename Traits>
    friend inline std::basic_istream<CharT, Traits> &operator>> (
            std::basic_istream<CharT, Traits> &is,
            EngineResultWrapper<ResultType, Eng> &eng)
    {
        engine_type eng_tmp;
        result_type weyl_tmp = 0;
        if (is) is >> std::ws >> eng_tmp;
        if (is) {
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
            eng.eng_ = cxx11::move(eng_tmp);
#else
            eng.eng_ = eng_tmp;
#endif
        }

        return is;
    }

    private :

    Eng eng_;
    StaticVector<ResultType, K_> res_;
    std::size_t remain_;
}; // class EngineResultWrapper

} // namespace

#endif // VSMC_RNG_ENGINE_RESULT_WRAPPER_HPP
