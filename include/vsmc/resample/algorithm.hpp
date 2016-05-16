//============================================================================
// vSMC/include/vsmc/resample/algorithm.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_RESAMPLE_ALGORITHM_HPP
#define VSMC_RESAMPLE_ALGORITHM_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/resample/transform.hpp>
#include <vsmc/rng/u01_sequence.hpp>

namespace vsmc
{

/// \brief Resampling algorithm
/// \ingroup Resample
template <typename U01SeqType, bool Residual>
class ResampleAlgorithm
{
    public:
    /// \brief Generate replication numbers from normalized weights
    ///
    /// \param N Sample size before resampling
    /// \param M Sample size after resampling
    /// \param rng An RNG engine
    /// \param weight N-vector of normalized weights
    /// \param replication N-vector of replication numbers
    template <typename RNGType, typename InputIter, typename OutputIter>
    OutputIter operator()(std::size_t N, std::size_t M, RNGType &rng,
        InputIter weight, OutputIter replication) const
    {
        return eval(N, M, rng, weight, replication,
            std::integral_constant<bool, Residual>());
    }

    private:
    U01SeqType u01seq_;

    template <typename RNGType, typename InputIter, typename OutputIter>
    OutputIter eval(std::size_t N, std::size_t M, RNGType &rng,
        InputIter weight, OutputIter replication, std::false_type) const
    {
        using real_type = typename std::iterator_traits<InputIter>::value_type;

        Vector<real_type> u01(M);
        u01seq_(rng, M, u01.data());

        return resample_trans_u01_rep(N, M, weight, u01.data(), replication);
    }

    template <typename RNGType, typename InputIter, typename OutputIter>
    OutputIter eval(std::size_t N, std::size_t M, RNGType &rng,
        InputIter weight, OutputIter replication, std::true_type) const
    {
        using real_type = typename std::iterator_traits<InputIter>::value_type;
        using rep_type = typename std::iterator_traits<OutputIter>::value_type;

        Vector<real_type> resid(N);
        Vector<rep_type> integ(N);
        std::size_t R =
            resample_trans_residual(N, M, weight, resid.data(), integ.data());

        Vector<real_type> u01(R);
        u01seq_(rng, R, u01.data());
        resample_trans_u01_rep(N, R, resid.data(), u01.data(), replication);
        for (std::size_t i = 0; i != N; ++i, ++replication)
            *replication += integ[i];

        return replication;
    }
}; // class ResampleAlgorithm

/// \brief Sampler<T>::eval_type subtype
template <typename T>
class ResampleEval
{
    public:
    using eval_type = std::function<void(std::size_t, std::size_t,
        typename Particle<T>::rng_type &, const double *,
        typename Particle<T>::size_type *)>;

    /// \brief Construct a `Sampler::move_type` object
    ///
    /// \param eval A resampling algorithm evaluation object, see interface of
    /// ResampleAlgorithm
    explicit ResampleEval(const eval_type &eval) : eval_(eval) {}

    /// \brief Set a new evaluation object of type eval_type
    void eval(const eval_type &new_eval) { eval_ = new_eval; }

    /// \brief Returns how many particles having non-zero replication number
    std::size_t operator()(std::size_t, Particle<T> &particle) const
    {
        runtime_assert(static_cast<bool>(eval_),
            "**ResampleEval::operator()** invalid evaluation object");

        using size_type = typename Particle<T>::size_type;

        const std::size_t N = static_cast<std::size_t>(particle.size());
        Vector<size_type> rep(N);
        Vector<size_type> idx(N);
        eval_(N, N, particle.rng(), particle.weight().data(), rep.data());
        resample_trans_rep_index(N, N, rep.data(), idx.data());
        particle.state().select(N, idx.data());
        particle.weight().set_equal();

        std::size_t R = 0;
        for (auto r : rep)
            if (r != 0)
                ++R;

        return R;
    }

    private:
    eval_type eval_;
}; // class ResampleEval

/// \brief Multinomial resampling
/// \ingroup Resample
using ResampleMultinomial = ResampleAlgorithm<U01SequenceSorted, false>;

/// \brief Stratified resampling
/// \ingroup Resample
using ResampleStratified = ResampleAlgorithm<U01SequenceStratified, false>;

/// \brief Systematic resampling
/// \ingroup Resample
using ResampleSystematic = ResampleAlgorithm<U01SequenceSystematic, false>;

/// \brief Residual resampling
/// \ingroup Resample
using ResampleResidual = ResampleAlgorithm<U01SequenceSorted, true>;

/// \brief Residual stratified resampling
/// \ingroup Resample
using ResampleResidualStratified =
    ResampleAlgorithm<U01SequenceStratified, true>;

/// \brief Residual systematic resampling
/// \ingroup Resample
using ResampleResidualSystematic =
    ResampleAlgorithm<U01SequenceSystematic, true>;

/// \brief Type trait of ResampleScheme parameter
/// \ingroup Resample
template <ResampleScheme>
class ResampleTypeTrait;

/// \brief Type trait of Multinomial scheme
/// \ingroup Resample
template <>
class ResampleTypeTrait<Multinomial>
{
    public:
    using type = ResampleMultinomial;
}; // class ResampleTypeTrait

/// \brief Type trait of Stratified scheme
/// \ingroup Resample
template <>
class ResampleTypeTrait<Stratified>
{
    public:
    using type = ResampleStratified;
}; // class ResampleTypeTrait

/// \brief Type trait of Systematic scheme
/// \ingroup Resample
template <>
class ResampleTypeTrait<Systematic>
{
    public:
    using type = ResampleSystematic;
}; // class ResampleTypeTrait

/// \brief Type trait of Residual scheme
/// \ingroup Resample
template <>
class ResampleTypeTrait<Residual>
{
    public:
    using type = ResampleResidual;
}; // class ResampleTypeTrait

/// \brief Type trait of ResidualStratified scheme
/// \ingroup Resample
template <>
class ResampleTypeTrait<ResidualStratified>
{
    public:
    using type = ResampleResidualStratified;
}; // class ResampleTypeTrait

/// \brief Type trait of ResidualSystematic scheme
/// \ingroup Resample
template <>
class ResampleTypeTrait<ResidualSystematic>
{
    public:
    using type = ResampleResidualSystematic;
}; // class ResampleTypeTrait

/// \brief Type of resample class corresponding to ResampleScheme parameter
/// \ingroup Resample
template <ResampleScheme Scheme>
using ResampleType = typename ResampleTypeTrait<Scheme>::type;

} // namespace vsmc

#endif // VSMC_RESAMPLE_ALGORITHM_HPP
