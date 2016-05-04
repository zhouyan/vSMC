//============================================================================
// vSMC/include/vsmc/smp/backend_seq.hpp
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

#ifndef VSMC_SMP_BACKEND_SEQ_HPP
#define VSMC_SMP_BACKEND_SEQ_HPP

#include <vsmc/smp/backend_base.hpp>

namespace vsmc
{

/// \brief SMP implementation ID for sequential
/// \ingroup SEQ
class BackendSEQ;

/// \brief Sampler<T>::init_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
class InitializeSMP<BackendSEQ, T, Derived> : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        this->eval_param(particle, param);
        this->eval_pre(particle);
        std::size_t accept = this->eval_range(particle.range());
        this->eval_post(particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(SEQ, Initialize)
}; // class InitializeSMP

/// \brief Sampler<T>::move_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
class MoveSMP<BackendSEQ, T, Derived> : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        this->eval_pre(iter, particle);
        std::size_t accept = this->eval_range(iter, particle.range());
        this->eval_post(iter, particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(SEQ, Move)
}; // class MoveSMP

/// \brief Monitor<T>::eval_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
class MonitorEvalSMP<BackendSEQ, T, Derived>
    : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        this->eval_pre(iter, particle);
        this->eval_range(iter, dim, particle.range(), r);
        this->eval_post(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(SEQ, MonitorEval)
}; // class MonitorEvalSMP

/// \brief Sampler<T>::init_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
using InitializeSEQ = InitializeSMP<BackendSEQ, T, Derived>;

/// \brief Sampler<T>::move_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
using MoveSEQ = MoveSMP<BackendSEQ, T, Derived>;

/// \brief Monitor<T>::eval_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
using MonitorEvalSEQ = MonitorEvalSMP<BackendSEQ, T, Derived>;

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_SEQ_HPP
