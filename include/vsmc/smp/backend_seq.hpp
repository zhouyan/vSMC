//============================================================================
// vSMC/include/vsmc/smp/backend_seq.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
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

VSMC_DEFINE_SMP_BACKEND_FORWARD(SEQ)

/// \brief Particle::value_type subtype
/// \ingroup SEQ
template <typename StateBase>
class StateSEQ : public StateBase
{
    public:
    typedef typename traits::SizeTypeTrait<StateBase>::type size_type;

    explicit StateSEQ(size_type N) : StateBase(N) {}

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        for (size_type to = 0; to != N; ++to)
            this->copy_particle(static_cast<size_type>(copy_from[to]), to);
    }
}; // class StateSEQ

/// \brief Sampler<T>::init_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
class InitializeSEQ : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = particle.size();
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        std::size_t accept = 0;
        for (size_type i = 0; i != N; ++i)
            accept += this->initialize_state(SingleParticle<T>(i, &particle));
        this->post_processor(particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(SEQ, Initialize)
}; // class InitializeSEQ

/// \brief Sampler<T>::move_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
class MoveSEQ : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = particle.size();
        this->pre_processor(iter, particle);
        std::size_t accept = 0;
        for (size_type i = 0; i != N; ++i)
            accept += this->move_state(iter, SingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(SEQ, Move)
}; // class MoveSEQ

/// \brief Monitor<T>::eval_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
class MonitorEvalSEQ : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = particle.size();
        this->pre_processor(iter, particle);
        for (size_type i = 0; i != N; ++i) {
            this->monitor_state(iter, dim, SingleParticle<T>(i, &particle),
                res + static_cast<std::size_t>(i) * dim);
        }
        this->post_processor(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(SEQ, MonitorEval)
}; // class MonitorEvalSEQ

/// \brief Path<T>::eval_type subtype
/// \ingroup SEQ
template <typename T, typename Derived>
class PathEvalSEQ : public PathEvalBase<T, Derived>
{
    public:
    double operator()(std::size_t iter, Particle<T> &particle, double *res)
    {
        typedef typename Particle<T>::size_type size_type;
        const size_type N = particle.size();
        this->pre_processor(iter, particle);
        for (size_type i = 0; i != N; ++i)
            res[i] = this->path_state(iter, SingleParticle<T>(i, &particle));
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(SEQ, PathEval)
}; // class PathEvalSEQ

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_SEQ_HPP
