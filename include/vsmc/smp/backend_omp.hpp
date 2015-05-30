//============================================================================
// vSMC/include/vsmc/smp/backend_omp.hpp
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

#ifndef VSMC_SMP_BACKEND_OMP_HPP
#define VSMC_SMP_BACKEND_OMP_HPP

#include <vsmc/smp/backend_base.hpp>
#include <omp.h>

namespace vsmc
{

namespace internal
{

#if defined(_OPENMP) && _OPENMP >= 200805 // OpenMP 3.0
template <typename T>
using OMPSizeType = typename traits::SizeTypeTrait<T>::type;
#else
template <typename T>
using OMPSizeType =
    std::make_signed<typename traits::SizeTypeTrait<T>::type>::type;
#endif

} // namespace vsmc::internal

VSMC_DEFINE_SMP_BACKEND_FORWARD(OMP)

/// \brief Particle::value_type subtype using OpenMP
/// \ingroup OMP
template <typename StateBase>
class StateOMP : public StateBase
{
    public:
    typedef typename traits::SizeTypeTrait<StateBase>::type size_type;

    explicit StateOMP(size_type N) : StateBase(N) {}

    template <typename IntType>
    void copy(size_type N, const IntType *copy_from)
    {
        internal::OMPSizeType<StateBase> NN =
            static_cast<internal::OMPSizeType<StateBase>>(N);
#pragma omp parallel for default(shared)
        for (internal::OMPSizeType<StateBase> to = 0; to < NN; ++to) {
            this->copy_particle(static_cast<size_type>(copy_from[to]),
                static_cast<size_type>(to));
        }
    }
}; // class StateOMP

/// \brief Sampler<T>::init_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class InitializeOMP : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        internal::OMPSizeType<T> N =
            static_cast<internal::OMPSizeType<T>>(particle.size());
        this->initialize_param(particle, param);
        this->pre_processor(particle);
        std::size_t accept = 0;
#pragma omp parallel for reduction(+ : accept) default(shared)
        for (internal::OMPSizeType<T> i = 0; i < N; ++i) {
            accept += this->initialize_state(SingleParticle<T>(
                static_cast<typename Particle<T>::size_type>(i), &particle));
        }
        this->post_processor(particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, Initialize)
}; // class InitializeOMP

/// \brief Sampler<T>::move_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class MoveOMP : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        const internal::OMPSizeType<T> N =
            static_cast<internal::OMPSizeType<T>>(particle.size());
        this->pre_processor(iter, particle);
        std::size_t accept = 0;
#pragma omp parallel for reduction(+ : accept) default(shared)
        for (internal::OMPSizeType<T> i = 0; i < N; ++i) {
            accept += this->move_state(
                iter, SingleParticle<T>(
                          static_cast<typename Particle<T>::size_type>(i),
                          &particle));
        }
        this->post_processor(iter, particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, Move)
}; // class MoveOMP

/// \brief Monitor<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class MonitorEvalOMP : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *res)
    {
        const internal::OMPSizeType<T> N =
            static_cast<internal::OMPSizeType<T>>(particle.size());
        this->pre_processor(iter, particle);
#pragma omp parallel for default(shared)
        for (internal::OMPSizeType<T> i = 0; i < N; ++i) {
            this->monitor_state(
                iter, dim, SingleParticle<T>(
                               static_cast<typename Particle<T>::size_type>(i),
                               &particle),
                res + i * dim);
        }
        this->post_processor(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, MonitorEval)
}; // class MonitorEvalOMP

/// \brief Path<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class PathEvalOMP : public PathEvalBase<T, Derived>
{
    public:
    double operator()(std::size_t iter, Particle<T> &particle, double *res)
    {
        const internal::OMPSizeType<T> N =
            static_cast<internal::OMPSizeType<T>>(particle.size());
        this->pre_processor(iter, particle);
#pragma omp parallel for default(shared)
        for (internal::OMPSizeType<T> i = 0; i < N; ++i) {
            res[i] = this->path_state(
                iter, SingleParticle<T>(
                          static_cast<typename Particle<T>::size_type>(i),
                          &particle));
        }
        this->post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, PathEval)
}; // class PathEvalOMP

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_OMP_HPP
