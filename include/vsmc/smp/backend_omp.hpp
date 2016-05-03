//============================================================================
// vSMC/include/vsmc/smp/backend_omp.hpp
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

#ifndef VSMC_SMP_BACKEND_OMP_HPP
#define VSMC_SMP_BACKEND_OMP_HPP

#include <vsmc/smp/backend_base.hpp>
#include <omp.h>

namespace vsmc
{

namespace internal
{

template <typename IntType>
inline void backend_omp_range(IntType N, IntType &begin, IntType &end)
{
    const IntType np = static_cast<IntType>(::omp_get_num_threads());
    const IntType id = static_cast<IntType>(::omp_get_thread_num());
    const IntType m = N / np;
    const IntType r = N % np;
    const IntType n = m + (id < r ? 1 : 0);
    begin = id < r ? n * id : (n + 1) * r + n * (id - r);
    end = begin + n;
}

} // namespace vsmc::internal

/// \brief SMP implementation ID for OpenMP
/// \ingroup OMP
class BackendOMP;

/// \brief Sampler<T>::init_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class InitializeSMP<BackendOMP, T, Derived> : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_param(particle, param);
        this->eval_pre(particle);
        std::size_t accept = 0;
        Particle<T> *pptr = &particle;
#pragma omp parallel default(none) shared(accept) firstprivate(pptr)
        {
            size_type begin = 0;
            size_type end = 0;
            internal::backend_omp_range(pptr->size(), begin, end);
            std::size_t acc = this->eval_range(pptr->range(begin, end));
#pragma omp atomic
            accept += acc;
        }
        this->eval_post(particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, Initialize)
}; // class InitializeSMP

/// \brief Sampler<T>::move_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class MoveSMP<BackendOMP, T, Derived> : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_pre(iter, particle);
        std::size_t accept = 0;
        Particle<T> *pptr = &particle;
#pragma omp parallel default(none) shared(accept) firstprivate(pptr, iter)
        {
            size_type begin = 0;
            size_type end = 0;
            internal::backend_omp_range(pptr->size(), begin, end);
            std::size_t acc = this->eval_range(iter, pptr->range(begin, end));
#pragma omp atomic
            accept += acc;
        }
        this->eval_post(iter, particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, Move)
}; // class MoveSMP

/// \brief Monitor<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class MonitorEvalSMP<BackendOMP, T, Derived>
    : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_pre(iter, particle);
        Particle<T> *pptr = &particle;
#pragma omp parallel default(none) firstprivate(pptr, iter, dim, r)
        {
            size_type begin = 0;
            size_type end = 0;
            internal::backend_omp_range(pptr->size(), begin, end);
            this->eval_range(iter, dim, pptr->range(begin, end),
                r + static_cast<std::size_t>(begin) * dim);
        }
        this->eval_post(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, MonitorEval)
}; // class MonitorEvalSMP

/// \brief Sampler<T>::init_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
using InitializeOMP = InitializeSMP<BackendOMP, T, Derived>;

/// \brief Sampler<T>::move_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
using MoveOMP = MoveSMP<BackendOMP, T, Derived>;

/// \brief Monitor<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
using MonitorEvalOMP = MonitorEvalSMP<BackendOMP, T, Derived>;

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_OMP_HPP
