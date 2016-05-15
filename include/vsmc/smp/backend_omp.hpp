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
inline void backend_omp_range(IntType N, IntType &first, IntType &last)
{
    const IntType np = static_cast<IntType>(::omp_get_num_threads());
    const IntType id = static_cast<IntType>(::omp_get_thread_num());
    const IntType m = N / np;
    const IntType r = N % np;
    const IntType n = m + (id < r ? 1 : 0);
    first = id < r ? n * id : (n + 1) * r + n * (id - r);
    last = first + n;
}

} // namespace vsmc::internal

/// \brief Sampler<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class SamplerEvalSMP<T, Derived, BackendOMP>
    : public SamplerEvalBase<T, Derived>
{
    public:
    void operator()(std::size_t iter, Particle<T> &particle)
    {
        run(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, SamplerEval)

    void run(std::size_t iter, Particle<T> &particle)
    {
        run(iter, particle, 1);
    }

    template <typename... Args>
    void run(std::size_t iter, Particle<T> &particle, std::size_t, Args &&...)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_first(iter, particle);
        Particle<T> *pptr = &particle;
#pragma omp parallel default(none) firstprivate(pptr, iter)
        {
            size_type first = 0;
            size_type last = 0;
            internal::backend_omp_range(pptr->size(), first, last);
            this->eval_range(iter, pptr->range(first, last));
        }
        this->eval_last(iter, particle);
    }
}; // class SamplerEvalSMP

/// \brief Monitor<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
class MonitorEvalSMP<T, Derived, BackendOMP>
    : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        run(iter, dim, particle, r);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(OMP, MonitorEval)

    void run(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        run(iter, dim, particle, r, 1);
    }

    template <typename... Args>
    void run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r, std::size_t, Args &&...)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_first(iter, particle);
        Particle<T> *pptr = &particle;
#pragma omp parallel default(none) firstprivate(pptr, iter, dim, r)
        {
            size_type first = 0;
            size_type last = 0;
            internal::backend_omp_range(pptr->size(), first, last);
            this->eval_range(iter, dim, pptr->range(first, last),
                r + static_cast<std::size_t>(first) * dim);
        }
        this->eval_last(iter, particle);
    }
}; // class MonitorEvalSMP

/// \brief Sampler<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
using SamplerEvalOMP = SamplerEvalSMP<T, Derived, BackendOMP>;

/// \brief Monitor<T>::eval_type subtype using OpenMP
/// \ingroup OMP
template <typename T, typename Derived>
using MonitorEvalOMP = MonitorEvalSMP<T, Derived, BackendOMP>;

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_OMP_HPP
