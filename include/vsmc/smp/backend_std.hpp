//============================================================================
// vSMC/include/vsmc/smp/backend_std.hpp
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

#ifndef VSMC_SMP_BACKEND_STD_HPP
#define VSMC_SMP_BACKEND_STD_HPP

#include <vsmc/smp/backend_base.hpp>

namespace vsmc
{

namespace internal
{

template <typename IntType>
inline void backend_std_range(
    IntType N, vsmc::Vector<IntType> &begin, vsmc::Vector<IntType> &end)
{
    begin.clear();
    end.clear();
    const IntType np = std::max(static_cast<IntType>(1),
        static_cast<IntType>(std::thread::hardware_concurrency()));
    if (np == 1) {
        begin.push_back(0);
        end.push_back(N);
        return;
    }

    const IntType m = N / np;
    const IntType r = N % np;
    for (IntType id = 0; id != np; ++id) {
        const IntType n = m + (id < r ? 1 : 0);
        begin.push_back(id < r ? n * id : (n + 1) * r + n * (id - r));
        end.push_back(begin.back() + n);
    }
}

} // namespace internal

/// \brief Sampler<T>::init_type subtype using the standard library
/// \ingroup STD
template <typename T, typename Derived>
class InitializeSMP<T, Derived, BackendSTD> : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_param(particle, param);
        this->eval_pre(particle);
        vsmc::Vector<size_type> begin;
        vsmc::Vector<size_type> end;
        internal::backend_std_range(particle.size(), begin, end);
        vsmc::Vector<std::future<std::size_t>> task_group;
        for (std::size_t i = 0; i != begin.size(); ++i) {
            const size_type b = begin[i];
            const size_type e = end[i];
            task_group.push_back(
                std::async(std::launch::async, [this, &particle, b, e]() {
                    return this->eval_range(particle.range(b, e));
                }));
        }
        std::size_t accept = 0;
        for (auto &task : task_group)
            accept += task.get();
        this->eval_post(particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(STD, Initialize)
}; // class InitializeSMP

/// \brief Sampler<T>::move_type subtype using the standard library
/// \ingroup STD
template <typename T, typename Derived>
class MoveSMP<T, Derived, BackendSTD> : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_pre(iter, particle);
        vsmc::Vector<size_type> begin;
        vsmc::Vector<size_type> end;
        internal::backend_std_range(particle.size(), begin, end);
        vsmc::Vector<std::future<std::size_t>> task_group;
        for (std::size_t i = 0; i != begin.size(); ++i) {
            const size_type b = begin[i];
            const size_type e = end[i];
            task_group.push_back(std::async(
                std::launch::async, [this, iter, &particle, b, e]() {
                    return this->eval_range(iter, particle.range(b, e));
                }));
        }
        std::size_t accept = 0;
        for (auto &task : task_group)
            accept += task.get();
        this->eval_post(iter, particle);

        return accept;
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(STD, Move)
}; // class MoveSMP

/// \brief Monitor<T>::eval_type subtype using the standard library
/// \ingroup STD
template <typename T, typename Derived>
class MonitorEvalSMP<T, Derived, BackendSTD>
    : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        using size_type = typename Particle<T>::size_type;

        this->eval_pre(iter, particle);
        vsmc::Vector<size_type> begin;
        vsmc::Vector<size_type> end;
        internal::backend_std_range(particle.size(), begin, end);
        vsmc::Vector<std::future<void>> task_group;
        for (std::size_t i = 0; i != begin.size(); ++i) {
            const size_type b = begin[i];
            const size_type e = end[i];
            task_group.push_back(std::async(
                std::launch::async, [this, iter, dim, &particle, r, b, e]() {
                    this->eval_range(iter, dim, particle.range(b, e),
                        r + static_cast<std::size_t>(b) * dim);
                }));
        }
        for (auto &task : task_group)
            task.wait();
        this->eval_post(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(STD, MonitorEval)
}; // class MonitorEvalSMP

/// \brief Sampler<T>::init_type subtype using the standard library
/// \ingroup STD
template <typename T, typename Derived>
using InitializeSTD = InitializeSMP<T, Derived, BackendSTD>;

/// \brief Sampler<T>::move_type subtype using the standard library
/// \ingroup STD
template <typename T, typename Derived>
using MoveSTD = MoveSMP<T, Derived, BackendSTD>;

/// \brief Monitor<T>::eval_type subtype using the standard library
/// \ingroup STD
template <typename T, typename Derived>
using MonitorEvalSTD = MonitorEvalSMP<T, Derived, BackendSTD>;

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_STD_HPP
