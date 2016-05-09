//============================================================================
// vSMC/include/vsmc/smp/backend_tbb.hpp
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

#ifndef VSMC_SMP_BACKEND_TBB_HPP
#define VSMC_SMP_BACKEND_TBB_HPP

#include <vsmc/smp/backend_base.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

namespace vsmc
{

namespace internal
{

template <typename IntType>
inline ::tbb::blocked_range<IntType> backend_tbb_range(
    IntType N, std::size_t grainsize)
{
    return grainsize == 0 ? ::tbb::blocked_range<IntType>(0, N) :
                            ::tbb::blocked_range<IntType>(0, N, grainsize);
}

} // namespace internal

/// \brief Sampler<T>::init_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class InitializeSMP<T, Derived, BackendTBB> : public InitializeBase<T, Derived>
{
    public:
    std::size_t operator()(Particle<T> &particle, void *param)
    {
        return run(particle, param);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, Initialize)

    class work_type
    {
        public:
        using size_type = typename Particle<T>::size_type;

        work_type(
            InitializeSMP<T, Derived, BackendTBB> *wptr, Particle<T> *pptr)
            : wptr_(wptr), pptr_(pptr), accept_(0)
        {
        }

        work_type(const work_type &other, ::tbb::split)
            : wptr_(other.wptr_), pptr_(other.pptr_), accept_(0)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range)
        {
            accept_ +=
                wptr_->eval_range(pptr_->range(range.begin(), range.end()));
        }

        void join(const work_type &other) { accept_ += other.accept_; }

        std::size_t accept() const { return accept_; }

        private:
        InitializeSMP<T, Derived, BackendTBB> *const wptr_;
        Particle<T> *const pptr_;
        std::size_t accept_;
    }; // class work_type

    template <typename... Args>
    std::size_t run(Particle<T> &particle, void *param, std::size_t grainsize,
        Args &&... args)
    {
        this->eval_param(particle, param);
        this->eval_pre(particle);
        work_type work(this, &particle);
        ::tbb::parallel_reduce(
            internal::backend_tbb_range(particle.size(), grainsize), work,
            std::forward<Args>(args)...);
        this->eval_post(particle);

        return work.accept();
    }

    std::size_t run(Particle<T> &particle, void *param)
    {
        return run(particle, param, 1);
    }
}; // class InitializeSMP

/// \brief Sampler<T>::move_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class MoveSMP<T, Derived, BackendTBB> : public MoveBase<T, Derived>
{
    public:
    std::size_t operator()(std::size_t iter, Particle<T> &particle)
    {
        return run(iter, particle);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, Move)

    class work_type
    {
        public:
        using size_type = typename Particle<T>::size_type;

        work_type(MoveSMP<T, Derived, BackendTBB> *wptr, std::size_t iter,
            Particle<T> *pptr)
            : wptr_(wptr), iter_(iter), pptr_(pptr), accept_(0)
        {
        }

        work_type(const work_type &other, ::tbb::split)
            : wptr_(other.wptr_)
            , iter_(other.iter_)
            , pptr_(other.pptr_)
            , accept_(0)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range)
        {
            accept_ += wptr_->eval_range(
                iter_, pptr_->range(range.begin(), range.end()));
        }

        void join(const work_type &other) { accept_ += other.accept_; }

        std::size_t accept() const { return accept_; }

        private:
        MoveSMP<T, Derived, BackendTBB> *const wptr_;
        const std::size_t iter_;
        Particle<T> *const pptr_;
        std::size_t accept_;
    }; // class work_type

    template <typename... Args>
    std::size_t run(std::size_t iter, Particle<T> &particle,
        std::size_t grainsize, Args &&... args)
    {
        this->eval_pre(iter, particle);
        work_type work(this, iter, &particle);
        ::tbb::parallel_reduce(
            internal::backend_tbb_range(particle.size(), grainsize), work,
            std::forward<Args>(args)...);
        this->eval_post(iter, particle);

        return work.accept();
    }

    std::size_t run(std::size_t iter, Particle<T> &particle)
    {
        return run(iter, particle, 0);
    }
}; // class MoveSMP

/// \brief Monitor<T>::eval_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
class MonitorEvalSMP<T, Derived, BackendTBB>
    : public MonitorEvalBase<T, Derived>
{
    public:
    void operator()(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        run(iter, dim, particle, r);
    }

    protected:
    VSMC_DEFINE_SMP_BACKEND_SPECIAL(TBB, MonitorEval)

    class work_type
    {
        public:
        using size_type = typename Particle<T>::size_type;

        work_type(MonitorEvalSMP<T, Derived, BackendTBB> *wptr,
            std::size_t iter, std::size_t dim, Particle<T> *pptr, double *r)
            : wptr_(wptr), iter_(iter), dim_(dim), pptr_(pptr), r_(r)
        {
        }

        void operator()(const ::tbb::blocked_range<size_type> &range) const
        {
            wptr_->eval_range(iter_, dim_,
                pptr_->range(range.begin(), range.end()),
                r_ + static_cast<std::size_t>(range.begin()) * dim_);
        }

        private:
        MonitorEvalSMP<T, Derived, BackendTBB> *const wptr_;
        const std::size_t iter_;
        const std::size_t dim_;
        Particle<T> *const pptr_;
        double *const r_;
    }; // class work_type

    template <typename... Args>
    void run(std::size_t iter, std::size_t dim, Particle<T> &particle,
        double *r, std::size_t grainsize, Args &&... args)
    {
        this->eval_pre(iter, particle);
        work_type work(this, iter, dim, &particle, r);
        ::tbb::parallel_for(
            internal::backend_tbb_range(particle.size(), grainsize), work,
            std::forward<Args>(args)...);
        this->eval_post(iter, particle);
    }

    void run(
        std::size_t iter, std::size_t dim, Particle<T> &particle, double *r)
    {
        run(iter, dim, particle, r, 1);
    }
}; // class MonitorEvalSMP

/// \brief Sampler<T>::init_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
using InitializeTBB = InitializeSMP<T, Derived, BackendTBB>;

/// \brief Sampler<T>::move_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
using MoveTBB = MoveSMP<T, Derived, BackendTBB>;

/// \brief Monitor<T>::eval_type subtype using Intel Threading Building Blocks
/// \ingroup TBB
template <typename T, typename Derived>
using MonitorEvalTBB = MonitorEvalSMP<T, Derived, BackendTBB>;

} // namespace vsmc

#endif // VSMC_SMP_BACKEND_TBB_HPP
