//============================================================================
// vSMC/include/vsmc/smp/internal/parallel_work.hpp
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

#ifndef VSMC_SMP_INTERNAL_PARALLEL_WORK_HPP
#define VSMC_SMP_INTERNAL_PARALLEL_WORK_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/particle.hpp>
#include <vsmc/core/single_particle.hpp>

namespace vsmc {

namespace traits {

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(RangeTypeConstIterator, const_iterator,
        std::size_t)

} // namespace traits

namespace internal {

template <typename T, typename IntType>
class ParallelCopyParticle
{
    public :

    ParallelCopyParticle (T *state, const IntType *copy_from) :
        state_(state), copy_from_(copy_from) {}

    template <typename SizeType>
    typename std::enable_if<std::is_integral<SizeType>::value>::type
    operator() (SizeType id) const
    {
        typedef typename traits::SizeTypeTrait<T>::type size_type;

        state_->copy_particle(
                static_cast<size_type>(copy_from_[id]),
                static_cast<size_type>(id));
    }

    template <typename RangeType>
    typename std::enable_if<!std::is_integral<RangeType>::value>::type
    operator() (const RangeType &range) const
    {
        typedef typename traits::RangeTypeConstIteratorTrait<RangeType>::type
            const_iterator;

        const const_iterator begin =
            static_cast<const_iterator>(range.begin());
        const const_iterator end =
            static_cast<const_iterator>(range.end());
        for (const_iterator id = begin; id != end; ++id)
            operator()(id);
    }

    private :

    T *const state_;
    const IntType *const copy_from_;
}; // class ParallelCopyParticle

template <typename T, typename InitType>
class ParallelInitializeState
{
    public :

    ParallelInitializeState (InitType *init, Particle<T> *particle) :
        init_(init), particle_(particle), accept_(0) {}

    template <typename SplitType>
    ParallelInitializeState (const ParallelInitializeState<T, InitType> &other,
            SplitType) :
        init_(other.init_), particle_(other.particle_), accept_(0) {}

    template <typename SizeType>
    typename std::enable_if<std::is_integral<SizeType>::value>::type
    operator() (SizeType id)
    {
        typedef typename traits::SizeTypeTrait<T>::type size_type;

        accept_ += init_->initialize_state(SingleParticle<T>(
                    static_cast<size_type>(id), particle_));
    }

    template <typename RangeType>
    typename std::enable_if<!std::is_integral<RangeType>::value>::type
    operator() (const RangeType &range)
    {
        typedef typename traits::RangeTypeConstIteratorTrait<RangeType>::type
            const_iterator;

        const const_iterator begin =
            static_cast<const_iterator>(range.begin());
        const const_iterator end =
            static_cast<const_iterator>(range.end());
        for (const_iterator id = begin; id != end; ++id)
            operator()(id);
    }

    void join (const ParallelInitializeState<T, InitType> &other)
    {accept_ += other.accept_;}

    std::size_t accept () const {return accept_;}

    private :

    InitType *const init_;
    Particle<T> *const particle_;
    std::size_t accept_;
}; // class ParallelInitializeState

template <typename T, typename MoveType>
class ParallelMoveState
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;

    ParallelMoveState (MoveType *move, std::size_t iter,
            Particle<T> *particle) :
        move_(move), iter_(iter), particle_(particle), accept_(0) {}

    template <typename SplitType>
    ParallelMoveState (const ParallelMoveState<T, MoveType> &other,
            SplitType) :
        move_(other.move_), iter_(other.iter_), particle_(other.particle_),
        accept_(0) {}

    template <typename SizeType>
    typename std::enable_if<std::is_integral<SizeType>::value>::type
    operator() (SizeType id)
    {
        typedef typename traits::SizeTypeTrait<T>::type size_type;

        accept_ += move_->move_state(iter_, SingleParticle<T>(
                    static_cast<size_type>(id), particle_));
    }

    template <typename RangeType>
    typename std::enable_if<!std::is_integral<RangeType>::value>::type
    operator() (const RangeType &range)
    {
        typedef typename traits::RangeTypeConstIteratorTrait<RangeType>::type
            const_iterator;

        const const_iterator begin =
            static_cast<const_iterator>(range.begin());
        const const_iterator end =
            static_cast<const_iterator>(range.end());
        for (const_iterator id = begin; id != end; ++id)
            operator()(id);
    }

    void join (const ParallelMoveState<T, MoveType> &other)
    {accept_ += other.accept_;}

    std::size_t accept () const {return accept_;}

    private :

    MoveType *const move_;
    const std::size_t iter_;
    Particle<T> *const particle_;
    std::size_t accept_;
}; // class ParallelMoveState

template <typename T, typename MonitorEvalType>
class ParallelMonitorState
{
    public :

    ParallelMonitorState (MonitorEvalType *monitor,
            std::size_t iter, std::size_t dim,
            const Particle<T> *particle, double *res) :
        monitor_(monitor), iter_(iter), dim_(dim),
        particle_(particle), res_(res) {}

    template <typename SizeType>
    typename std::enable_if<std::is_integral<SizeType>::value>::type
    operator() (SizeType id) const
    {
        typedef typename traits::SizeTypeTrait<T>::type size_type;

        monitor_->monitor_state(iter_, dim_,
                ConstSingleParticle<T>(static_cast<size_type>(id), particle_),
                res_ + id * dim_);
    }

    template <typename RangeType>
    typename std::enable_if<!std::is_integral<RangeType>::value>::type
    operator() (const RangeType &range) const
    {
        typedef typename traits::RangeTypeConstIteratorTrait<RangeType>::type
            const_iterator;

        const const_iterator begin =
            static_cast<const_iterator>(range.begin());
        const const_iterator end =
            static_cast<const_iterator>(range.end());
        for (const_iterator id = begin; id != end; ++id)
            operator()(id);
    }

    private :

    MonitorEvalType *const monitor_;
    const std::size_t iter_;
    const std::size_t dim_;
    const Particle<T> *const particle_;
    double *const res_;
}; // class ParallelMonitorState

template <typename T, typename PathEvalType>
class ParallelPathState
{
    public :

    ParallelPathState (PathEvalType *path, std::size_t iter,
            const Particle<T> *particle, double *res) :
        path_(path), iter_(iter), particle_(particle), res_(res) {}

    template <typename SizeType>
    typename std::enable_if<std::is_integral<SizeType>::value>::type
    operator() (SizeType id) const
    {
        typedef typename traits::SizeTypeTrait<T>::type size_type;

        res_[id] = path_->path_state(iter_,
                ConstSingleParticle<T>(static_cast<size_type>(id), particle_));
    }

    template <typename RangeType>
    typename std::enable_if<!std::is_integral<RangeType>::value>::type
    operator() (const RangeType &range) const
    {
        typedef typename traits::RangeTypeConstIteratorTrait<RangeType>::type
            const_iterator;

        const const_iterator begin =
            static_cast<const_iterator>(range.begin());
        const const_iterator end =
            static_cast<const_iterator>(range.end());
        for (const_iterator id = begin; id != end; ++id)
            operator()(id);
    }

    private :

    PathEvalType *const path_;
    const std::size_t iter_;
    const Particle<T> *const particle_;
    double *const res_;
}; // class ParallelPathState

} // namespace vsmc::internal

} // namespace vsmc

#endif // VSMC_SMP_INTERNAL_PARALLEL_WORK_HPP
