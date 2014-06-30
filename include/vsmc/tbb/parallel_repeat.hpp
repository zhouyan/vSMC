//============================================================================
// include/vsmc/tbb/parallel_repeat.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_TBB_PARALLEL_REPEAT_HPP
#define VSMC_TBB_PARALLEL_REPEAT_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <iterator>

namespace vsmc {

namespace internal {

template <typename Body>
class TBBParallelRepeatBody
{
    public :

    TBBParallelRepeatBody (const Body &body) : body_(body) {}

    void operator() (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i)
            body_(i);
    }

    private :

    const Body body_;
}; // class TBBParallelRepeatBody

template <typename Body, typename RandomIter>
class TBBParallelRepeatBodyRI
{
    public :

    TBBParallelRepeatBodyRI (const Body &body, RandomIter iter) :
        body_(body), iter_(iter) {}

    void operator() (const tbb::blocked_range<std::size_t> &range) const
    {
        typedef typename std::iterator_traits<RandomIter>::value_type
            value_type;
        RandomIter iter = iter_ + range.begin();
        for (std::size_t i = range.begin(); i != range.end(); ++i, ++iter)
            *iter = static_cast<value_type>(body_(i));
    }

    private :

    const Body body_;
    const RandomIter iter_;
}; // class TBBParallelRepeatBodyRI

} // namespace vsmc::internal

/// \brief parallel_repeat with default partitioner
/// \ingroup TBBAlg
template <typename Body>
inline void tbb_parallel_repeat (std::size_t n, const Body &body)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<Body>(body));
}

/// \brief parallel_repeat with auto_partitioner
/// \ingroup TBBAlg
template <typename Body>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::auto_partitioner &partitioner)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<Body>(body),
            partitioner);
}

/// \brief parallel_repeat with simple_partitioner
/// \ingroup TBBAlg
template <typename Body>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::simple_partitioner &partitioner)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<Body>(body),
            partitioner);
}

/// \brief parallel_repeat with affinity_partitioner
/// \ingroup TBBAlg
template <typename Body>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        tbb::affinity_partitioner &partitioner)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<Body>(body),
            partitioner);
}

/// \brief parallel_repeat with default partitioner and output
/// \ingroup TBBAlg
template <typename Body, typename RandomIter>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        RandomIter iter)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBodyRI<Body, RandomIter>(body, iter));
}

/// \brief parallel_repeat with auto_partitioner and output
/// \ingroup TBBAlg
template <typename Body, typename RandomIter>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        RandomIter iter, const tbb::auto_partitioner &partitioner)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBodyRI<Body, RandomIter>(body, iter),
            partitioner);
}

/// \brief parallel_repeat with simple_partitioner and output
/// \ingroup TBBAlg
template <typename Body, typename RandomIter>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        RandomIter iter, const tbb::simple_partitioner &partitioner)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBodyRI<Body, RandomIter>(body, iter),
            partitioner);
}
/// \brief parallel_repeat with affinity_partitioner and output
/// \ingroup TBBAlg
template <typename Body, typename RandomIter>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        RandomIter iter, tbb::affinity_partitioner &partitioner)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBodyRI<Body, RandomIter>(body, iter),
            partitioner);
}

#if __TBB_TASK_GROUP_CONTEXT

/// \brief parallel_repeat with auto_partitioner
/// \ingroup TBBAlg
template <typename Body>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::auto_partitioner &partitioner,
        tbb::task_group_context &context)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<Body>(body),
            partitioner, context);
}

/// \brief parallel_repeat with simple_partitioner
/// \ingroup TBBAlg
template <typename Body>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::simple_partitioner &partitioner,
        tbb::task_group_context &context)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<Body>(body),
            partitioner, context);
}

/// \brief parallel_repeat with affinity_partitioner
/// \ingroup TBBAlg
template <typename Body>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        tbb::affinity_partitioner &partitioner,
        tbb::task_group_context &context)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<Body>(body),
            partitioner, context);
}

/// \brief parallel_repeat with auto_partitioner and output
/// \ingroup TBBAlg
template <typename Body, typename RandomIter>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        RandomIter iter, const tbb::auto_partitioner &partitioner,
        tbb::task_group_context &context)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBodyRI<Body, RandomIter>(body, iter),
            partitioner, context);
}

/// \brief parallel_repeat with simple_partitioner and output
/// \ingroup TBBAlg
template <typename Body, typename RandomIter>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        RandomIter iter, const tbb::simple_partitioner &partitioner,
        tbb::task_group_context &context)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBodyRI<Body, RandomIter>(body, iter),
            partitioner, context);
}

/// \brief parallel_repeat with affinity_partitioner and output
/// \ingroup TBBAlg
template <typename Body, typename RandomIter>
inline void tbb_parallel_repeat (std::size_t n, const Body &body,
        RandomIter iter, tbb::affinity_partitioner &partitioner,
        tbb::task_group_context &context)
{
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBodyRI<Body, RandomIter>(body, iter),
            partitioner, context);
}

#endif // __TBB_TASK_GROUP_CONTEXT

} // namespace vsmc

#endif // VSMC_TBB_PARALLEL_REPEAT_HPP
