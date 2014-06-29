#ifndef VSMC_TBB_PARALLEL_REPEAT_HPP
#define VSMC_TBB_PARALLEL_REPEAT_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace vsmc {

namespace internal {

template <typename ResultType, typename Body,
         bool = cxx11::is_void<ResultType>::value>
class TBBParallelRepeatBody
{
    public :

    TBBParallelRepeatBody (ResultType *result, const Body &body) :
        result_(result), body_(body) {}

    void operator() (const tbb::blocked_range<std::size_t> &range) const
    {
        ResultType *const result = result_;
        for (std::size_t i = range.begin(); i != range.end(); ++i)
            result[i] = static_cast<ResultType>(body_(i));
    }

    private :

    ResultType *const result_;
    Body body_;
}; // class TBBParallelRepeatBody

template <typename ResultType, typename Body>
class TBBParallelRepeatBody<ResultType, Body, true>
{
    public :

    TBBParallelRepeatBody (ResultType *, const Body &body) : body_(body) {}

    void operator() (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i)
            body_(i);
    }

    private :

    Body body_;
}; // class TBBParallelRepeatBody

template <typename ResultType, bool = cxx11::is_void<ResultType>::value>
class TBBResultVector
{
    public :

    typedef std::vector<ResultType> type;

    TBBResultVector (std::size_t n) : vector_(n) {}

    ResultType *pointer () {return &vector_[0];}

    type result () {return vector_;}

    private :

    type vector_;
}; // class TBBResultVector

template <typename ResultType>
class TBBResultVector<ResultType, true>
{
    public :

    typedef std::size_t type;

    TBBResultVector (std::size_t n) : size_(n) {}

    ResultType *pointer () {return VSMC_NULLPTR;}

    type result () {return size_;}

    private :

    std::size_t size_;
}; // class TBBResultVector

} // namespace vsmc::internal

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::TBBResultVector<ResultType>::type
tbb_parallel_repeat (std::size_t n, const Body &body)
{
    internal::TBBResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body));

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::TBBResultVector<ResultType>::type
tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::auto_partitioner &partitioner)
{
    internal::TBBResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::TBBResultVector<ResultType>::type
tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::simple_partitioner &partitioner)
{
    internal::TBBResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::TBBResultVector<ResultType>::type
tbb_parallel_repeat (std::size_t n, const Body &body,
        tbb::affinity_partitioner &partitioner)
{
    internal::TBBResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner);

    return vec.result();
}

#if __TBB_TASK_GROUP_CONTEXT

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::TBBResultVector<ResultType>::type
tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::auto_partitioner &partitioner,
        tbb::task_group_context &context)
{
    internal::TBBResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner, context);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::TBBResultVector<ResultType>::type
tbb_parallel_repeat (std::size_t n, const Body &body,
        const tbb::simple_partitioner &partitioner,
        tbb::task_group_context &context)
{
    internal::TBBResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner, context);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::TBBResultVector<ResultType>::type
tbb_parallel_repeat (std::size_t n, const Body &body,
        tbb::affinity_partitioner &partitioner,
        tbb::task_group_context &context)
{
    internal::TBBResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::TBBParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner, context);

    return vec.result();
}

#endif // __TBB_TASK_GROUP_CONTEXT

} // namespace vsmc

#endif // VSMC_TBB_PARALLEL_REPEAT_HPP
