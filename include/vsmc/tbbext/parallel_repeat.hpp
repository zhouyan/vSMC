#ifndef VSMC_TBBEXT_PARALLEL_REPEAT_HPP
#define VSMC_TBBEXT_PARALLEL_REPEAT_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/type_traits.hpp>
#include <tbb/tbb.h>

namespace vsmc {

namespace tbbext {

namespace internal {

template <typename ResultType, typename Body,
         bool = cxx11::is_void<ResultType>::value>
class ParallelRepeatBody
{
    public :

    ParallelRepeatBody (ResultType *result, const Body &body) :
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
}; // class ParallelRepeatBody

template <typename ResultType, typename Body>
class ParallelRepeatBody<ResultType, Body, true>
{
    public :

    ParallelRepeatBody (ResultType *, const Body &body) : body_(body) {}

    void operator() (const tbb::blocked_range<std::size_t> &range) const
    {
        for (std::size_t i = range.begin(); i != range.end(); ++i)
            body_(i);
    }

    private :

    Body body_;
}; // class ParallelRepeatBody

template <typename ResultType, bool = cxx11::is_void<ResultType>::value>
class ResultVector
{
    public :

    typedef std::vector<ResultType> type;

    ResultVector (std::size_t n) : vector_(n) {}

    ResultType *pointer () {return &vector_[0];}

    type result () {return vector_;}

    private :

    type vector_;
}; // class ResultVector

template <typename ResultType>
class ResultVector<ResultType, true>
{
    public :

    typedef std::size_t type;

    ResultVector (std::size_t n) : size_(n) {}

    ResultType *pointer () {return VSMC_NULLPTR;}

    type result () {return size_;}

    private :

    std::size_t size_;
}; // class ResultVector

} // namespace vsmc::tbbext::internal

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::ResultVector<ResultType>::type
parallel_repeat (std::size_t n, const Body &body)
{
    internal::ResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::ParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body));

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::ResultVector<ResultType>::type
parallel_repeat (std::size_t n, const Body &body,
        const tbb::auto_partitioner &partitioner)
{
    internal::ResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::ParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::ResultVector<ResultType>::type
parallel_repeat (std::size_t n, const Body &body,
        const tbb::simple_partitioner &partitioner)
{
    internal::ResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::ParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::ResultVector<ResultType>::type
parallel_repeat (std::size_t n, const Body &body,
        tbb::affinity_partitioner &partitioner)
{
    internal::ResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::ParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner);

    return vec.result();
}

#if __TBB_TASK_GROUP_CONTEXT

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::ResultVector<ResultType>::type
parallel_repeat (std::size_t n, const Body &body,
        const tbb::auto_partitioner &partitioner,
        tbb::task_group_context &context)
{
    internal::ResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::ParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner, context);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::ResultVector<ResultType>::type
parallel_repeat (std::size_t n, const Body &body,
        const tbb::simple_partitioner &partitioner,
        tbb::task_group_context &context)
{
    internal::ResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::ParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner, context);

    return vec.result();
}

/// \brief Intel TBB parallel repeat algorithm
/// \ingroup TBBAlg
template <typename ResultType, typename Body>
inline typename internal::ResultVector<ResultType>::type
parallel_repeat (std::size_t n, const Body &body,
        tbb::affinity_partitioner &partitioner,
        tbb::task_group_context &context)
{
    internal::ResultVector<ResultType> vec(n);
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, n),
            internal::ParallelRepeatBody<ResultType, Body>(
                vec.pointer(), body), partitioner, context);

    return vec.result();
}

#endif // __TBB_TASK_GROUP_CONTEXT

} // namespace vsmc::tbbext

} // namespace vsmc

#endif // VSMC_TBBEXT_PARALLEL_REPEAT_HPP
