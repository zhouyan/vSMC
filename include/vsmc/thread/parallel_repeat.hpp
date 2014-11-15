//============================================================================
// vSMC/include/vsmc/thread/parallel_repeat.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_THREAD_PARALLEL_REPEAT_HPP
#define VSMC_THREAD_PARALLEL_REPEAT_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/thread/blocked_range.hpp>
#include <vsmc/thread/parallel_for.hpp>

namespace vsmc {

/// \brief Parallel repeat using std::thread
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType:
/// ~~~{.cpp}
/// WorkType work;
/// work(std::size_t);
/// ~~~
template <typename WorkType>
inline void parallel_repeat (std::size_t n, WorkType &&work)
{
    struct body
    {
        body (const WorkType &w) : work_(w) {}

        body (WorkType &&w) : work_(std::move(w)) {}

        void operator() (const BlockedRange<std::size_t> &range) const
        {
            for (std::size_t i = range.begin(); i != range.end(); ++i)
                work_(i);
        }

        private :

        WorkType work_;
    };
    parallel_for(BlockedRange<std::size_t>(0, n),
            body(std::forward<WorkType>(work)));
}

} // namesapce vsmc

#endif // VSMC_THREAD_PARALLEL_REPEAT_HPP
