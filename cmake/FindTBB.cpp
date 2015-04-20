//============================================================================
// vSMC/cmake/FindTBB.cpp
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

#include <iostream>
#include <cassert>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

class say
{
    public:
    template <typename IntType>
    void operator()(const tbb::blocked_range<IntType> &block) const
    {
        IntType sum = 0;
        for (IntType i = block.begin(); i != block.end(); ++i)
            sum += i;
        std::cout << sum << std::endl;
    }
};

class sum
{
    public:
    sum() : res_(0) {}

    sum(const sum &other, tbb::split) : res_(0) {}

    void join(const sum &other) { res_ += other.res_; }

    template <typename IntType>
    void operator()(const tbb::blocked_range<IntType> &block)
    {
        long r = res_;
        for (IntType i = block.begin(); i != block.end(); ++i)
            r += i;
        res_ = r;
    }

    long res() const { return res_; }

    private:
    long res_;
};

int main()
{
    tbb::parallel_for(tbb::blocked_range<int>(0, 100), say());
    sum s;
    tbb::parallel_reduce(tbb::blocked_range<int>(0, 100), s);
    assert(s.res() == 4950);
    return 0;
}
