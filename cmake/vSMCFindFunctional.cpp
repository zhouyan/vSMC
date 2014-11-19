//============================================================================
// vSMC/cmake/vSMCFindFunctional.cpp
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

#include <cassert>
#include <vsmc/cxx11/functional.hpp>
#include <vsmc/internal/defines.hpp>

inline int fn (int a, double, double, double)
{
    return 2 * a;
}

class cl
{
    public :

    typedef vsmc::cxx11::function<int (int, double, double, double)> f_type;

    cl (f_type f = VSMC_NULLPTR) : f_(f) {}

    int operator() (int a, double, double, double)
    {
        return f_(a, 0, 0, 0);
    }

    private :

    f_type f_;
};

int main ()
{
    typedef vsmc::cxx11::function<int (int, double, double, double)> f_type;
    f_type f;
    assert(!bool(f));

    f = fn;
    assert(bool(f));
    assert(f(2, 0, 0, 0) == 4);

    f = VSMC_NULLPTR;
    assert(!bool(f));
    f_type f1 = f_type();
    assert(!bool(f1));
    f_type f2(f1);
    assert(!bool(f2));
    f_type f3(f2);
    assert(!bool(f3));

    cl c;
    c = cl(fn);
    assert(c(2, 0, 0, 0) == 4);

    return 0;
}
