//============================================================================
// vSMC/cmake/vSMCFindOpenMP.cpp
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

#include <vector>
#include <cassert>
#include <omp.h>

class Element
{
    public :

    Element (int i, std::vector<int> *src) : i_(i), src_(src) {}

    Element (const Element &other) : i_(other.i_), src_(other.src_) {}

    int i () const {return i_;}
    int size () const {return src_->size();}
    int src () const {return (*src_)[i_];}

    private :

    const int i_;
    std::vector<int> *const src_;
};

template <typename Derived>
class Base
{
    protected :

    virtual ~Base () {}

    void eval (int dim, Element elem, int *res)
    {
        eval_dispatch(dim, elem, res, &Derived::eval);
    }

    private :

    template <typename D>
    void eval_dispatch(int dim, Element elem, int *res,
            void (D::*) (int, Element, int *))
    {
        assert((dynamic_cast<Derived *>(this)));
        static_cast<Derived *>(this)->eval(dim, elem, res);
    }

    void eval_dispatch(int dim, Element elem, int *res,
            void (*) (int, Element, int *))
    {
        Derived::eval(dim, elem, res);
    }

    void eval_dispatch(int dim, Element elem, int *res,
            void (Base::*) (int, Element, int *))
    {
        for (int d = 0; d != dim; ++d)
            res[d] = elem.src();
    }
};

template <>
class Base<int>
{
    protected :

    virtual ~Base () {}
    virtual void eval (int dim, Element elem, int *res);
};

// A middle-man who provides the interface operator(), call Base::eval, and
// Base dispatch it to possible default behavior or Derived::eval
template <typename Derived>
class Evaluator : public Base<Derived>
{
    public :

    void operator() (int N , int dim, int *res)
    {
        std::vector<int> src(N);
        for (int i = 0; i != N; ++i)
            src[i] = i;

#pragma omp parallel for default(shared)
        for (int i = 0; i < N; ++i)
            this->eval(dim, Element(i, &src), res + i * dim);
    }
};

class StaticImpl : public Evaluator<StaticImpl>
{
    public :

    static void eval (int dim, Element elem, int *res)
    {
        for (int d = 0; d != dim; ++d)
            res[d] = elem.src();
    }
};

class NonStaticImpl : public Evaluator<NonStaticImpl>
{
    public :

    void eval (int dim, Element elem, int *r)
    {
        for (int d = 0; d != dim; ++d)
            r[d] = elem.src();
    }
};

class VirtualImpl : public Evaluator<VirtualImpl>
{
    public :

    void eval (int dim, Element elem, int *r)
    {
        for (int d = 0; d != dim; ++d)
            r[d] = elem.src();
    }
};

class DefaultImpl : public Evaluator<DefaultImpl>
{};

int main ()
{
#ifndef _OPENMP
    assert(0);
#endif

    const int N = 5000000;
    const int Dim = 4;
    int *res = new int[N * Dim];

    StaticImpl static_impl;
    static_impl(N, Dim, res);
    for (int d = 0; d != Dim; ++d)
        for (int i = 0; i != N; ++i)
            assert(res[i * Dim + d] == i);

    NonStaticImpl non_static_impl;
    non_static_impl(N, Dim, res);
    for (int d = 0; d != Dim; ++d)
        for (int i = 0; i != N; ++i)
            assert(res[i * Dim + d] == i);

    VirtualImpl virtual_impl;
    virtual_impl(N, Dim, res);
    for (int d = 0; d != Dim; ++d)
        for (int i = 0; i != N; ++i)
            assert(res[i * Dim + d] == i);

    DefaultImpl default_impl;
    default_impl(N, Dim, res);
    for (int d = 0; d != Dim; ++d)
        for (int i = 0; i != N; ++i)
            assert(res[i * Dim + d] == i);

    delete [] res;

    return 0;
}
