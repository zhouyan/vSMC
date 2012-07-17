#include <vector>
#include <cassert>

class Element
{
    public :

    Element (int i, std::vector<int> *src) : i_(i), src_(src) {}

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

#pragma omp parallel for default(none) shared(N, dim, src, res)
        for (int i = 0; i < src.size(); ++i)
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
