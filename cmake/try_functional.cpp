#include <cassert>
#include <vsmc/cxx11/functional.hpp>

int fn (int a, double, double, double)
{
    return 2 * a;
}

void gn (int &a)
{
    a *= 2;
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
    vsmc::cxx11::function<int (int, double, double, double)> f;
    assert(!bool(f));

    f = fn;
    assert(bool(f));
    assert(f(2, 0, 0, 0) == 4);

    f = VSMC_NULLPTR;
    assert(!bool(f));

    cl c;
    c = cl(fn);
    assert(c(2, 0, 0, 0) == 4);

    int a = 1;
    gn(vsmc::cxx11::ref(a));
    assert(a == 2);

    return 0;
}
