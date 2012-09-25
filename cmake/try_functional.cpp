#include <cassert>
#include <vsmc/cxx11/functional.hpp>

int fn (int a, double, double, double)
{
    return 2 * a;
}

class cl
{
    public :

    typedef vsmc::cxx11::function<int (int, double, double, double)> f_type;

    cl (f_type f = 0) : f_(f) {}

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

    return 0;
}
