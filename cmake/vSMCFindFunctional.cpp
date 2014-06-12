#include <cassert>
#include <vsmc/cxx11/functional.hpp>
#include <vsmc/internal/defines.hpp>

int fn (int a, double, double, double)
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
