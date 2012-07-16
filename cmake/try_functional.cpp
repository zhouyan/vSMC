#include <cassert>
#include <vsmc/cxx11/functional.hpp>

int fn (int a)
{
    return 2 * a;
}

class cl
{
    public :

    typedef vsmc::cxx11::function<int (int)> f_type;

    cl (f_type f = 0) : f_(f) {}

    int operator() (int a)
    {
        return f_(a);
    }

    private :

    f_type f_;
};

int main ()
{
    vsmc::cxx11::function<int (int)> f;
    assert(!bool(f));

    f = fn;
    assert(bool(f));
    assert(f(2) == 4);

    f = 0;
    assert(!bool(f));

    cl c;
    c = cl(fn);
    assert(c(2) == 4);

    return 0;
}
