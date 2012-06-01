#include <cassert>
#include <cstddef>
#include <vSMC/internal/function.hpp>

int fn (int a)
{
    return 2 * a;
}

class cl
{
    public :

    typedef vSMC::function<int (int)> f_type;

    cl (f_type f = NULL) : f_(f) {}

    int operator() (int a)
    {
        return f_(a);
    }

    private :

    f_type f_;
};

int main ()
{
    vSMC::function<int (int)> f;
    assert(!bool(f));

    f = fn;
    assert(bool(f));
    assert(f(2) == 4);

    f = NULL;
    assert(!bool(f));

    cl c;
    c = cl(fn);
    assert(c(2) == 4);

    return 0;
}
