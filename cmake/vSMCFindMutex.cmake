SET (VSMC_MUTEX_TEST_SOURCE "
#include <vsmc/cxx11/mutex.hpp>

vsmc::cxx11::mutex mtx;
int mtx_d;

void op1 ()
{
    vsmc::cxx11::lock_guard<vsmc::cxx11::mutex> guard(mtx);
    mtx_d = 1;
}

void op2 ()
{
    vsmc::cxx11::lock_guard<vsmc::cxx11::mutex> guard(mtx);
    mtx_d = 2;
}

int main ()
{
    op1();
    op2();

    return 0;
}
")
