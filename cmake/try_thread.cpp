#include <vsmc/cxx11/thread.hpp>

int main ()
{
    const int N = 1000;

    int x = 0;
    vsmc::cxx11::mutex m;
    for (int i = 0; i < N; ++i) {
        vsmc::cxx11::lock_guard<vsmc::cxx11::mutex> lock(m);
        x = i;
    }
}
