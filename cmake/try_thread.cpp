#include <vsmc/internal/thread.hpp>

int main ()
{
    const int N = 1000;

    int x = 0;
    vsmc::internal::mutex m;
    for (int i = 0; i < N; ++i) {
        vsmc::internal::lock_guard<boost::mutex> lock(m);
        x = i;
    }
}
