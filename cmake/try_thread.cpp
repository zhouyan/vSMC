#include <vector>
#include <iostream>
#include <vsmc/cxx11/thread.hpp>

#ifndef _GLIBCXX_HAS_GTHREADS
#warning _GLIBCXX_HAS_GTHREADS
#endif

#ifndef _GLIBCXX_USE_C99_STDINT_TR1
#warning _GLIBCXX_USE_C99_STDINT_TR1
#endif

class work
{
    public :

    void operator() ()
    {
        std::cout << "Hello" << std::endl;
    }
};

int main ()
{
#if VSMC_HAS_CXX11LIB_THREAD
    std::vector<std::thread> tg;
    for (unsigned i = 0; i != 4; ++i)
        tg.push_back(std::thread(work()));
    for (unsigned i = 0; i != 4; ++i)
        tg[i].join();
#else
    boost::thread_group tg;
    for (unsigned i = 0; i != 4; ++i)
        tg.create_thread(work());
    tg.join_all();
#endif

    return 0;
}
