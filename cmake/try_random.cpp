#include <cassert>
#include <Random123/threefry.h>
#include <Random123/conventional/Engine.hpp>
#include <vSMC/internal/random.hpp>

int main ()
{
    int N = 1000000;
    r123::Engine<r123::Threefry4x64> eng(101);

    vSMC::internal::uniform_real_distribution<> runif(0, 1);
    double p = 0;
    double sum = 0;
    for (int i = 0; i != N; ++i) {
        p = runif(eng);
        assert(p >= 0 && p <= 1);
    }

    while (p < 0.2 || p > 0.8)
        p = runif(eng);

    int size = 20;
    int res = 0;
    vSMC::internal::binomial_distribution<> rbinom(size, p);
    for (int i = 0; i != N; ++i) {
        res = rbinom(eng);
        assert(res >= 0 && res <= size);
    }

    return 0;
}
