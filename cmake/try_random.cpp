#include <cassert>
#include <vSMC/internal/random.hpp>

int main ()
{
    int N = 100000;
    r123::Engine<V_SMC_CRNG_TYPE> eng(V_SMC_CRNG_SEED);

    vSMC::internal::uniform_real_distribution<> runif(0, 1);
    for (int i = 0; i != N; ++i) {
        double u = runif(eng);
        assert(u >= 0 && u <= 1);
    }

    vSMC::internal::binomial_distribution<> rbinom(20, 0.7);
    for (int i = 0; i != N; ++i) {
        int b = rbinom(eng);
        assert(b >= 0 && b <= 20);
    }

    return 0;
}
