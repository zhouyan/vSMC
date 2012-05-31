#include <cassert>
#include <vSMC/rng/random.hpp>

int main ()
{
    int N = 10000;
    r123::Engine<V_SMC_CBRNG_TYPE> eng(V_SMC_CBRNG_SEED);

    vSMC::rng::uniform_real_distribution<> runif(0, 1);
    for (int i = 0; i != N; ++i) {
        int u = runif(eng);
        assert(u >= 0 && u <= 100);
    }

    vSMC::rng::uniform_int_distribution<> ruint(1, 100);
    for (int i = 0; i != N; ++i) {
        int u = ruint(eng);
        assert(u >= 1 && u <= 100);
    }

    vSMC::rng::bernoulli_distribution rbern(0.5);
    for (int i = 0; i != N; ++i){
        int b = rbern(eng);
        assert(b == 0 || b == 1);
    }

    vSMC::rng::binomial_distribution<> rbinom(20, 0.7);
    for (int i = 0; i != N; ++i) {
        int b = rbinom(eng);
        assert(b >= 0 && b <= 20);
    }

    vSMC::rng::gamma_distribution<> rgamma(1, 1);
    for (int i = 0; i != N; ++i) {
        double g = rgamma(eng);
        assert(g >= 0);
    }

    vSMC::rng::lognormal_distribution<> rlnom(1, 1);
    for (int i = 0; i != N; ++i) {
        double l = rlnom(eng);
        assert(l >= 0);
    }

    vSMC::rng::normal_distribution<> rnorm(0, 1);
    for (int i = 0; i != N; ++i) {
        double n = rnorm(eng);
    }

    return 0;
}
