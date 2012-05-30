#include <cassert>
#include <vSMC/internal/random.hpp>

int main ()
{
    int N = 100000;
    r123::Engine<V_SMC_CRNG_TYPE> eng(V_SMC_CRNG_SEED);

#ifdef V_SMC_USE_BOOST_RANDOM
    vSMC::rng::bernoulli_distribution<> rbern(0.5);
#else
    vSMC::rng::bernoulli_distribution rbern(0.5);
#endif
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

    vSMC::rng::uniform_real_distribution<> ruint(1, 100);
    for (int i = 0; i != N; ++i) {
        int u = ruint(eng);
        assert(u >= 1 && u <= 100);
    }

    vSMC::rng::uniform_real_distribution<> runif(0, 1);
    for (int i = 0; i != N; ++i) {
        double u = runif(eng);
        assert(u >= 0 && u <= 1);
    }

    return 0;
}
