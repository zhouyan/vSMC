#include <cassert>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/core/rng.hpp>

int main ()
{
    int N = 10000;
    r123::Engine<VSMC_CBRNG_TYPE> eng(VSMC_RNG_SEED);

    vsmc::cxx11::uniform_real_distribution<> runif(0, 1);
    for (int i = 0; i != N; ++i) {
        int u = runif(eng);
        assert(u >= 0 && u <= 100);
    }

    vsmc::cxx11::uniform_int_distribution<> ruint(1, 100);
    for (int i = 0; i != N; ++i) {
        int u = ruint(eng);
        assert(u >= 1 && u <= 100);
    }

    vsmc::cxx11::bernoulli_distribution rbern(0.5);
    for (int i = 0; i != N; ++i){
        int b = rbern(eng);
        assert(b == 0 || b == 1);
    }

    vsmc::cxx11::binomial_distribution<> rbinom(20, 0.7);
    for (int i = 0; i != N; ++i) {
        int b = rbinom(eng);
        assert(b >= 0 && b <= 20);
    }

    vsmc::cxx11::gamma_distribution<> rgamma(1, 1);
    for (int i = 0; i != N; ++i) {
        double g = rgamma(eng);
        assert(g >= 0);
    }

    vsmc::cxx11::lognormal_distribution<> rlnom(1, 1);
    for (int i = 0; i != N; ++i) {
        double l = rlnom(eng);
        assert(l >= 0);
    }

    vsmc::cxx11::normal_distribution<> rnorm(0, 1);
    for (int i = 0; i != N; ++i) {
        double n = rnorm(eng);
    }

    return 0;
}
