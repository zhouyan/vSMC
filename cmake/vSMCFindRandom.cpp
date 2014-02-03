#include <cassert>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/rng/rng_set.hpp>

#if defined(VSMC_RANDOM123_AES_FOUND)
#include <Random123/aes.h>
#endif

#if defined(VSMC_RANDOM123_ARS_FOUND)
#include <Random123/ars.h>
#endif

int main ()
{
    const int N = 1000;

    vsmc::cxx11::uniform_real_distribution<> ru01(0, 1);
    double u;
    {vsmc::cxx11::mt19937       eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::mt19937_64    eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::minstd_rand0  eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::minstd_rand   eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::ranlux24_base eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::ranlux48_base eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::ranlux24      eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::ranlux48      eng; u = ru01(eng); assert(u >=0 && u <= 1);}
    {vsmc::cxx11::knuth_b       eng; u = ru01(eng); assert(u >=0 && u <= 1);}

#if defined(VSMC_RANDOM123_AES_FOUND)
    vsmc::RngSet<r123::Engine<r123::AESNI4x32>, vsmc::VectorRng> eng(N);
#elif defined(VSMC_RANDOM123_ARS_FOUND)
    vsmc::RngSet<r123::Engine<r123::ARS4x32_R<7> >, vsmc::VectorRng> eng(N);
#else
    vsmc::traits::RngSetTypeTrait<vsmc::NullType>::type eng(N);
#endif

    vsmc::cxx11::uniform_real_distribution<> runif(0, 1);
    for (int i = 0; i != N; ++i) {
        double u = runif(eng.rng(i));
        assert(u >= 0 && u <= 1);
    }

    vsmc::cxx11::uniform_int_distribution<std::size_t> ruint(1, 100);
    for (int i = 0; i != N; ++i) {
        std::size_t u = ruint(eng.rng(i));
        assert(u >= 1 && u <= 100);
    }

    vsmc::cxx11::bernoulli_distribution rbern(0.5);
    for (int i = 0; i != N; ++i){
        int b = rbern(eng.rng(i));
        assert(b == 0 || b == 1);
    }

    vsmc::cxx11::binomial_distribution<> rbinom(20, 0.7);
    for (int i = 0; i != N; ++i) {
        int b = rbinom(eng.rng(i));
        assert(b >= 0 && b <= 20);
    }

    vsmc::cxx11::gamma_distribution<> rgamma(1, 1);
    for (int i = 0; i != N; ++i) {
        double g = rgamma(eng.rng(i));
        assert(g >= 0);
    }

    vsmc::cxx11::lognormal_distribution<> rlnom(1, 1);
    for (int i = 0; i != N; ++i) {
        double l = rlnom(eng.rng(i));
        assert(l >= 0);
    }

    vsmc::cxx11::normal_distribution<> rnorm(0, 1);
    for (int i = 0; i != N; ++i) {
        double n = rnorm(eng.rng(i));
    }

    return 0;
}
