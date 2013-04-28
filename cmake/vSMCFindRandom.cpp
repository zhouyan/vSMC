#include <cassert>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/utility/rng_set.hpp>
#include <vector>

int main ()
{
    const int N = 1000;
    vsmc::traits::RngSetTypeTrait<vsmc::NullType>::type eng(N);

    vsmc::cxx11::uniform_real_distribution<> runif(0, 1);
    for (int i = 0; i != N; ++i) {
        double u = runif(eng.rng(i));
        assert(u >= 0 && u <= 100);
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

    std::vector<double> w(3);
    w[0] = 0.2;
    w[1] = 0.3;
    w[2] = 0.4;
    vsmc::cxx11::discrete_distribution<> rdist(w.begin(), w.end());
    for (int i = 0; i != N; ++i) {
        int d = rdist(eng.rng(i));
        assert(d >= 0 && d < 3);
    }

    return 0;
}
