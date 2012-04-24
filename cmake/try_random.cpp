#include <cassert>
#include <vSMC/internal/random.hpp>

#ifdef V_SMC_USE_STD_RANDOM
using std::binomial_distribution;
using std::gamma_distribution;
using std::lognormal_distribution;
using std::normal_distribution;
using std::uniform_real_distribution;
#else
#include <boost/random/gamma_distribution.hpp>
#include <boost/random/lognormal_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
using boost::random::binomial_distribution;
using boost::random::gamma_distribution;
using boost::random::lognormal_distribution;
using boost::random::normal_distribution;
using boost::random::uniform_real_distribution;
#endif

int main ()
{
    int N = 100000;
    r123::Engine<V_SMC_CRNG_TYPE> eng(V_SMC_CRNG_SEED);

    binomial_distribution<> rbinom(20, 0.7);
    for (int i = 0; i != N; ++i) {
        int b = rbinom(eng);
        assert(b >= 0 && b <= 20);
    }

    gamma_distribution<> rgamma(1, 1);
    for (int i = 0; i != N; ++i) {
        double g = rgamma(eng);
        assert(g >= 0);
    }

    lognormal_distribution<> rlnom(1, 1);
    for (int i = 0; i != N; ++i) {
        double l = rlnom(eng);
        assert(l >= 0);
    }

    normal_distribution<> rnorm(0, 1);
    for (int i = 0; i != N; ++i) {
        double n = rnorm(eng);
    }

    uniform_real_distribution<> runif(0, 1);
    for (int i = 0; i != N; ++i) {
        double u = runif(eng);
        assert(u >= 0 && u <= 1);
    }

    return 0;
}
