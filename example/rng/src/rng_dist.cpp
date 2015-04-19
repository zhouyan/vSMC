#include "rng_dist.hpp"

int main(int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_dist);

    VSMC_RNG_DIST_T2(uniform_int, -100, 100);
    VSMC_RNG_DIST_T2(uniform_real, 0, 1);
    VSMC_RNG_DIST_B1(bernoulli, 0.5);
    VSMC_RNG_DIST_T2(binomial, 100, 0.5);
    VSMC_RNG_DIST_T2(negative_binomial, 100, 0.5);
    VSMC_RNG_DIST_T1(geometric, 0.5);
    VSMC_RNG_DIST_T1(poisson, 1);
    VSMC_RNG_DIST_T1(poisson, 0.01);
    VSMC_RNG_DIST_T1(poisson, 100);
    VSMC_RNG_DIST_T1(exponential, 1);
    VSMC_RNG_DIST_T2(gamma, 1, 1);
    VSMC_RNG_DIST_T2(gamma, 0.01, 1);
    VSMC_RNG_DIST_T2(gamma, 100, 1);
    VSMC_RNG_DIST_T2(weibull, 1, 1);
    VSMC_RNG_DIST_T2(extreme_value, 0, 1);
    VSMC_RNG_DIST_T2(normal, 0, 1);
    VSMC_RNG_DIST_T2(lognormal, 0, 1);
    VSMC_RNG_DIST_T1(chi_squared, 1);
    VSMC_RNG_DIST_T1(chi_squared, 100);
    VSMC_RNG_DIST_T2(cauchy, 0, 1);
    VSMC_RNG_DIST_T2(fisher_f, 1, 1);
    VSMC_RNG_DIST_T2(fisher_f, 100, 100);
    VSMC_RNG_DIST_T1(student_t, 1);
    VSMC_RNG_DIST_T1(student_t, 100);

    VSMC_RNG_TEST_POST;

    return 0;
}
