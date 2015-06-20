#include "rng_dist.hpp"

int main(int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_dist);

    VSMC_RNG_DIST_2(std::uniform_int_distribution<>, uniform_int, -100, 100);
    VSMC_RNG_DIST_2(std::uniform_real_distribution<>, uniform_real, 0, 1);
    VSMC_RNG_DIST_1(std::bernoulli_distribution, bernoulli, 0.5);
    VSMC_RNG_DIST_2(std::binomial_distribution<>, binomial, 100, 0.5);
    VSMC_RNG_DIST_2(
        std::negative_binomial_distribution<>, negative_binomial, 100, 0.5);
    VSMC_RNG_DIST_1(std::geometric_distribution<>, geometric, 0.5);
    VSMC_RNG_DIST_1(std::poisson_distribution<>, poisson, 1);
    VSMC_RNG_DIST_1(std::poisson_distribution<>, poisson, 0.01);
    VSMC_RNG_DIST_1(std::poisson_distribution<>, poisson, 100);
    VSMC_RNG_DIST_1(std::exponential_distribution<>, exponential, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<>, gamma, 1, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<>, gamma, 0.01, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<>, gamma, 100, 1);
    VSMC_RNG_DIST_2(std::weibull_distribution<>, weibull, 1, 1);
    VSMC_RNG_DIST_2(std::extreme_value_distribution<>, extreme_value, 0, 1);
    VSMC_RNG_DIST_2(std::normal_distribution<>, normal, 0, 1);
    VSMC_RNG_DIST_2(std::lognormal_distribution<>, lognormal, 0, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<>, chi_squared, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<>, chi_squared, 100);
    VSMC_RNG_DIST_2(std::cauchy_distribution<>, cauchy, 0, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<>, fisher_f, 1, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<>, fisher_f, 100, 100);
    VSMC_RNG_DIST_1(std::student_t_distribution<>, student_t, 1);
    VSMC_RNG_DIST_1(std::student_t_distribution<>, student_t, 100);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<>, stable, 1, 0, 0, 1);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<>, stable, 0.1, 0, 0, 1);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<>, stable, 1.5, 0, 0, 1);

    VSMC_RNG_TEST_POST;

    return 0;
}
