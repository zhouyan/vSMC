#include "rng_dist.hpp"

int main(int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_dist);

    VSMC_RNG_DIST_1(vsmc::StableDistribution<>, 1);
    VSMC_RNG_DIST_1(vsmc::StableDistribution<>, 0.1);
    VSMC_RNG_DIST_1(vsmc::StableDistribution<>, 10);
    VSMC_RNG_DIST_2(vsmc::UniformRealOpenOpenDistribution<>, 0, 1);
    VSMC_RNG_DIST_2(vsmc::UniformRealOpenClosedDistribution<>, 0, 1);
    VSMC_RNG_DIST_2(vsmc::UniformRealClosedOpenDistribution<>, 0, 1);
    VSMC_RNG_DIST_2(vsmc::UniformRealClosedClosedDistribution<>, 0, 1);
    VSMC_RNG_DIST_2(std::uniform_real_distribution<>, 0, 1);
    VSMC_RNG_DIST_2(std::uniform_int_distribution<>, -100, 100);
    VSMC_RNG_DIST_1(std::bernoulli_distribution, 0.5);
    VSMC_RNG_DIST_2(std::binomial_distribution<>, 100, 0.5);
    VSMC_RNG_DIST_2(std::negative_binomial_distribution<>, 100, 0.5);
    VSMC_RNG_DIST_1(std::geometric_distribution<>, 0.5);
    VSMC_RNG_DIST_1(std::poisson_distribution<>, 1);
    VSMC_RNG_DIST_1(std::poisson_distribution<>, 0.01);
    VSMC_RNG_DIST_1(std::poisson_distribution<>, 100);
    VSMC_RNG_DIST_1(std::exponential_distribution<>, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<>, 1, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<>, 0.01, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<>, 100, 1);
    VSMC_RNG_DIST_2(std::weibull_distribution<>, 1, 1);
    VSMC_RNG_DIST_2(std::extreme_value_distribution<>, 0, 1);
    VSMC_RNG_DIST_2(std::normal_distribution<>, 0, 1);
    VSMC_RNG_DIST_2(std::lognormal_distribution<>, 0, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<>, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<>, 100);
    VSMC_RNG_DIST_2(std::cauchy_distribution<>, 0, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<>, 1, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<>, 100, 100);
    VSMC_RNG_DIST_1(std::student_t_distribution<>, 1);
    VSMC_RNG_DIST_1(std::student_t_distribution<>, 100);

    VSMC_RNG_TEST_POST;

    return 0;
}
