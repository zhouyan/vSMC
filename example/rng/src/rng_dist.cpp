#include "rng_dist.hpp"

int main(int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_dist);

    using UniformRealCC =
        vsmc::UniformRealDistribution<double, vsmc::Closed, vsmc::Closed>;
    using UniformRealCO =
        vsmc::UniformRealDistribution<double, vsmc::Closed, vsmc::Open>;
    using UniformRealOC =
        vsmc::UniformRealDistribution<double, vsmc::Open, vsmc::Closed>;
    using UniformRealOO =
        vsmc::UniformRealDistribution<double, vsmc::Open, vsmc::Open>;

    VSMC_RNG_DIST_2(
        std::uniform_int_distribution<int>, uniform_int, -100, 100);
    VSMC_RNG_DIST_2(
        std::uniform_real_distribution<double>, uniform_real, 0, 1);
    VSMC_RNG_DIST_2(UniformRealCC, uniform_real_cc, 0, 1);
    VSMC_RNG_DIST_2(UniformRealCO, uniform_real_co, 0, 1);
    VSMC_RNG_DIST_2(UniformRealOC, uniform_real_oc, 0, 1);
    VSMC_RNG_DIST_2(UniformRealOO, uniform_real_oo, 0, 1);
    VSMC_RNG_DIST_1(std::bernoulli_distribution, bernoulli, 0.5);
    VSMC_RNG_DIST_2(std::binomial_distribution<int>, binomial, 100, 0.5);
    VSMC_RNG_DIST_2(
        std::negative_binomial_distribution<int>, negative_binomial, 100, 0.5);
    VSMC_RNG_DIST_1(std::geometric_distribution<int>, geometric, 0.5);
    VSMC_RNG_DIST_1(std::poisson_distribution<int>, poisson, 1);
    VSMC_RNG_DIST_1(std::poisson_distribution<int>, poisson, 0.01);
    VSMC_RNG_DIST_1(std::poisson_distribution<int>, poisson, 100);
    VSMC_RNG_DIST_1(std::exponential_distribution<double>, exponential, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<double>, gamma, 1, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<double>, gamma, 0.01, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<double>, gamma, 100, 1);
    VSMC_RNG_DIST_2(std::weibull_distribution<double>, weibull, 1, 1);
    VSMC_RNG_DIST_2(
        std::extreme_value_distribution<double>, extreme_value, 0, 1);
    VSMC_RNG_DIST_2(std::normal_distribution<double>, normal, 0, 1);
    VSMC_RNG_DIST_2(std::lognormal_distribution<double>, lognormal, 0, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<double>, chi_squared, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<double>, chi_squared, 100);
    VSMC_RNG_DIST_2(std::cauchy_distribution<double>, cauchy, 0, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<double>, fisher_f, 1, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<double>, fisher_f, 100, 100);
    VSMC_RNG_DIST_1(std::student_t_distribution<double>, student_t, 1);
    VSMC_RNG_DIST_1(std::student_t_distribution<double>, student_t, 100);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<double>, stable, 1, 0, 0, 1);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<double>, stable, 0.1, 0, 0, 1);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<double>, stable, 1.5, 0, 0, 1);

    VSMC_RNG_TEST_POST;

    return 0;
}
