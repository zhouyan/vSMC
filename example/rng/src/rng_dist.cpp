#include "rng_dist.hpp"

int main(int argc, char **argv)
{
    VSMC_RNG_TEST_PRE(rng_dist);

    VSMC_RNG_DIST_2(
        std::uniform_int_distribution<int>, uniform_int, -100, 100);
    VSMC_RNG_DIST_2(
        std::uniform_real_distribution<double>, uniform_real, 0, 1);
    VSMC_RNG_DIST_1(std::bernoulli_distribution, bernoulli, 0.5);
    VSMC_RNG_DIST_2(std::binomial_distribution<int>, binomial, 100, 0.5);
    VSMC_RNG_DIST_2(
        std::negative_binomial_distribution<int>, negative_binomial, 100, 0.5);
    VSMC_RNG_DIST_1(std::geometric_distribution<int>, geometric, 0.5);
    VSMC_RNG_DIST_1(std::poisson_distribution<int>, poisson, 1);
    VSMC_RNG_DIST_1(std::poisson_distribution<int>, poisson, 0.01);
    VSMC_RNG_DIST_1(std::poisson_distribution<int>, poisson, 100);
    VSMC_RNG_DIST_1(std::exponential_distribution<double>, exponential, 1);
    VSMC_RNG_DIST_1(vsmc::ExponentialDistribution<double>, exponential, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<double>, gamma, 0.1, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<double>, gamma, 0.9, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<double>, gamma, 1, 1);
    VSMC_RNG_DIST_2(std::gamma_distribution<double>, gamma, 100, 1);
    VSMC_RNG_DIST_2(vsmc::GammaDistribution<double>, gamma, 0.1, 1);
    VSMC_RNG_DIST_2(vsmc::GammaDistribution<double>, gamma, 0.9, 1);
    VSMC_RNG_DIST_2(vsmc::GammaDistribution<double>, gamma, 1, 1);
    VSMC_RNG_DIST_2(vsmc::GammaDistribution<double>, gamma, 100, 1);
    VSMC_RNG_DIST_2(std::weibull_distribution<double>, weibull, 1, 1);
    VSMC_RNG_DIST_2(vsmc::WeibullDistribution<double>, weibull, 1, 1);
    VSMC_RNG_DIST_2(
        std::extreme_value_distribution<double>, extreme_value, 0, 1);
    VSMC_RNG_DIST_2(
        vsmc::ExtremeValueDistribution<double>, extreme_value, 0, 1);
    VSMC_RNG_DIST_2(std::normal_distribution<double>, normal, 0, 1);
    VSMC_RNG_DIST_2(vsmc::NormalDistribution<double>, normal, 0, 1);
    VSMC_RNG_DIST_2(std::lognormal_distribution<double>, lognormal, 0, 1);
    VSMC_RNG_DIST_2(vsmc::LognormalDistribution<double>, lognormal, 0, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<double>, chi_squared, 1);
    VSMC_RNG_DIST_1(std::chi_squared_distribution<double>, chi_squared, 100);
    VSMC_RNG_DIST_1(vsmc::ChiSquaredDistribution<double>, chi_squared, 1);
    VSMC_RNG_DIST_1(vsmc::ChiSquaredDistribution<double>, chi_squared, 100);
    VSMC_RNG_DIST_2(std::cauchy_distribution<double>, cauchy, 0, 1);
    VSMC_RNG_DIST_2(vsmc::CauchyDistribution<double>, cauchy, 0, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<double>, fisher_f, 1, 1);
    VSMC_RNG_DIST_2(std::fisher_f_distribution<double>, fisher_f, 100, 100);
    VSMC_RNG_DIST_2(vsmc::FisherFDistribution<double>, fisher_f, 1, 1);
    VSMC_RNG_DIST_2(vsmc::FisherFDistribution<double>, fisher_f, 100, 100);
    VSMC_RNG_DIST_1(std::student_t_distribution<double>, student_t, 1);
    VSMC_RNG_DIST_1(std::student_t_distribution<double>, student_t, 100);
    VSMC_RNG_DIST_1(vsmc::StudentTDistribution<double>, student_t, 1);
    VSMC_RNG_DIST_1(vsmc::StudentTDistribution<double>, student_t, 100);
    VSMC_RNG_DIST_2(vsmc::LaplaceDistribution<double>, laplace, 0, 1);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<double>, stable, 1, 0, 0, 1);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<double>, stable, 0.1, 0, 0, 1);
    VSMC_RNG_DIST_4(vsmc::StableDistribution<double>, stable, 1.5, 0, 0, 1);

    VSMC_RNG_TEST_POST;

    return 0;
}
