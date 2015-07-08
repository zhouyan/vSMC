#include "rng_dist.hpp"

int main(int argc, char **argv)
{
    std::size_t N = 10000;
    if (argc > 1)
        N = static_cast<std::size_t>(std::atoi(argv[1]));
    vsmc::Vector<std::string> names;
    vsmc::Vector<vsmc::StopWatch> sw;
    std::array<double, 1> param1;
    std::array<double, 2> param2;

    VSMC_RNG_DIST_2(UniformReal, std::uniform_real_distribution, 0, 1);
    VSMC_RNG_DIST_2(Normal, std::normal_distribution, 0, 1);
    VSMC_RNG_DIST_2(Lognormal, std::lognormal_distribution, 0, 1);
    VSMC_RNG_DIST_2(Cauchy, std::cauchy_distribution, 0, 1);
    VSMC_RNG_DIST_2(ExtremeValue, std::extreme_value_distribution, 0, 1);
    VSMC_RNG_DIST_2(Laplace, vsmc::LaplaceDistribution, 0, 1);
    VSMC_RNG_DIST_1(Exponential, std::exponential_distribution, 1);
    VSMC_RNG_DIST_1(Exponential, std::exponential_distribution, 0.1);
    VSMC_RNG_DIST_1(Exponential, std::exponential_distribution, 10);
    VSMC_RNG_DIST_2(Weibull, std::weibull_distribution, 1, 1);
    VSMC_RNG_DIST_2(Weibull, std::weibull_distribution, 0.1, 1);
    VSMC_RNG_DIST_2(Weibull, std::weibull_distribution, 10, 1);
    VSMC_RNG_DIST_2(Gamma, std::gamma_distribution, 1, 1);
    VSMC_RNG_DIST_2(Gamma, std::gamma_distribution, 0.1, 1);
    VSMC_RNG_DIST_2(Gamma, std::gamma_distribution, 0.5, 1);
    VSMC_RNG_DIST_2(Gamma, std::gamma_distribution, 0.7, 1);
    VSMC_RNG_DIST_2(Gamma, std::gamma_distribution, 0.9, 1);
    VSMC_RNG_DIST_2(Gamma, std::gamma_distribution, 1.5, 1);
    VSMC_RNG_DIST_1(ChiSquared, std::chi_squared_distribution, 0.2);
    VSMC_RNG_DIST_1(ChiSquared, std::chi_squared_distribution, 1);
    VSMC_RNG_DIST_1(ChiSquared, std::chi_squared_distribution, 1.5);
    VSMC_RNG_DIST_1(ChiSquared, std::chi_squared_distribution, 2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 0.2, 3);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 3);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 1, 3);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 0.2);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 1);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 1.5);
    VSMC_RNG_DIST_2(FisherF, std::fisher_f_distribution, 3, 3);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 1, 1);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 1, 0.5);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 1, 1.5);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 0.5, 1);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 1.5, 1);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 0.5, 0.5);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 0.9, 0.9);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 1.5, 1.5);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 1.5, 0.5);
    VSMC_RNG_DIST_2(Beta, vsmc::BetaDistribution, 0.5, 1.5);

    rng_dist_output_sw(names, sw);

    return 0;
}
