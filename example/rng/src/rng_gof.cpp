//============================================================================
// vSMC/example/rng/src/rng_gof.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include "rng_gof.hpp"

int main(int argc, char **argv)
{
    std::size_t N = 10000;
    std::size_t M = 10;
    if (argc > 1)
        N = static_cast<std::size_t>(std::atoi(argv[1]));
    if (argc > 2)
        M = static_cast<std::size_t>(std::atoi(argv[2]));
    std::array<double, 1> param1;
    std::array<double, 2> param2;
    vsmc::Vector<std::string> names;
    vsmc::Vector<double> pval;

    VSMC_RNG_GOF_2(UniformReal, std::uniform_real_distribution, 0, 1);
    VSMC_RNG_GOF_2(Normal, std::normal_distribution, 0, 1);
    VSMC_RNG_GOF_1(Exponential, std::exponential_distribution, 1);
    VSMC_RNG_GOF_2(Laplace, boost::random::laplace_distribution, 0, 1);
    VSMC_RNG_GOF_2(Weibull, std::weibull_distribution, 1, 1);
    VSMC_RNG_GOF_2(Weibull, std::weibull_distribution, 0.1, 1);
    VSMC_RNG_GOF_2(Weibull, std::weibull_distribution, 10, 1);
    VSMC_RNG_GOF_2(Cauchy, std::cauchy_distribution, 0, 1);
    VSMC_RNG_GOF_1(Rayleigh, vsmc::RayleighDistribution, 1);
    VSMC_RNG_GOF_2(Lognormal, std::lognormal_distribution, 0, 1);
    VSMC_RNG_GOF_2(ExtremeValue, std::extreme_value_distribution, 0, 1);
    VSMC_RNG_GOF_2(Gamma, std::gamma_distribution, 1, 1);
    VSMC_RNG_GOF_2(Gamma, std::gamma_distribution, 0.1, 1);
    VSMC_RNG_GOF_2(Gamma, std::gamma_distribution, 0.5, 1);
    VSMC_RNG_GOF_2(Gamma, std::gamma_distribution, 0.7, 1);
    VSMC_RNG_GOF_2(Gamma, std::gamma_distribution, 0.9, 1);
    VSMC_RNG_GOF_2(Gamma, std::gamma_distribution, 1.5, 1);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 1, 1);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 1, 0.5);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 1, 1.5);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 0.5, 1);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 1.5, 1);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 0.5, 0.5);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 0.9, 0.9);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 1.5, 1.5);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 1.5, 0.5);
    VSMC_RNG_GOF_2(Beta, boost::random::beta_distribution, 0.5, 1.5);
    VSMC_RNG_GOF_1(ChiSquared, std::chi_squared_distribution, 0.2);
    VSMC_RNG_GOF_1(ChiSquared, std::chi_squared_distribution, 1);
    VSMC_RNG_GOF_1(ChiSquared, std::chi_squared_distribution, 1.5);
    VSMC_RNG_GOF_1(ChiSquared, std::chi_squared_distribution, 2);
    VSMC_RNG_GOF_1(StudentT, std::student_t_distribution, 0.2);
    VSMC_RNG_GOF_1(StudentT, std::student_t_distribution, 1);
    VSMC_RNG_GOF_1(StudentT, std::student_t_distribution, 1.5);
    VSMC_RNG_GOF_1(StudentT, std::student_t_distribution, 2);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 0.2, 0.2);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 0.2, 1);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 0.2, 1.5);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 0.2, 3);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 1, 0.2);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 1, 1);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 1, 1.5);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 1, 3);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 3, 0.2);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 3, 1);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 3, 1.5);
    VSMC_RNG_GOF_2(FisherF, std::fisher_f_distribution, 3, 3);
    // VSMC_RNG_GOF_1(Stable, vsmc::StableDistribution, 1);
    // VSMC_RNG_GOF_1(Stable, vsmc::StableDistribution, 0.1);
    // VSMC_RNG_GOF_1(Stable, vsmc::StableDistribution, 2);

    rng_gof_output(names, pval);

    return 0;
}
