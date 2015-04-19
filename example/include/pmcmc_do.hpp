//============================================================================
// vSMC/example/include/pmcmc_do.hpp
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

#ifndef VSMC_EXAMPLE_PMCMC_DO_HPP
#define VSMC_EXAMPLE_PMCMC_DO_HPP

template <typename T>
inline double pmcmc_do(vsmc::Sampler<T> &sampler, std::size_t model_num)
{
    const std::size_t nchain = static_cast<std::size_t>(sampler.size());
    std::vector<double> log_likelihood(nchain);

    sampler.particle().value().comp_num(model_num);
    sampler.initialize();
    sampler.iterate(BurninNum);
    for (std::size_t k = 0; k != nchain; ++k)
        log_likelihood[k] = 0;
    for (std::size_t j = 0; j != IterNum; ++j) {
        sampler.iterate();
        for (std::size_t k = 0; k != nchain; ++k) {
            log_likelihood[k] +=
                sampler.particle().value().state(k, 0).log_likelihood();
        }
    }
    for (std::size_t k = 0; k != nchain; ++k)
        log_likelihood[k] /= IterNum;
    double ps = sampler.particle().value().log_likelihood_const();
    for (std::size_t k = 1; k != nchain; ++k) {
        ps += sampler.particle().value().state(k, 0).alpha_inc() * 0.5 *
              (log_likelihood[k - 1] + log_likelihood[k]);
    }

    return ps;
}

template <typename InitType, typename T>
inline void pmcmc_do(vsmc::Sampler<T> &sampler,
                     std::ostream &zconst_file,
                     const std::string &schedule_name)
{
    sampler.init(InitType());

    for (std::size_t i = 0; i != Repeat; ++i) {
        if (Repeat > 1)
            std::cout << "Run: " << i + 1 << std::endl;
        zconst_file << schedule_name << ' ';
        zconst_file << sampler.size() << ' ';
        zconst_file << pmcmc_do(sampler, SM) << ' ';
        zconst_file << pmcmc_do(sampler, CM) << ' ';
        zconst_file << std::endl;
    }
}

template <typename T, typename Init, typename SD, typename Config>
inline void pmcmc_do(const Config &config,
                     vsmc::Sampler<T> &sampler,
                     std::ostream &zconst_file)
{
    typedef Init init;
    typedef SD sd;
    typedef init_pmcmc<T, alpha_linear<T>, sd, init> linear;
    typedef init_pmcmc<T, alpha_prior<T, 2>, sd, init> prior2;
    typedef init_pmcmc<T, alpha_prior<T, 5>, sd, init> prior5;
    typedef init_pmcmc<T, alpha_posterior<T, 2>, sd, init> posterior2;
    typedef init_pmcmc<T, alpha_posterior<T, 5>, sd, init> posterior5;

    if (config.count("linear"))
        pmcmc_do<linear>(sampler, zconst_file, "Linear");

    if (config.count("prior2"))
        pmcmc_do<prior2>(sampler, zconst_file, "Prior2");

    if (config.count("prior5"))
        pmcmc_do<prior5>(sampler, zconst_file, "Prior5");

    if (config.count("posterior2"))
        pmcmc_do<posterior2>(sampler, zconst_file, "Posterior2");

    if (config.count("posterior5"))
        pmcmc_do<posterior5>(sampler, zconst_file, "Posterior5");
}

#endif  // VSMC_EXAMPLE_PMCMC_DO_HPP
