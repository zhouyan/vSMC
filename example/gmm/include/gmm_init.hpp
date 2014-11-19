//============================================================================
// vSMC/vSMCExample/gmm/include/gmm_init.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_EXAMPLE_GMM_INIT_HPP
#define VSMC_EXAMPLE_GMM_INIT_HPP

class gmm_init : public BASE_INIT<gmm_state, gmm_init>
{
    public :

    void pre_processor (vsmc::Particle<gmm_state> &particle)
    {
        particle.value().alpha(0);
        particle.value().alpha_inc(0);
        particle.weight_set().set_equal_weight();
    }

    void initialize_param (vsmc::Particle<gmm_state> &particle, void *info)
    {
        if (particle.value().state(0, 0).comp_num() == 0)
            particle.value().comp_num(InitCompNum);
        if (info)
            particle.value().read_data(static_cast<const data_info *>(info));
    }

    std::size_t initialize_state (vsmc::SingleParticle<gmm_state> sp)
    {
        double mu0    = sp.particle().value().mu0();
        double sd0    = sp.particle().value().sd0();
        double shape0 = sp.particle().value().shape0();
        double scale0 = sp.particle().value().scale0();

        vsmc::cxx11::normal_distribution<> rmu(mu0, sd0);
        vsmc::cxx11::gamma_distribution<>  rlambda(shape0, scale0);
        vsmc::cxx11::gamma_distribution<>  rweight(1, 1);

        const std::size_t cn = sp.state(0).comp_num();
        double sum_weight = 0;
        for (std::size_t d = 0; d != cn; ++d) {
            sp.state(0).mu(d)     = rmu(sp.rng());
            sp.state(0).lambda(d) = rlambda(sp.rng());
            sp.state(0).weight(d) = rweight(sp.rng());
            sum_weight += sp.state(0).weight(d);
        }
        for (std::size_t d = 0; d != cn; ++d)
            sp.state(0).weight(d) /= sum_weight;
        if (sp.particle().value().ordered())
            sp.state(0).sort_mu();
        sp.particle().value().log_target(sp.state(0));

        return 0;
    }
};

class gmm_init_pair : public gmm_init
{
    public :

    void pre_processor (vsmc::Particle<gmm_state> &particle)
    {
        gmm_init::pre_processor(particle);
        particle.value().beta(0);
        particle.value().beta_inc(0);
    }
};

class gmm_init_rjsmc : public gmm_init
{
    public :

    gmm_init_rjsmc (
            std::size_t min_comp = MinCompNum,
            std::size_t max_comp = MaxCompNum) :
        min_comp_(min_comp), max_comp_(max_comp) {}

    std::size_t initialize_state (vsmc::SingleParticle<gmm_state> sp)
    {
        vsmc::cxx11::uniform_int_distribution<> rcn(
                static_cast<int>(min_comp_), static_cast<int>(max_comp_));
        sp.state(0).comp_num(static_cast<std::size_t>(rcn(sp.rng())));
        return gmm_init::initialize_state(sp);
    }

    private :

    std::size_t min_comp_;
    std::size_t max_comp_;
};

class gmm_init_rjmcmc : public gmm_init_rjsmc
{
    public :

    gmm_init_rjmcmc (
            std::size_t min_comp = MinCompNum,
            std::size_t max_comp = MaxCompNum) :
        gmm_init_rjsmc(min_comp, max_comp) {}

    void pre_processor (vsmc::Particle<gmm_state> &particle)
    {
        particle.value().alpha(1);
        particle.value().alpha_inc(0);
        particle.weight_set().set_equal_weight();
        vsmc::Particle<gmm_state>::size_type N = particle.size();
        for (vsmc::Particle<gmm_state>::size_type i = 0; i != N; ++i) {
            particle.value().state(i, 0).mu_sd()     = 0.15;
            particle.value().state(i, 0).lambda_sd() = 0.15;
            particle.value().state(i, 0).weight_sd() = 0.2;
        }
    }
};

#endif // VSMC_EXAMPLE_GMM_INIT_HPP
