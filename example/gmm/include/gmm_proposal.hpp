//============================================================================
// vSMC/example/gmm/include/gmm_proposal.hpp
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

#ifndef VSMC_EXAMPLE_GMM_PROPOSAL_HPP
#define VSMC_EXAMPLE_GMM_PROPOSAL_HPP

class gmm_proposal
{
    public:
    typedef const vsmc::Sampler<gmm_state> *value_type;

    gmm_proposal() {}
    gmm_proposal(value_type) {}

    void proposal_iter(std::size_t, vsmc::Particle<gmm_state> &particle) const
    {
        double alpha = particle.value().state(0, 0).alpha();
        double weight_sd, mu_sd, lambda_sd;
        alpha2sd(alpha, mu_sd, lambda_sd, weight_sd);
        for (vsmc::Particle<gmm_state>::size_type i = 0; i != particle.size();
             ++i) {
            particle.value().state(i, 0).mu_sd() = mu_sd;
            particle.value().state(i, 0).lambda_sd() = lambda_sd;
            particle.value().state(i, 0).weight_sd() = weight_sd;
        }
    }

    void proposal_init(vsmc::Particle<gmm_state> &particle) const
    {
        for (vsmc::Particle<gmm_state>::size_type i = 0; i != particle.size();
             ++i) {
            alpha2sd(particle.value().state(i, 0).alpha(),
                     particle.value().state(i, 0).mu_sd(),
                     particle.value().state(i, 0).lambda_sd(),
                     particle.value().state(i, 0).weight_sd());
        }
    }

    private:
    void alpha2sd(double alpha,
                  double &mu_sd,
                  double &lambda_sd,
                  double &weight_sd) const
    {
        using std::sqrt;

        alpha = alpha < 0.05 ? 0.05 : alpha;
        mu_sd = 0.15 / alpha;
        lambda_sd = (1 + sqrt(1.0 / alpha)) * 0.15;
        weight_sd = (1 + sqrt(1.0 / alpha)) * 0.2;
    }
};

class gmm_proposal_adaptive
{
    public:
    typedef const vsmc::Sampler<gmm_state> *value_type;

    gmm_proposal_adaptive(value_type sampler) : sampler_(sampler) {}

    void proposal_iter(std::size_t, vsmc::Particle<gmm_state> &particle) const
    {
        using std::sqrt;

        double coeff = 2.38 / sqrt(static_cast<double>(
                                  particle.value().state(0, 0).comp_num()));
        double mu_sd = coeff * get_sd("rm.mu");
        double lambda_sd = coeff * get_sd("rm.lambda");
        double weight_sd = coeff * get_sd("rm.weight");
        for (vsmc::Particle<gmm_state>::size_type i = 0; i != particle.size();
             ++i) {
            particle.value().state(i, 0).mu_sd() = mu_sd;
            particle.value().state(i, 0).lambda_sd() = lambda_sd;
            particle.value().state(i, 0).weight_sd() = weight_sd;
        }
    }

    private:
    const vsmc::Sampler<gmm_state> *const sampler_;

    double get_sd(const std::string &name) const
    {
        using std::sqrt;

        const vsmc::Monitor<gmm_state> &monitor = sampler_->monitor(name);
        double mean = monitor.record(0);
        double r2mn = monitor.record(1);

        return sqrt(r2mn - mean * mean);
    }
};

#endif  // VSMC_EXAMPLE_GMM_PROPOSAL_HPP
