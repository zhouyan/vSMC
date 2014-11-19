//============================================================================
// vSMC/vSMCExample/pet/include/pet_proposal.hpp
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

#ifndef VSMC_EXAMPLE_PET_PROPOSAL_HPP
#define VSMC_EXAMPLE_PET_PROPOSAL_HPP

class pet_proposal
{
    public :

    typedef const vsmc::Sampler<pet_state> * value_type;

    pet_proposal (value_type) {}

    void proposal_iter (std::size_t, vsmc::Particle<pet_state> &particle) const
    {
        for (vsmc::Particle<pet_state>::size_type i = 0;
                i != particle.size(); ++i) {
            double alpha = particle.value().state(i, 0).alpha();
            double coeff;
            const std::size_t cn = particle.value().state(i, 0).comp_num();
            for (std::size_t d = 0; d != cn; ++d) {
                coeff = 30 / (1 + 30 * alpha);
                particle.value().state(i, 0).phi_sd(d) =
                    particle.value().phi_sd(d) * coeff;
                coeff = 5 / (1 + 5 * alpha);
                particle.value().state(i, 0).theta_sd(d) =
                    particle.value().theta_sd(d) * coeff;
            }
            coeff = 80 / (1 + 20 * alpha);
            particle.value().state(i, 0).lambda_sd() =
                particle.value().lambda_sd() * coeff;
            particle.value().state(i, 0).nu_sd() =
                particle.value().nu_sd() * coeff;
        }
    }

    void proposal_init (vsmc::Particle<pet_state> &particle) const
    {proposal_iter(0, particle);}
};

class pet_proposal_adaptive
{
    public :

    typedef const vsmc::Sampler<pet_state> * value_type;

    pet_proposal_adaptive (value_type sampler) : sampler_(sampler) {}

    void proposal_iter (std::size_t, vsmc::Particle<pet_state> &particle) const
    {
        using std::sqrt;

        std::size_t cn = particle.value().state(0,0).comp_num();
        const vsmc::Monitor<pet_state> &monitor =
            sampler_->monitor("pet_moments");
        double lambda_sd = 2.38 * sqrt(monitor.record(2 + 2 * cn) -
                monitor.record(0) * monitor.record(0));
        double nu_sd = 2.38 * sqrt(monitor.record(3 + 2 * cn) -
                monitor.record(1) * monitor.record(1));
        std::vector<double> phi_sd(cn);
        std::vector<double> theta_sd(cn);
        const double coeff = 2.38 / sqrt(static_cast<double>(cn));
        for (std::size_t d = 0; d != cn; ++d) {
            phi_sd[d] = coeff * sqrt(monitor.record(d + 4 + 2 * cn) -
                    monitor.record(d + 2) * monitor.record(d + 2));
            theta_sd[d] = coeff / 2 * sqrt(monitor.record(d + 4 + 3 * cn) -
                    monitor.record(d + 2 + cn) * monitor.record(d + 2 + cn));
        }

        for (vsmc::Particle<pet_state>::size_type i = 0;
                i != particle.size(); ++i) {
            particle.value().state(i, 0).lambda_sd() = lambda_sd;
            particle.value().state(i, 0).nu_sd() = nu_sd;
            for (std::size_t d = 0; d != cn; ++d) {
                particle.value().state(i, 0).phi_sd(d) = phi_sd[d];
                particle.value().state(i, 0).theta_sd(d) = theta_sd[d];
            }
        }
    }

    private :

    const vsmc::Sampler<pet_state> *const sampler_;
};

#endif // VSMC_EXAMPLE_PET_PROPOSAL_HPP
