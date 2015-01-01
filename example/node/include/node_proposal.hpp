//============================================================================
// vSMC/example/node/include/node_proposal.hpp
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

#ifndef VSMC_EXAMPLE_NODE_PROPOSAL_HPP
#define VSMC_EXAMPLE_NODE_PROPOSAL_HPP

class node_proposal
{
    public :

    typedef const vsmc::Sampler<node_state> * value_type;

    node_proposal () {}
    node_proposal (value_type) {}

    void proposal_iter (std::size_t,
            vsmc::Particle<node_state> &particle) const
    {
        double alpha = particle.value().state(0, 0).alpha();
        std::size_t cn = particle.value().state(0, 0).comp_num();
        double a0_sd, a1_sd, a2_sd, k_sd;
        alpha2sd(alpha, a0_sd, a1_sd, a2_sd, k_sd);
        for (vsmc::Particle<node_state>::size_type i = 0;
                i != particle.size(); ++i) {
            particle.value().state(i, 0).a0_sd() = a0_sd;
            particle.value().state(i, 0).a1_sd() = a1_sd;
            particle.value().state(i, 0).a2_sd() = a2_sd;
            for (std::size_t d = 0; d != cn - 1; ++d)
                particle.value().state(i, 0).k_sd(d) = k_sd;
        }
    }

    void proposal_init (vsmc::Particle<node_state> &particle) const
    {
        for (vsmc::Particle<node_state>::size_type i = 0;
                i != particle.size(); ++i) {
            double alpha = particle.value().state(i, 0).alpha();
            std::size_t cn = particle.value().state(i, 0).comp_num();
            double a0_sd, a1_sd, a2_sd, k_sd;
            alpha2sd(alpha, a0_sd, a1_sd, a2_sd, k_sd);
            particle.value().state(i, 0).a0_sd() = a0_sd;
            particle.value().state(i, 0).a1_sd() = a1_sd;
            particle.value().state(i, 0).a2_sd() = a2_sd;
            for (std::size_t d = 0; d != cn - 1; ++d)
                particle.value().state(i, 0).k_sd(d) = k_sd;
        }
    }

    private :

    void alpha2sd (double alpha,
            double &a0_sd, double &a1_sd, double &a2_sd, double &k_sd) const
    {
        using std::sqrt;

        a0_sd = 5 / (1 + 100 * alpha);
        a1_sd = 5 / (1 + 100 * alpha);
        a2_sd = 5 / (1 + 100 * alpha);
        k_sd  = 5 / (1 + 100 * alpha);
    }
};

class node_proposal_adaptive
{
    public :

    typedef const vsmc::Sampler<node_state> * value_type;

    node_proposal_adaptive (value_type sampler) : sampler_(sampler) {}

    void proposal_iter (std::size_t,
            vsmc::Particle<node_state> &particle) const
    {
        using std::sqrt;

        std::size_t cn = particle.value().state(0,0).comp_num();
        const vsmc::Monitor<node_state> &monitor =
            sampler_->monitor("node_moments");

        const double coeff = 2.38;
        double a0_sd = coeff * sqrt(monitor.record(2 + cn) -
                monitor.record(0) * monitor.record(0));
        double a1_sd = coeff * sqrt(monitor.record(3 + cn) -
                monitor.record(1) * monitor.record(1));
        double a2_sd = coeff * sqrt(monitor.record(4 + cn) -
                monitor.record(2) * monitor.record(2));
        std::vector<double> k_sd(cn - 1);
        for (std::size_t d = 0; d != cn - 1; ++d) {
            k_sd[d] = coeff * sqrt(monitor.record(5 + cn + d) -
                    monitor.record(3 + d) * monitor.record(3 + d));
        }

        for (vsmc::Particle<node_state>::size_type i = 0;
                i != particle.size(); ++i) {
            particle.value().state(i, 0).a0_sd() = a0_sd;
            particle.value().state(i, 0).a1_sd() = a1_sd;
            particle.value().state(i, 0).a2_sd() = a2_sd;
            for (std::size_t d = 0; d != cn - 1; ++d)
                particle.value().state(i, 0).k_sd(d) = k_sd[d];
        }
    }

    private :

    const vsmc::Sampler<node_state> *sampler_;
};

#endif // VSMC_EXAMPLE_NODE_PROPOSAL_HPP
