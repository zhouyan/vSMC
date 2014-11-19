//============================================================================
// vSMC/vSMCExample/node/include/node_init.hpp
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

#ifndef VSMC_EXAMPLE_NODE_INIT_HPP
#define VSMC_EXAMPLE_NODE_INIT_HPP

class node_init : public BASE_INIT<node_state, node_init>
{
    public :

    void pre_processor (vsmc::Particle<node_state> &particle)
    {
        particle.value().alpha(0);
        particle.value().alpha_inc(0);
        particle.weight_set().set_equal_weight();
    }

    void initialize_param (vsmc::Particle<node_state> &particle, void *info)
    {
        if (particle.value().state(0, 0).comp_num() == 0)
            particle.value().comp_num(InitCompNum);
        if (info)
            particle.value().read_data(static_cast<const data_info *>(info));
    }

    std::size_t initialize_state (vsmc::SingleParticle<node_state> sp)
    {
        double shape0 = sp.particle().value().shape0();
        double scale0 = sp.particle().value().scale0();

        vsmc::cxx11::gamma_distribution<> rgamma(shape0, scale0);

        const std::size_t cn = sp.state(0).comp_num();
        sp.state(0).a0() = rgamma(sp.rng());
        sp.state(0).a1() = rgamma(sp.rng());
        sp.state(0).a2() = rgamma(sp.rng());
        for (std::size_t d = 0; d != cn - 1; ++d)
            sp.state(0).k(d) = rgamma(sp.rng());
        sp.particle().value().log_target(sp.state(0));

        return 0;
    }
};

#endif // VSMC_EXAMPLE_NODE_INIT_HPP
