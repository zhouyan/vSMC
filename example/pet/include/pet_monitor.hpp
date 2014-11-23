//============================================================================
// vSMC/example/pet/include/pet_monitor.hpp
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

#ifndef VSMC_EXAMPLE_PET_MONITOR_HPP
#define VSMC_EXAMPLE_PET_MONITOR_HPP

class pet_phi : public BASE_MONITOR<pet_state, pet_phi>
{
    public :

    void monitor_state (std::size_t, std::size_t,
            vsmc::ConstSingleParticle<pet_state> csp, double *res)
    {
        for (std::size_t d = 0; d != csp.state(0).comp_num(); ++d)
            res[d] = csp.state(0).phi(d);
    }
};

class pet_theta : public BASE_MONITOR<pet_state, pet_theta>
{
    public :

    void monitor_state (std::size_t, std::size_t,
            vsmc::ConstSingleParticle<pet_state> csp, double *res)
    {
        for (std::size_t d = 0; d != csp.state(0).comp_num(); ++d)
            res[d] = csp.state(0).theta(d);
    }
};

class pet_vd : public BASE_MONITOR<pet_state, pet_vd>
{
    public :

    void monitor_state (std::size_t, std::size_t,
            vsmc::ConstSingleParticle<pet_state> csp, double *res)
    {
        double vd = 0;
        double decay = csp.particle().value().decay();
        for (std::size_t d = 0; d != csp.state(0).comp_num(); ++d)
            vd += csp.state(0).phi(d) / (csp.state(0).theta(d) - decay);
        res[0] = vd;
        res[1] = vd * vd;
    }
};

class pet_moments : public BASE_MONITOR<pet_state, pet_moments>
{
    public :

    void monitor_state (std::size_t, std::size_t dim,
            vsmc::ConstSingleParticle<pet_state> csp, double *res)
    {
        using std::log;

        for (std::size_t i = 0; i != dim; ++i)
            res[i] = 0;
        const pet_param &state = csp.state(0);
        assert(dim >= (2 + 2 * state.comp_num()) * 2);
        std::size_t offset = 0;
        res[offset++] = log(state.lambda());
        res[offset++] = log(state.nu());
        for (std::size_t d = 0; d != state.comp_num(); ++d)
            res[offset++] = state.phi(d);
        for (std::size_t d = 0; d != state.comp_num(); ++d)
            res[offset++] = state.theta(d);
        res[offset++] = res[0] * res[0];
        res[offset++] = res[1] * res[1];
        for (std::size_t d = 0; d != state.comp_num(); ++d)
            res[offset++] = state.phi(d) * state.phi(d);
        for (std::size_t d = 0; d != state.comp_num(); ++d)
            res[offset++] = state.theta(d) * state.theta(d);
    }
};

#endif // VSMC_EXAMPLE_PET_MONITOR_HPP
