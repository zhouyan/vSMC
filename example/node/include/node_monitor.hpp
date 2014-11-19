//============================================================================
// vSMC/vSMCExample/node/include/node_monitor.hpp
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

#ifndef VSMC_EXAMPLE_NODE_MONITOR_HPP
#define VSMC_EXAMPLE_NODE_MONITOR_HPP

class node_moments : public BASE_MONITOR<node_state, node_moments>
{
    public :

    void monitor_state (std::size_t, std::size_t dim,
            vsmc::ConstSingleParticle<node_state> csp, double *res)
    {
        using std::log;

        for (std::size_t i = 0; i != dim; ++i)
            res[i] = 0;
        const node_param &state = csp.state(0);
        assert(dim >= (2 + state.comp_num()) * 2);
        std::size_t offset = 0;
        res[offset++] = log(state.a0());
        res[offset++] = log(state.a1());
        res[offset++] = log(state.a2());
        for (std::size_t d = 0; d != state.comp_num() - 1; ++d)
            res[offset++] = log(state.k(d));
        res[offset++] = res[0] * res[0];
        res[offset++] = res[1] * res[1];
        res[offset++] = res[2] * res[2];
        for (std::size_t d = 0; d != state.comp_num() - 1; ++d)
            res[offset++] = res[3 + d] * res[3 + d];
    }
};

#endif // VSMC_EXAMPLE_NODE_MONITOR_HPP
