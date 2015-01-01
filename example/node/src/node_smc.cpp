//============================================================================
// vSMC/example/node/src/node_smc.cpp
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

#include "node_@smp@.hpp"
#include "smc.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "options_smc.hpp"
#include "node_options.hpp"
#include "options_process.hpp"

    //////////////////////////////////////////////////////////////////////

    vsmc::Sampler<node_state> sampler(ParticleNum,
            vsmc::Stratified, Threshold);
    sampler
        .init(node_init())
        .mcmc(node_move_a0(), true)
        .mcmc(node_move_a1(), true)
        .mcmc(node_move_a2(), true)
        .path_sampling(smc_path<node_state>());
    for (std::size_t i = 0; i != (SM > CM ? SM : CM) - 1; ++i)
        sampler.mcmc(node_move_k(i), true);
    if (ProposalScale == 2) {
        sampler.monitor("node_moments", 2 * (2 + (SM > CM ? SM : CM)),
                node_moments());
    }

    data_info info(DataNum, Resolution, DataFile.c_str());
    sampler.initialize(&info);

    //////////////////////////////////////////////////////////////////////

    std::string zconst_file_name("smc." + Suffix);
    std::ofstream zconst_file;
    zconst_file.open(zconst_file_name.c_str());
    zconst_file << "Schedule Config ";
    print_zconst_header(zconst_file, SM);
    print_zconst_header(zconst_file, CM);
    zconst_file << std::endl;
    if (ProposalScale == 2) {
        typedef node_proposal_adaptive sd;
        smc_do<node_state, sd>(Config, sampler, zconst_file);
    } else {
        typedef node_proposal sd;
        smc_do<node_state, sd>(Config, sampler, zconst_file);
    }
    zconst_file.close();
    zconst_file.clear();

    return 0;
}
