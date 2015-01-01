//============================================================================
// vSMC/example/gmm/src/gmm_smc.cpp
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

#include "gmm_@smp@.hpp"
#include "smc.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "options_smc.hpp"
#include "gmm_options.hpp"
#include "options_process.hpp"

    //////////////////////////////////////////////////////////////////////

    vsmc::Sampler<gmm_state> sampler(ParticleNum, vsmc::Stratified, Threshold);
    sampler
        .init(gmm_init())
        .mcmc(gmm_move_mu(), true)
        .mcmc(gmm_move_lambda(), true)
        .mcmc(gmm_move_weight(), true)
        .path_sampling(smc_path<gmm_state>());
    if (ProposalScale == 2) {
        sampler
            .monitor("rm.mu", 2, gmm_rm_mu())
            .monitor("rm.lambda", 2, gmm_rm_lambda())
            .monitor("rm.weight", 2, gmm_rm_weight());
    }

    data_info info(DataNum, DataFile.c_str());
    sampler.initialize(&info);

    //////////////////////////////////////////////////////////////////////

    std::ofstream zconst_file;
    std::string zconst_file_name("smc." + Suffix);
    zconst_file.open(zconst_file_name.c_str());
    zconst_file << "Schedule Config ";
    print_zconst_header(zconst_file, SM);
    print_zconst_header(zconst_file, CM);
    zconst_file << std::endl;
    if (ProposalScale == 2) {
        typedef gmm_proposal_adaptive sd;
        smc_do<gmm_state, sd>(Config, sampler, zconst_file);
    } else {
        typedef gmm_proposal sd;
        smc_do<gmm_state, sd>(Config, sampler, zconst_file);
    }
    zconst_file.close();
    zconst_file.clear();

    return 0;
}
