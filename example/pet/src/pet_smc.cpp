//============================================================================
// vSMC/example/pet/src/pet_smc.cpp
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

#include "pet_@smp@.hpp"
#include "smc.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "options_smc.hpp"
#include "pet_options.hpp"
#include "options_process.hpp"
#include "pet_data.hpp"

    //////////////////////////////////////////////////////////////////////

    vsmc::Sampler<pet_state> sampler(ParticleNum, vsmc::Stratified, Threshold);
    sampler
        .init(pet_init())
        .mcmc(pet_move_phi(), true)
        .mcmc(pet_move_theta(), true)
        .mcmc(pet_move_lambda(), true)
        .mcmc(pet_move_nu(), true)
        .monitor("vd", 2, pet_vd())
        .monitor("phi", (SM > CM ? SM : CM), pet_phi())
        .monitor("theta", (SM > CM ? SM : CM), pet_theta())
        .path_sampling(smc_path<pet_state>());
    if (ProposalScale == 2) {
        sampler.monitor("pet_moments", 2 * (2 + 2 * (SM > CM ? SM : CM)),
                pet_moments());
    }

    sampler.initialize(&info);
    info.read_time  = false;
    info.read_conv  = false;
    info.read_prior = false;
    info.read_sd    = false;
    info.read_model = false;

    //////////////////////////////////////////////////////////////////////

    std::string zconst_file_name("smc." + Suffix);
    std::ofstream zconst_file;
    zconst_file.open(zconst_file_name.c_str());
    zconst_file << "Schedule Config ";
    print_zconst_header(zconst_file, SM);
    print_zconst_header(zconst_file, CM);
    zconst_file << std::endl;
    if (ProposalScale == 2) {
        typedef pet_proposal_adaptive sd;
        for (std::size_t i = DataStart; i != DataStop; ++i) {
            info_d.data_value = &Data[i * DataNum];
            sampler.initialize(&info);
            smc_do<pet_state, sd>(Config, sampler, zconst_file);
        }
    } else {
        typedef pet_proposal sd;
        for (std::size_t i = DataStart; i != DataStop; ++i) {
            info_d.data_value = &Data[i * DataNum];
            sampler.initialize(&info);
            smc_do<pet_state, sd>(Config, sampler, zconst_file);
        }
    }
    zconst_file.close();
    zconst_file.clear();

    return 0;
}
