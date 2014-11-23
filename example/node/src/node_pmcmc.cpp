//============================================================================
// vSMC/example/node/src/node_pmcmc.cpp
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

#include "node_@smp@.hpp"
#include "pmcmc.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "options_pmcmc.hpp"
#include "node_options.hpp"
#include "options_process.hpp"

    //////////////////////////////////////////////////////////////////////

    vsmc::Sampler<node_state> sampler(ChainNum);
    sampler.init(node_init());
    sampler.mcmc(pmcmc_global<node_state>(), false);

    Parallel ?
        sampler.move(pmcmc_local_parallel<node_move>(), false):
        sampler.move(pmcmc_local_serial<node_state, node_move>(), false);

    data_info info(DataNum, Resolution, DataFile.c_str());
    sampler.initialize(&info);

    //////////////////////////////////////////////////////////////////////

    std::string zconst_file_name("pmcmc." + Suffix);
    std::ofstream zconst_file;
    zconst_file.open(zconst_file_name.c_str());
    zconst_file << "Schedule Chains ";
    if (SM) zconst_file << "Accept." << SM << " Path." << SM << ' ';
    if (CM) zconst_file << "Accept." << CM << " Path." << CM << ' ';
    zconst_file << std::endl;
    pmcmc_do<node_state, node_init, node_proposal>(
            Config, sampler, zconst_file);
    zconst_file.close();
    zconst_file.clear();

    return 0;
}
