//============================================================================
// vSMC/example/gmm/src/gmm_rjmcmc.cpp
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

#include "gmm_@smp@.hpp"
#include "gmm_move_rj.hpp"

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "gmm_options.hpp"
    std::size_t BurninNum;
    std::size_t IterNum;
    Config
        .add("burnin_num", "Number of burin iterations",   &BurninNum, 10000)
        .add("iter_num",   "Number of recored iterations", &IterNum,   10000);
#include "options_process.hpp"

    //////////////////////////////////////////////////////////////////////

    vsmc::Sampler<gmm_state> sampler(Repeat);
    sampler.particle().value().ordered() = true;
    sampler
        .init(gmm_init_rjmcmc())
        .mcmc(gmm_move_mu(), true)
        .mcmc(gmm_move_lambda(), true)
        .mcmc(gmm_move_weight(), true)
        .mcmc(gmm_rj_sc<gmm_logodds_flat>(), true)
        .mcmc(gmm_rj_bd<gmm_logodds_flat>(), true);

    data_info info(DataNum, DataFile.c_str());
    sampler.initialize(&info);

    for (std::size_t d = 0; d != BurninNum; ++d) {
        if (!(d % 1000)) std::cout << "Burnin: " << d << std::endl;
        sampler.iterate();
    }

    std::vector<std::vector<std::size_t> > cn(Repeat);
    for (std::size_t i = 0; i != Repeat; ++i)
        cn[i].resize(MaxCompNum);
    for (std::size_t d = 0; d != IterNum; ++d) {
        if (!(d % 1000)) std::cout << "Iter: " << d << std::endl;
        sampler.iterate();
        for (std::size_t r = 0; r != Repeat; ++r)
            ++cn[r][sampler.particle().value().state(r,0).comp_num() - 1];
    }

    //////////////////////////////////////////////////////////////////////

    std::string zconst_file_name("rjmcmc." + Suffix);
    std::ofstream zconst_file;
    zconst_file.open(zconst_file_name.c_str());
    for (std::size_t r = 0; r != Repeat; ++r) {
        for (std::size_t d = 0; d != MaxCompNum; ++d)
            zconst_file << cn[r][d] / static_cast<double>(IterNum) << ' ';
        zconst_file << '\n';
    }
    zconst_file.close();
    zconst_file.clear();

    //////////////////////////////////////////////////////////////////////

    return 0;
}
