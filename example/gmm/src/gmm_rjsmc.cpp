//============================================================================
// vSMC/example/gmm/src/gmm_rjsmc.cpp
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
#include "smc.hpp"

template <typename MoveType, typename T>
inline void do_rjsmc (
        vsmc::Sampler<T> &sampler, const std::string &,
        std::ostream &zconst_file, const std::string &,
        typename MoveType::alpha_type::value_type alpha_config,
        typename MoveType::proposal_type::value_type proposal_config,
        std::size_t repeat)
{
    sampler.move(MoveType(alpha_config, proposal_config), false);
    std::vector<double> cn(MaxCompNum);
    std::vector<double> w(sampler.size());
    for (std::size_t i = 0; i != cn.size(); ++i)
        cn[i] = 0;

    for (std::size_t i = 0; i != repeat; ++i) {
        if (repeat > 1)
            std::cout << "Run: " << i + 1 << std::endl;
        sampler.initialize();
        while (sampler.particle().value().state(0, 0).alpha() < 1)
            sampler.iterate();
        sampler.particle().weight_set().read_weight(&w[0]);
        for (typename vsmc::Sampler<T>::size_type j = 0;
                j != sampler.size(); ++j)
            cn[sampler.particle().value().state(j, 0).comp_num() - 1] += w[j];
        for (std::size_t j = 0; j != cn.size(); ++j)
            zconst_file << cn[j] << ' ';
        zconst_file << std::endl;
    }
}

int main (int argc, char **argv)
{
#include "options_main.hpp"
#include "options_smc.hpp"
#include "gmm_options.hpp"
#include "options_process.hpp"

    //////////////////////////////////////////////////////////////////////

    vsmc::Sampler<gmm_state> sampler(ParticleNum, vsmc::Stratified, Threshold);
    sampler.particle().value().ordered() = true;
    sampler
        .init(gmm_init_rjsmc())
        .mcmc(gmm_move_mu(), true)
        .mcmc(gmm_move_lambda(), true)
        .mcmc(gmm_move_weight(), true)
        .mcmc(gmm_rj_sc<gmm_logodds_flat>(), true)
        .mcmc(gmm_rj_bd<gmm_logodds_flat>(), true);

    data_info info(DataNum, DataFile.c_str());
    sampler.initialize(&info);

    //////////////////////////////////////////////////////////////////////

    std::string zconst_file_name("rjsmc." + Suffix);
    std::ofstream zconst_file;
    zconst_file.open(zconst_file_name.c_str());

    //////////////////////////////////////////////////////////////////////

    typedef gmm_state gmm;
    typedef gmm_proposal sd;
    typedef move_smc<gmm, alpha_ess<gmm_state, ess01<gmm_state> >, sd>  ess;
    typedef move_smc<gmm, alpha_ess<gmm_state, cess01<gmm_state> >, sd>cess;
    typedef move_smc<gmm, alpha_linear   <gmm>,    sd> linear;
    typedef move_smc<gmm, alpha_prior    <gmm, 2>, sd> prior2;
    typedef move_smc<gmm, alpha_prior    <gmm, 5>, sd> prior5;
    typedef move_smc<gmm, alpha_posterior<gmm, 2>, sd> posterior2;
    typedef move_smc<gmm, alpha_posterior<gmm, 5>, sd> posterior5;

    if (Config.count("ess"))
        for (std::size_t i = 0; i != ESSDrop.size(); ++i)
            do_rjsmc<ess>(sampler, Suffix, zconst_file, "ESS",
                    ESSDrop[i], 0, Repeat);

    if (Config.count("cess"))
        for (std::size_t i = 0; i != CESSDrop.size(); ++i)
            do_rjsmc<cess>(sampler, Suffix, zconst_file, "CESS",
                    CESSDrop[i], 0, Repeat);

    if (Config.count("linear"))
        for (std::size_t i = 0; i != LinearIterNum.size(); ++i)
            do_rjsmc<linear>(sampler, Suffix, zconst_file, "Linear",
                    LinearIterNum[i], 0, Repeat);

    if (Config.count("prior2"))
        for (std::size_t i = 0; i != Prior2IterNum.size(); ++i)
            do_rjsmc<prior2>(sampler, Suffix, zconst_file, "Prior2",
                    Prior2IterNum[i], 0, Repeat);

    if (Config.count("prior5"))
        for (std::size_t i = 0; i != Prior5IterNum.size(); ++i)
            do_rjsmc<prior5>(sampler, Suffix, zconst_file, "Prior5",
                    Prior5IterNum[i], 0, Repeat);

    if (Config.count("posterior2"))
        for (std::size_t i = 0; i != Posterior2IterNum.size(); ++i)
            do_rjsmc<posterior2>(sampler, Suffix, zconst_file, "Posterior2",
                    Posterior2IterNum[i], 0, Repeat);

    if (Config.count("posterior5"))
        for (std::size_t i = 0; i != Posterior5IterNum.size(); ++i)
            do_rjsmc<posterior5>(sampler, Suffix, zconst_file, "Posterior5",
                    Posterior5IterNum[i], 0, Repeat);

    //////////////////////////////////////////////////////////////////////

    zconst_file.close();
    zconst_file.clear();

    return 0;

}
