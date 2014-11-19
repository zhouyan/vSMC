//============================================================================
// vSMC/vSMCExample/gmm/src/gmm_smc_pair.cpp
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
#include "move_smc_pair.hpp"
#include "smc.hpp"

template <typename MoveType, typename RJMoveType, typename MoveQueueType,
    typename T>
inline void smc_pair_do (
        vsmc::Sampler<T> &sampler, const std::string &,
        std::ostream &zconst_file, const std::string &schedule_name,
        typename MoveType::alpha_type::value_type alpha_config,
        typename MoveType::proposal_type::value_type proposal_config,
        const MoveQueueType &move_queue, const MoveQueueType &rjmove_queue,
        std::size_t simple_model, std::size_t complex_model,
        std::size_t repeat)
{

    for (std::size_t i = 0; i != repeat; ++i) {
        if (repeat > 1)
            std::cout << "Run: " << i + 1 << std::endl;
        zconst_file << schedule_name << ' ';
        zconst_file << alpha_config << ' ';
        sampler.particle().value().comp_num(simple_model);
        sampler.initialize();
        sampler.move(MoveType(alpha_config, proposal_config), false);
        sampler.mcmc(move_queue.begin(), move_queue.end(), false);
        while (sampler.particle().value().state(0, 0).alpha() < 1)
            sampler.iterate();
        sampler.move(RJMoveType(simple_model, complex_model, alpha_config),
                false);
        sampler.mcmc(rjmove_queue.begin(), rjmove_queue.end(), false);
        while (sampler.particle().value().state(0, 0).beta() < 1)
            sampler.iterate();
        zconst_file << sampler.iter_size() - 1 << ' ';
        zconst_file << sampler.particle().value().zconst()
            + sampler.particle().value().log_prior_const(complex_model)
            - sampler.particle().value().log_prior_const(simple_model) << ' ';
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
    sampler.init(gmm_init_pair());
    std::vector<vsmc::Sampler<gmm_state>::move_type> move_queue;
    move_queue.push_back(gmm_move_mu());
    move_queue.push_back(gmm_move_lambda());
    move_queue.push_back(gmm_move_weight());
    std::vector<vsmc::Sampler<gmm_state>::move_type> rjmove_queue(move_queue);
    rjmove_queue.push_back(gmm_rj_sc<gmm_logodds_pair>(SM, CM));
    rjmove_queue.push_back(gmm_rj_bd<gmm_logodds_pair>(SM, CM));
    sampler.monitor("Comp.Num", 1, gmm_comp_num());

    data_info info(DataNum, DataFile.c_str());
    sampler.initialize(&info);

    //////////////////////////////////////////////////////////////////////

    std::string zconst_file_name("smc-pair." + Suffix);
    std::ofstream zconst_file;
    zconst_file.open(zconst_file_name.c_str());
    zconst_file << "Schedule Config Iteration SMCB PathB";
    zconst_file << std::endl;

    //////////////////////////////////////////////////////////////////////

    typedef gmm_state gmm;
    typedef gmm_proposal sd;

    typedef move_smc<gmm, alpha_ess<gmm_state, ess01<gmm_state> >,  sd> ess;
    typedef move_smc<gmm, alpha_ess<gmm_state, cess01<gmm_state> >, sd> cess;
    typedef move_smc<gmm, alpha_linear   <gmm>,    sd> linear;
    typedef move_smc<gmm, alpha_prior    <gmm, 2>, sd> prior2;
    typedef move_smc<gmm, alpha_prior    <gmm, 5>, sd> prior5;
    typedef move_smc<gmm, alpha_posterior<gmm, 2>, sd> posterior2;
    typedef move_smc<gmm, alpha_posterior<gmm, 5>, sd> posterior5;

    typedef move_smc_pair<gmm, alpha_ess<gmm_state, ess01<gmm_state> > >
        rj_ess;
    typedef move_smc_pair<gmm, alpha_ess<gmm_state, cess01<gmm_state> > >
        rj_cess;
    typedef move_smc_pair<gmm, alpha_linear   <gmm> >    rj_linear;
    typedef move_smc_pair<gmm, alpha_prior    <gmm, 2> > rj_prior2;
    typedef move_smc_pair<gmm, alpha_prior    <gmm, 5> > rj_prior5;
    typedef move_smc_pair<gmm, alpha_posterior<gmm, 2> > rj_posterior2;
    typedef move_smc_pair<gmm, alpha_posterior<gmm, 5> > rj_posterior5;

    if (Config.count("ess"))
        for (std::size_t i = 0; i != ESSDrop.size(); ++i)
            smc_pair_do<ess, rj_ess>(sampler, Suffix, zconst_file, "ESS",
                    ESSDrop[i], 0, move_queue, rjmove_queue, SM, CM, Repeat);

    if (Config.count("cess"))
        for (std::size_t i = 0; i != CESSDrop.size(); ++i)
            smc_pair_do<cess, rj_cess>(sampler, Suffix, zconst_file, "CESS",
                    CESSDrop[i], 0, move_queue, rjmove_queue, SM, CM, Repeat);

    if (Config.count("linear"))
        for (std::size_t i = 0; i != LinearIterNum.size(); ++i)
            smc_pair_do<linear, rj_linear>(sampler, Suffix, zconst_file,
                    "Linear", LinearIterNum[i], 0, move_queue, rjmove_queue,
                    SM, CM, Repeat);

    if (Config.count("prior2"))
        for (std::size_t i = 0; i != Prior2IterNum.size(); ++i)
            smc_pair_do<prior2, rj_prior2>(sampler, Suffix, zconst_file,
                    "Prior2", Prior2IterNum[i], 0, move_queue, rjmove_queue,
                    SM, CM, Repeat);

    if (Config.count("prior5"))
        for (std::size_t i = 0; i != Prior5IterNum.size(); ++i)
            smc_pair_do<prior5, rj_prior5>(sampler, Suffix, zconst_file,
                    "Prior5", Prior5IterNum[i], 0, move_queue, rjmove_queue,
                    SM, CM, Repeat);

    if (Config.count("posterior2"))
        for (std::size_t i = 0; i != Posterior2IterNum.size(); ++i)
            smc_pair_do<posterior2, rj_posterior2>(sampler, Suffix,
                    zconst_file, "Posterior2", Posterior2IterNum[i], 0,
                    move_queue, rjmove_queue, SM, CM, Repeat);

    if (Config.count("posterior5"))
        for (std::size_t i = 0; i != Posterior5IterNum.size(); ++i)
            smc_pair_do<posterior5, rj_posterior5>(sampler, Suffix,
                    zconst_file, "Posterior5", Posterior5IterNum[i], 0,
                    move_queue, rjmove_queue, SM, CM, Repeat);

    //////////////////////////////////////////////////////////////////////

    zconst_file.close();
    zconst_file.clear();

    return 0;
}
