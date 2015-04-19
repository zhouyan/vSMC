//============================================================================
// vSMC/example/gmm/include/gmm_move.hpp
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

#ifndef VSMC_EXAMPLE_GMM_MOVE_HPP
#define VSMC_EXAMPLE_GMM_MOVE_HPP

class gmm_move_mu : public BASE_MOVE<gmm_state, gmm_move_mu>
{
    public :

    std::size_t move_state (std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        using std::log;

        std::normal_distribution<> rmu(0, sp.state(0).mu_sd());
        std::uniform_real_distribution<> runif(0, 1);
        const std::size_t cn = sp.state(0).comp_num();
        sp.state(0).save_old();

        for (std::size_t d = 0; d != cn; ++d)
            sp.state(0).mu(d) += rmu(sp.rng());

        if (sp.particle().value().ordered() && !sp.state(0).ordered())
            return sp.state(0).mh_reject_mu(0, 1);

        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff();
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject_mu(p, u);
    }
};

class gmm_move_lambda : public BASE_MOVE<gmm_state, gmm_move_lambda>
{
    public :

    std::size_t move_state (std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        using std::log;

        std::lognormal_distribution<> rlambda(
                0, sp.state(0).lambda_sd());
        std::uniform_real_distribution<> runif(0, 1);
        const std::size_t cn = sp.state(0).comp_num();
        sp.state(0).save_old();

        for (std::size_t d = 0; d != cn; ++d)
            sp.state(0).lambda(d) *= rlambda(sp.rng());
        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() +
            sp.state(0).log_lambda_diff();
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject_lambda(p, u);
    }
};

class gmm_move_weight : public BASE_MOVE<gmm_state, gmm_move_weight>
{
    public :

    std::size_t move_state (std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        using std::log;
        using std::exp;

        std::normal_distribution<> rweight(
                0, sp.state(0).weight_sd());
        std::uniform_real_distribution<> runif(0, 1);
        const std::size_t cn = sp.state(0).comp_num();
        sp.state(0).save_old();

        double sum_weight = 1;
        for (std::size_t d = 0; d != cn - 1; ++d) {
            sp.state(0).weight(d) = log(
                    sp.state(0).weight(d) /
                    sp.state(0).weight(cn - 1));
            sp.state(0).weight(d) += rweight(sp.rng());
            sp.state(0).weight(d) = exp(sp.state(0).weight(d));
            sum_weight += sp.state(0).weight(d);
        }
        sp.state(0).weight(cn - 1) = 1;
        for (std::size_t d = 0; d != cn; ++d)
            sp.state(0).weight(d) /= sum_weight;
        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() +
            sp.state(0).logit_weight_diff();
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject_weight(p, u);
    }
};

class gmm_move : public BASE_MOVE<gmm_state, gmm_move>
{
    public :

    std::size_t move_state (std::size_t iter,
            vsmc::SingleParticle<gmm_state> sp)
    {
        std::size_t acc = 0;
        acc += move_mu_.move_state(iter, sp);
        acc += move_lambda_.move_state(iter, sp);
        acc += move_weight_.move_state(iter, sp);

        return acc;
    }

    private :

    gmm_move_mu     move_mu_;
    gmm_move_lambda move_lambda_;
    gmm_move_weight move_weight_;
};

#endif // VSMC_EXAMPLE_GMM_MOVE_HPP
