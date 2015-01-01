//============================================================================
// vSMC/example/pet/include/pet_move.hpp
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

#ifndef VSMC_EXAMPLE_PET_MOVE
#define VSMC_EXAMPLE_PET_MOVE

class pet_move_phi : public BASE_MOVE<pet_state, pet_move_phi>
{
    public :

    std::size_t move_state (std::size_t, vsmc::SingleParticle<pet_state> sp)
    {
        using std::log;

        vsmc::cxx11::uniform_real_distribution<> runif(0, 1);
        const std::size_t cn = sp.state(0).comp_num();
        sp.state(0).save_old();

        for (std::size_t d = 0; d != cn; ++d) {
            vsmc::cxx11::normal_distribution<> rphi(
                    0, sp.state(0).phi_sd(d));
            sp.state(0).phi(d) += rphi(sp.rng());
        }

        if (!is_valid(sp.particle().value().log_prior(sp.state(0))))
            return sp.state(0).mh_reject_phi(0, 1);
        if (!is_valid(sp.particle().value().log_likelihood(sp.state(0))))
            return sp.state(0).mh_reject_phi(0, 1);
        sp.particle().value().log_target(sp.state(0), false);
        double p = sp.state(0).log_target_diff();
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject_phi(p, u);
    }
};

class pet_move_theta : public BASE_MOVE<pet_state, pet_move_theta>
{
    public :

    std::size_t move_state (std::size_t, vsmc::SingleParticle<pet_state> sp)
    {
        using std::log;

        vsmc::cxx11::uniform_real_distribution<> runif(0, 1);
        const std::size_t cn = sp.state(0).comp_num();
        sp.state(0).save_old();

        for (std::size_t d = 0; d != cn; ++d) {
            vsmc::cxx11::normal_distribution<> rtheta(
                    0, sp.state(0).theta_sd(d));
            sp.state(0).theta(d) += rtheta(sp.rng());
        }

        if (!is_valid(sp.particle().value().log_prior(sp.state(0))))
            return sp.state(0).mh_reject_theta(0, 1);
        if (!is_valid(sp.particle().value().log_likelihood(sp.state(0))))
            return sp.state(0).mh_reject_theta(0, 1);
        sp.particle().value().log_target(sp.state(0), false);
        double p = sp.state(0).log_target_diff();
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject_theta(p, u);
    }
};

class pet_move_lambda : public BASE_MOVE<pet_state, pet_move_lambda>
{
    public :

    std::size_t move_state (std::size_t, vsmc::SingleParticle<pet_state> sp)
    {
        using std::log;

        vsmc::cxx11::lognormal_distribution<> rlambda(
                0, sp.state(0).lambda_sd());
        vsmc::cxx11::uniform_real_distribution<> runif(0, 1);
        sp.state(0).save_old();

        sp.state(0).lambda() *= rlambda(sp.rng());

        if (!is_valid(sp.particle().value().log_prior(sp.state(0))))
            return sp.state(0).mh_reject_lambda(0, 1);
        if (!is_valid(sp.particle().value().log_likelihood(sp.state(0))))
            return sp.state(0).mh_reject_lambda(0, 1);
        sp.particle().value().log_target(sp.state(0), false);
        double p = sp.state(0).log_target_diff() +
            sp.state(0).log_lambda_diff();
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject_lambda(p, u);
    }
};

class pet_move_nu : public BASE_MOVE<pet_state, pet_move_nu>
{
    public :

    std::size_t move_state (std::size_t, vsmc::SingleParticle<pet_state> sp)
    {
        using std::log;

        if (sp.particle().value().model() != StudentT)
            return 0;

        vsmc::cxx11::lognormal_distribution<> rnu(0, sp.state(0).nu_sd());
        vsmc::cxx11::uniform_real_distribution<> runif(0, 1);
        sp.state(0).save_old();

        sp.state(0).nu() *= rnu(sp.rng());

        if (!is_valid(sp.particle().value().log_prior(sp.state(0))))
            return sp.state(0).mh_reject_nu(0, 1);
        if (!is_valid(sp.particle().value().log_likelihood(sp.state(0))))
            return sp.state(0).mh_reject_nu(0, 1);
        sp.particle().value().log_target(sp.state(0), false);
        double p = sp.state(0).log_target_diff() + sp.state(0).log_nu_diff();
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject_nu(p, u);
    }
};

class pet_move : public BASE_MOVE<pet_state, pet_move>
{
    public :

    std::size_t move_state (std::size_t iter,
            vsmc::SingleParticle<pet_state> sp)
    {
        std::size_t acc = 0;
        acc += move_phi_.move_state(iter, sp);
        acc += move_theta_.move_state(iter, sp);
        acc += move_lambda_.move_state(iter, sp);
        acc += move_nu_.move_state(iter, sp);

        return acc;
    }

    private :

    pet_move_phi    move_phi_;
    pet_move_theta  move_theta_;
    pet_move_lambda move_lambda_;
    pet_move_nu     move_nu_;
};

#endif // VSMC_EXAMPLE_PET_MOVE
