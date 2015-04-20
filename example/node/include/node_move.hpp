//============================================================================
// vSMC/example/node/include/node_move.hpp
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

#ifndef VSMC_EXAMPLE_NODE_MOVE_HPP
#define VSMC_EXAMPLE_NODE_MOVE_HPP

class node_move_a0 : public BASE_MOVE<node_state, node_move_a0>
{
    public:
    std::size_t move_state(std::size_t, vsmc::SingleParticle<node_state> sp)
    {
        using std::log;

        std::lognormal_distribution<> ra0(0, sp.state(0).a0_sd());
        std::uniform_real_distribution<> runif(0, 1);
        sp.state(0).save_old();

        sp.state(0).a0() *= ra0(sp.rng());
        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() + sp.state(0).log_a0_diff();
        double u = log(runif(sp.rng()));

        return is_valid(p) ? sp.state(0).mh_reject_a0(p, u) :
                             sp.state(0).mh_reject_a0(0, 1);
    }
};

class node_move_a1 : public BASE_MOVE<node_state, node_move_a1>
{
    public:
    std::size_t move_state(std::size_t, vsmc::SingleParticle<node_state> sp)
    {
        using std::log;

        std::lognormal_distribution<> ra1(0, sp.state(0).a1_sd());
        std::uniform_real_distribution<> runif(0, 1);
        sp.state(0).save_old();

        sp.state(0).a1() *= ra1(sp.rng());
        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() + sp.state(0).log_a1_diff();
        double u = log(runif(sp.rng()));

        return is_valid(p) ? sp.state(0).mh_reject_a1(p, u) :
                             sp.state(0).mh_reject_a1(0, 1);
    }
};

class node_move_a2 : public BASE_MOVE<node_state, node_move_a2>
{
    public:
    std::size_t move_state(std::size_t, vsmc::SingleParticle<node_state> sp)
    {
        using std::log;

        std::lognormal_distribution<> ra2(0, sp.state(0).a2_sd());
        std::uniform_real_distribution<> runif(0, 1);
        sp.state(0).save_old();

        sp.state(0).a2() *= ra2(sp.rng());
        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() + sp.state(0).log_a2_diff();
        double u = log(runif(sp.rng()));

        return is_valid(p) ? sp.state(0).mh_reject_a2(p, u) :
                             sp.state(0).mh_reject_a2(0, 1);
    }
};

class node_move_k : public BASE_MOVE<node_state, node_move_k>
{
    public:
    node_move_k(std::size_t k_id) : k_id_(k_id) {}

    std::size_t move_state(std::size_t, vsmc::SingleParticle<node_state> sp)
    {
        using std::log;

        if (k_id_ >= sp.state(0).comp_num() - 1)
            return 0;

        std::uniform_real_distribution<> runif(0, 1);
        std::lognormal_distribution<> rk(0, sp.state(0).k_sd(k_id_));

        sp.state(0).save_old();
        sp.state(0).k(k_id_) *= rk(sp.rng());
        sp.particle().value().log_target(sp.state(0));
        double p =
            sp.state(0).log_target_diff() + sp.state(0).log_k_diff(k_id_);
        double u = log(runif(sp.rng()));

        return is_valid(p) ? sp.state(0).mh_reject_k(p, u, k_id_) :
                             sp.state(0).mh_reject_k(0, 1, k_id_);
    }

    private:
    std::size_t k_id_;
};

class node_move : public BASE_MOVE<node_state, node_move>
{
    public:
    node_move()
    {
        for (std::size_t i = 0; i != MaxCompNum - 1; ++i)
            move_k_.push_back(node_move_k(i));
    }

    std::size_t move_state(
        std::size_t iter, vsmc::SingleParticle<node_state> sp)
    {
        std::size_t acc = 0;
        acc += move_a0_.move_state(iter, sp);
        acc += move_a1_.move_state(iter, sp);
        acc += move_a2_.move_state(iter, sp);
        for (std::size_t i = 0; i != sp.state(0).comp_num() - 1; ++i)
            acc += move_k_[i].move_state(iter, sp);

        return acc;
    }

    private:
    node_move_a0 move_a0_;
    node_move_a1 move_a1_;
    node_move_a2 move_a2_;
    std::vector<node_move_k> move_k_;
};

#endif // VSMC_EXAMPLE_NODE_MOVE_HPP
