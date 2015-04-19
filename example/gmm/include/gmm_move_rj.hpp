//============================================================================
// vSMC/example/gmm/include/gmm_move_rj.hpp
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

#ifndef VSMC_EXAMPLE_GMM_MOVE_RJ_HPP
#define VSMC_EXAMPLE_GMM_MOVE_RJ_HPP

#ifndef NDEBUG
inline void gmm_check_odd_nums(std::size_t old_num, std::size_t new_num)
{
    bool valid = false;
    if (old_num - new_num == 1)
        valid = true;
    if (new_num - old_num == 1)
        valid = true;
    if (!valid)
        throw std::invalid_argument("Difference is not ONE");
}
#else
inline void gmm_check_odd_nums(std::size_t, std::size_t) {}
#endif

class gmm_logodds_flat
{
    public:
    double operator()(std::size_t old_num,
                      std::size_t new_num,
                      const gmm_param &)
    {
        gmm_check_odd_nums(old_num, new_num);

        return 0;
    }
};

class gmm_logodds_pair
{
    public:
    double operator()(std::size_t old_num,
                      std::size_t new_num,
                      const gmm_param &state)
    {
        using std::log;

        gmm_check_odd_nums(old_num, new_num);

        if (old_num > new_num) {
            if (state.beta() >= 1)
                return -std::numeric_limits<double>::infinity();
            else if (state.beta() <= 0)
                return std::numeric_limits<double>::infinity();
            else
                return log((1 - state.beta()) / state.beta());
        } else {
            if (state.beta() >= 1)
                return std::numeric_limits<double>::infinity();
            else if (state.beta() <= 0)
                return -std::numeric_limits<double>::infinity();
            else
                return log(state.beta() / (1 - state.beta()));
        }
    }
};

// Split or combine
template <typename LogPriorOdd>
class gmm_rj_sc : public BASE_MOVE<gmm_state, gmm_rj_sc<LogPriorOdd>>
{
    public:
    gmm_rj_sc(std::size_t minc = MinCompNum, std::size_t maxc = MaxCompNum)
        : min_comp_(minc), max_comp_(maxc)
    {
    }

    std::size_t move_state(std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        using std::sqrt;
        using std::log;

        sp.state(0).save_old();
        bool split;
        const std::size_t cn = sp.state(0).comp_num();
        if (cn == max_comp_)
            split = false;
        else if (cn == min_comp_)
            split = true;
        else {
            std::bernoulli_distribution rsplit(0.5);
            split = rsplit(sp.rng());
        }

        if (split) {  // do split move
            std::gamma_distribution<> rgamma2(2, 1);
            std::uniform_int_distribution<std::size_t> rj(0, cn - 1);
            std::uniform_real_distribution<> runif(0, 1);

            // generate u1, u2, u3
            double a, b;
            a = rgamma2(sp.rng());
            b = rgamma2(sp.rng());
            double u1 = a / (a + b);
            a = rgamma2(sp.rng());
            b = rgamma2(sp.rng());
            double u2 = a / (a + b);
            double u3 = runif(sp.rng());

            // component to split
            std::size_t id = rj(sp.rng());
            double mu = sp.state(0).mu(id);
            double lambda = sp.state(0).lambda(id);
            double weight = sp.state(0).weight(id);

            // propose two new components
            double weight1 = weight * u1;
            double weight2 = weight - weight1;

            double mu1 = mu - u2 * sqrt(weight2 / weight1 / lambda);
            double mu2 = mu + u2 * sqrt(weight1 / weight2 / lambda);

            double lambda1 = lambda / (u3 * (1 - u2 * u2) * weight / weight1);
            double lambda2 =
                lambda / ((1 - u3) * (1 - u2 * u2) * weight / weight2);

            // insert the two new components
            std::size_t id1 = id;
            std::size_t id2 = id + 1;
            sp.state(0).insert(id2);
            sp.state(0).mu(id1) = mu1;
            sp.state(0).mu(id2) = mu2;
            sp.state(0).lambda(id1) = lambda1;
            sp.state(0).lambda(id2) = lambda2;
            sp.state(0).weight(id1) = weight1;
            sp.state(0).weight(id2) = weight2;

            // still ordered?
            if (!sp.state(0).ordered())
                return sp.state(0).mh_reject_rj(0, 1);

            sp.particle().value().log_target(sp.state(0));
            double p = static_cast<double>(cn + 1);
            if (max_comp_ - min_comp_ > 1) {
                if (cn == min_comp_)
                    p *= 0.5;
                if (cn == max_comp_ - 1)
                    p *= 2;
            }
            p *= weight * lambda * u1 * (1 - u1) / u3 / u3;
            double tmp = 1 - u2 * u2;
            p /= tmp * tmp * tmp;
            tmp = 1 - u3;
            p /= tmp * tmp;
            tmp = sqrt((1 - u1) / u1);
            p *= (tmp + 1 / tmp);
            p /= sqrt(lambda);
            p /= 36 * u1 * u2 * (1 - u1) * (1 - u2);
            p = log(p) + sp.state(0).log_target_diff();
            double log_odd = log_prior_odd_(cn, cn + 1, sp.state(0));
            p += log_odd;
            double u = log(runif(sp.rng()));

            return sp.state(0).mh_reject_rj(p, u);
        } else {  // do combine move
            std::uniform_int_distribution<std::size_t> rj(0, cn - 2);
            std::uniform_real_distribution<> runif(0, 1);
            std::size_t id1 = rj(sp.rng());
            std::size_t id2 = id1 + 1;

            // old components
            double weight1 = sp.state(0).weight(id1);
            double weight2 = sp.state(0).weight(id2);
            double mu1 = sp.state(0).mu(id1);
            double mu2 = sp.state(0).mu(id2);
            double lambda1 = sp.state(0).lambda(id1);
            double lambda2 = sp.state(0).lambda(id2);

            // compute new components
            double weight = weight1 + weight2;
            double mu = (weight1 * mu1 + weight2 * mu2) / weight;
            double lambda = 0;
            lambda += weight1 * (mu1 * mu1 + 1 / lambda1);
            lambda += weight2 * (mu2 * mu2 + 1 / lambda2);
            lambda /= weight;
            lambda -= mu * mu;
            lambda = 1 / lambda;

            // insert new components
            std::size_t id = id1;
            sp.state(0).remove(id2);
            sp.state(0).mu(id) = mu;
            sp.state(0).lambda(id) = lambda;
            sp.state(0).weight(id) = weight;

            double u1 = weight1 / weight;
            double u2 = (mu - mu1) * sqrt(weight1 * lambda / weight2);
            double u3 = lambda / lambda1 * weight1 / weight / (1 - u2 * u2);

            sp.particle().value().log_target(sp.state(0));
            double p = static_cast<double>(cn);
            if (max_comp_ - min_comp_ > 1) {
                if (cn == min_comp_ + 1)
                    p *= 0.5;
                if (cn == max_comp_)
                    p *= 2;
            }
            p *= weight * lambda * u1 * (1 - u1) / u3 / u3;
            double tmp = 1 - u2 * u2;
            p /= tmp * tmp * tmp;
            tmp = 1 - u3;
            p /= tmp * tmp;
            tmp = sqrt((1 - u1) / u1);
            p *= (tmp + 1 / tmp);
            p /= sqrt(lambda);
            p /= 36 * u1 * u2 * (1 - u1) * (1 - u2);
            p = -log(p) + sp.state(0).log_target_diff();
            double log_odd = log_prior_odd_(cn, cn - 1, sp.state(0));
            p += log_odd;
            double u = log(runif(sp.rng()));

            return sp.state(0).mh_reject_rj(p, u);
        }
    }

    private:
    std::size_t min_comp_;
    std::size_t max_comp_;
    LogPriorOdd log_prior_odd_;
};

// Birth or death
template <typename LogPriorOdd>
class gmm_rj_bd : public BASE_MOVE<gmm_state, gmm_rj_bd<LogPriorOdd>>
{
    public:
    gmm_rj_bd(std::size_t minc = MinCompNum, std::size_t maxc = MaxCompNum)
        : min_comp_(minc), max_comp_(maxc)
    {
    }

    std::size_t move_state(std::size_t, vsmc::SingleParticle<gmm_state> sp)
    {
        sp.state(0).save_old();
        bool birth;
        const std::size_t cn = sp.state(0).comp_num();
        if (cn == max_comp_)
            birth = false;
        else if (cn == min_comp_)
            birth = true;
        else {
            std::bernoulli_distribution rbirth(0.5);
            birth = rbirth(sp.rng());
        }

        if (birth) {  // do birth move
            std::gamma_distribution<> rgamma1(1, 1);
            std::gamma_distribution<> rgammak(static_cast<double>(cn), 1);
            std::uniform_real_distribution<> runif(0, 1);
            std::normal_distribution<> rmu(sp.particle().value().mu0(),
                                           sp.particle().value().sd0());
            std::gamma_distribution<> rlambda(sp.particle().value().shape0(),
                                              sp.particle().value().scale0());

            // propose new component
            double a = rgamma1(sp.rng());
            double b = rgammak(sp.rng());
            double weight = a / (a + b);
            double mu = rmu(sp.rng());
            double lambda = rlambda(sp.rng());
            double weight_1 = 1 - weight;

            // where to insert
            std::size_t id = 0;
            while (sp.state(0).mu(id) < mu && id != cn)
                ++id;
            sp.state(0).insert(id);
            sp.state(0).mu(id) = mu;
            sp.state(0).lambda(id) = lambda;
            sp.state(0).weight(id) = weight;

            double sum_weight = 0;
            for (std::size_t d = 0; d != cn + 1; ++d)
                sum_weight += sp.state(0).weight(d);
            for (std::size_t d = 0; d != cn + 1; ++d)
                sp.state(0).weight(d) /= sum_weight;

            sp.particle().value().log_target(sp.state(0));
            double p = (cn + 1) * weight_1;
            if (max_comp_ - min_comp_ > 1) {
                if (cn == min_comp_)
                    p *= 0.5;
                else if (cn == max_comp_ - 1)
                    p *= 2;
            }
            p = -log(p) + sp.state(0).log_target_diff();
            double log_odd = log_prior_odd_(cn, cn + 1, sp.state(0));
            p += log_odd;
            double u = log(runif(sp.rng()));

            return sp.state(0).mh_reject_rj(p, u);
        } else {  // do death move
            std::uniform_int_distribution<std::size_t> rj(0, cn - 1);
            std::uniform_real_distribution<> runif(0, 1);

            std::size_t id = rj(sp.rng());
            double weight_1 = 1 - sp.state(0).weight(id);
            sp.state(0).remove(id);

            double sum_weight = 0;
            for (std::size_t d = 0; d != cn - 1; ++d)
                sum_weight += sp.state(0).weight(d);
            for (std::size_t d = 0; d != cn - 1; ++d)
                sp.state(0).weight(d) /= sum_weight;

            sp.particle().value().log_target(sp.state(0));
            double p = cn * weight_1;
            if (max_comp_ - min_comp_ > 1) {
                if (cn == min_comp_ + 1)
                    p *= 0.5;
                else if (cn == max_comp_)
                    p *= 2;
            }
            p = -log(p) + sp.state(0).log_target_diff();
            double log_odd = log_prior_odd_(cn, cn - 1, sp.state(0));
            p += log_odd;
            double u = log(runif(sp.rng()));

            return sp.state(0).mh_reject_rj(p, u);
        }
    }

    private:
    std::size_t min_comp_;
    std::size_t max_comp_;
    LogPriorOdd log_prior_odd_;
};

#endif  // VSMC_EXAMPLE_GMM_MOVE_RJ_HPP
