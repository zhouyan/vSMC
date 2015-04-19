//============================================================================
// vSMC/example/gmm/include/gmm_monitor.hpp
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

#ifndef VSMC_EXAMPLE_GMM_MONITOR_HPP
#define VSMC_EXAMPLE_GMM_MONITOR_HPP

class gmm_monitor : public BASE_MONITOR<gmm_state, gmm_monitor>
{
    public:
    void monitor_state(std::size_t,
                       std::size_t dim,
                       vsmc::ConstSingleParticle<gmm_state> csp,
                       double *res)
    {
        assert(!(dim % 3));
        assert(csp.state(0).comp_num() >= dim / 3);
        const std::size_t cn = dim / 3;
        for (std::size_t d = 0; d != cn; ++d) {
            res[d + 1] = csp.state(0).mu(d);
            res[d + 1 + cn] = csp.state(0).lambda(d);
            res[d + 1 + 2 * cn] = csp.state(0).weight(d);
        }
    }
};

class gmm_comp_num : public BASE_MONITOR<gmm_state, gmm_comp_num>
{
    public:
    void monitor_state(std::size_t,
                       std::size_t,
                       vsmc::ConstSingleParticle<gmm_state> csp,
                       double *res)
    {
        *res = static_cast<double>(csp.state(0).comp_num());
    }
};

class gmm_rm_mu : public BASE_MONITOR<gmm_state, gmm_rm_mu>
{
    public:
    void monitor_state(std::size_t,
                       std::size_t dim,
                       vsmc::ConstSingleParticle<gmm_state> csp,
                       double *res)
    {
        double mu = 0;
        const std::size_t cn = csp.state(0).comp_num();
        for (std::size_t i = 0; i != cn; ++i)
            mu += csp.state(0).mu(i);
        mu /= cn;
        res[0] = mu;
        for (std::size_t d = 1; d != dim; ++d)
            res[d] = res[d - 1] * mu;
    }
};

class gmm_rm_lambda : public BASE_MONITOR<gmm_state, gmm_rm_lambda>
{
    public:
    void monitor_state(std::size_t,
                       std::size_t dim,
                       vsmc::ConstSingleParticle<gmm_state> csp,
                       double *res)
    {
        using std::log;

        double log_lambda = 0;
        const std::size_t cn = csp.state(0).comp_num();
        for (std::size_t i = 0; i != cn; ++i)
            log_lambda += log(csp.state(0).lambda(i));
        log_lambda /= cn;
        res[0] = log_lambda;
        for (std::size_t d = 1; d != dim; ++d)
            res[d] = res[d - 1] * log_lambda;
    }
};

class gmm_rm_weight : public BASE_MONITOR<gmm_state, gmm_rm_weight>
{
    public:
    void monitor_state(std::size_t,
                       std::size_t dim,
                       vsmc::ConstSingleParticle<gmm_state> csp,
                       double *res)
    {
        double max_w = csp.state(0).weight(0);
        const std::size_t cn = csp.state(0).comp_num();
        for (std::size_t i = 0; i != cn; ++i)
            if (max_w < csp.state(0).weight(i))
                max_w = csp.state(0).weight(i);
        double logit_weight = 0;
        for (std::size_t i = 0; i != cn; ++i)
            logit_weight += csp.state(0).weight(i) / max_w;
        logit_weight /= cn;
        res[0] = logit_weight;
        for (std::size_t d = 1; d != dim; ++d)
            res[d] = res[d - 1] * logit_weight;
    }
};

class gmm_moments : public BASE_MONITOR<gmm_state, gmm_moments>
{
    public:
    void monitor_state(std::size_t,
                       std::size_t dim,
                       vsmc::ConstSingleParticle<gmm_state> csp,
                       double *res)
    {
        for (std::size_t i = 0; i != dim; ++i)
            res[i] = 0;
        assert(dim >= csp.state(0).comp_num() * 3);
        const std::size_t cn = csp.state(0).comp_num();
        std::size_t offset = 0;
        for (std::size_t d = 0; d != cn; ++d)
            res[offset++] = csp.state(0).mu(d);
        for (std::size_t d = 0; d != cn; ++d)
            res[offset++] = csp.state(0).lambda(d);
        for (std::size_t d = 0; d != cn; ++d)
            res[offset++] = csp.state(0).weight(d);
    }
};

#endif  // VSMC_EXAMPLE_GMM_MONITOR_HPP
