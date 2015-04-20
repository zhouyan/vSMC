//============================================================================
// vSMC/example/gmm/include/gmm_param.hpp
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

#ifndef VSMC_EXAMPLE_GMM_PARAM_HPP
#define VSMC_EXAMPLE_GMM_PARAM_HPP

#include <algorithm>

class gmm_param
{
    public:
    gmm_param()
        : comp_num_(0),
          comp_num_old_(0),
          log_prior_(0),
          log_prior_old_(0),
          log_likelihood_(0),
          log_likelihood_old_(0),
          log_target_(0),
          log_target_old_(0),
          alpha_(0),
          alpha_inc_(0),
          beta_(0),
          beta_inc_(0),
          mu_sd_(1),
          lambda_sd_(1),
          weight_sd_(1)
    {
    }

    gmm_param(const gmm_param &other)
        : comp_num_(0),
          comp_num_old_(0),
          log_prior_(0),
          log_prior_old_(0),
          log_likelihood_(0),
          log_likelihood_old_(0),
          log_target_(0),
          log_target_old_(0),
          alpha_(0),
          alpha_inc_(0),
          beta_(0),
          beta_inc_(0),
          mu_sd_(1),
          lambda_sd_(1),
          weight_sd_(1)
    {
        comp_num(other.comp_num_);
        for (std::size_t i = 0; i != comp_num_; ++i) {
            mu_[i] = other.mu_[i];
            lambda_[i] = other.lambda_[i];
            log_lambda_[i] = other.log_lambda_[i];
            weight_[i] = other.weight_[i];
        }
        log_prior_ = other.log_prior_;
        log_likelihood_ = other.log_likelihood_;
        log_target_ = other.log_target_;
    }

    // assignment only manipulate parameters and log likelihood etc.
    gmm_param &operator=(const gmm_param &other)
    {
        if (this != &other) {
            comp_num(other.comp_num_);
            for (std::size_t i = 0; i != comp_num_; ++i) {
                mu_[i] = other.mu_[i];
                lambda_[i] = other.lambda_[i];
                log_lambda_[i] = other.log_lambda_[i];
                weight_[i] = other.weight_[i];
            }
            log_prior_ = other.log_prior_;
            log_likelihood_ = other.log_likelihood_;
            log_target_ = other.log_target_;
        }

        return *this;
    }

    // Read only access
    double mu(std::size_t d) const { return mu_[d]; }
    double lambda(std::size_t d) const { return lambda_[d]; }
    double log_lambda(std::size_t d) const { return log_lambda_[d]; }
    double weight(std::size_t d) const { return weight_[d]; }
    double log_prior() const { return log_prior_; }
    double log_likelihood() const { return log_likelihood_; }
    double log_target() const { return log_target_; }
    double alpha() const { return alpha_; }
    double alpha_inc() const { return alpha_inc_; }
    double beta() const { return beta_; }
    double beta_inc() const { return beta_inc_; }
    double mu_sd() const { return mu_sd_; }
    double lambda_sd() const { return lambda_sd_; }
    double weight_sd() const { return weight_sd_; }

    // Read and write access
    double &mu(std::size_t d) { return mu_[d]; }
    double &lambda(std::size_t d) { return lambda_[d]; }
    double &log_lambda(std::size_t d) { return log_lambda_[d]; }
    double &weight(std::size_t d) { return weight_[d]; }
    double &log_prior() { return log_prior_; }
    double &log_likelihood() { return log_likelihood_; }
    double &log_target() { return log_target_; }
    double &alpha() { return alpha_; }
    double &alpha_inc() { return alpha_inc_; }
    double &beta() { return beta_; }
    double &beta_inc() { return beta_inc_; }
    double &mu_sd() { return mu_sd_; }
    double &lambda_sd() { return lambda_sd_; }
    double &weight_sd() { return weight_sd_; }

    std::size_t comp_num() const { return comp_num_; }

    // Minimize dynamic memory allocation
    // Not suitable if many particles will occaciaonally grow very large
    void comp_num(std::size_t num)
    {
        grow(num, mu_);
        grow(num, lambda_);
        grow(num, log_lambda_);
        grow(num, weight_);
        grow(num, mu_old_);
        grow(num, lambda_old_);
        grow(num, log_lambda_old_);
        grow(num, weight_old_);
        comp_num_ = num;
    }

    void sort_mu() { std::sort(mu_.begin(), mu_.end()); }

    bool ordered() const
    {
        for (std::size_t d = 1; d != comp_num_; ++d)
            if (mu_[d] < mu_[d - 1])
                return false;

        return true;
    }

    // swap is an expansive operation, only population MCMC shall use it
    void swap(gmm_param &other)
    {
        using std::swap;

        if (this != &other) {
            swap(mu_, other.mu_);
            swap(lambda_, other.lambda_);
            swap(log_lambda_, other.log_lambda_);
            swap(weight_, other.weight_);
            swap(log_prior_, other.log_prior_);
            swap(log_likelihood_, other.log_likelihood_);
            swap(log_target_, other.log_target_);
            std::size_t cn = comp_num_;
            comp_num(other.comp_num_);
            other.comp_num(cn);
        }
    }

    void save_old()
    {
        comp_num_old_ = comp_num_;

        for (std::size_t i = 0; i != comp_num_; ++i) {
            mu_old_[i] = mu_[i];
            lambda_old_[i] = lambda_[i];
            log_lambda_old_[i] = log_lambda_[i];
            weight_old_[i] = weight_[i];
        }

        log_prior_old_ = log_prior_;
        log_likelihood_old_ = log_likelihood_;
        log_target_old_ = log_prior_ + alpha_ * log_likelihood_;
    }

    void remove(std::size_t id)
    {
        for (std::size_t d = id; d != comp_num_ - 1; ++d) {
            mu_[d] = mu_[d + 1];
            lambda_[d] = lambda_[d + 1];
            log_lambda_[d] = log_lambda_[d + 1];
            weight_[d] = weight_[d + 1];
        }
        comp_num(comp_num_ - 1);
    }

    // after insert, id position is empty for insertion
    void insert(std::size_t id)
    {
        comp_num(comp_num_ + 1);
        for (std::size_t d = comp_num_ - 1; d != id; --d) {
            mu_[d] = mu_[d - 1];
            lambda_[d] = lambda_[d - 1];
            log_lambda_[d] = log_lambda_[d - 1];
            weight_[d] = weight_[d - 1];
        }
    }

    double log_target_diff() const { return log_target_ - log_target_old_; }

    double log_lambda_diff() const
    {
        double sum = 0;
        for (std::size_t i = 0; i != comp_num_; ++i)
            sum += log_lambda_[i] - log_lambda_old_[i];

        return sum;
    }

    void log_lambda()
    {
        using std::log;

        for (std::size_t i = 0; i != comp_num_; ++i)
            log_lambda_[i] = log(lambda_[i]);
    }

    double logit_weight_diff() const
    {
        using std::log;

        double sum_lw, sum_llw;

        sum_lw = 1;
        sum_llw = 0;
        for (std::size_t d = 0; d != comp_num_ - 1; ++d) {
            double w = weight_[d] / weight_[comp_num_ - 1];
            sum_lw += w;
            sum_llw += log(w);
        }
        sum_llw -= comp_num_ * log(sum_lw);
        double lp = sum_llw;

        sum_lw = 1;
        sum_llw = 0;
        for (std::size_t d = 0; d != comp_num_ - 1; ++d) {
            double w = weight_old_[d] / weight_old_[comp_num_ - 1];
            sum_lw += w;
            sum_llw += log(w);
        }
        sum_llw -= comp_num_ * log(sum_lw);
        double lp_old = sum_llw;

        return lp - lp_old;
    }

    unsigned mh_reject_mu(double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            for (std::size_t i = 0; i != comp_num_; ++i)
                mu_[i] = mu_old_[i];
            return 0;
        } else {
            for (std::size_t i = 0; i != comp_num_; ++i)
                mu_old_[i] = mu_[i];
            return 1;
        }
    }

    unsigned mh_reject_lambda(double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            for (std::size_t i = 0; i != comp_num_; ++i) {
                lambda_[i] = lambda_old_[i];
                log_lambda_[i] = log_lambda_old_[i];
            }
            return 0;
        } else {
            for (std::size_t i = 0; i != comp_num_; ++i) {
                lambda_old_[i] = lambda_[i];
                log_lambda_old_[i] = log_lambda_[i];
            }
            return 1;
        }
    }

    unsigned mh_reject_weight(double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            for (std::size_t i = 0; i != comp_num_; ++i)
                weight_[i] = weight_old_[i];
            return 0;
        } else {
            for (std::size_t i = 0; i != comp_num_; ++i)
                weight_old_[i] = weight_[i];
            return 1;
        }
    }

    unsigned mh_reject_rj(double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            comp_num(comp_num_old_);
            for (std::size_t i = 0; i != comp_num_; ++i) {
                mu_[i] = mu_old_[i];
                lambda_[i] = lambda_old_[i];
                log_lambda_[i] = log_lambda_old_[i];
                weight_[i] = weight_old_[i];
            }
            return 0;
        } else {
            return 1;
        }
    }

    void mh_reject_common(double p, double u)
    {
        if (p < u) {
            log_prior_ = log_prior_old_;
            log_likelihood_ = log_likelihood_old_;
            log_target_ = log_target_old_;
        } else {
            log_prior_old_ = log_prior_;
            log_likelihood_old_ = log_likelihood_;
            log_target_old_ = log_target_;
        }
    }

    private:
    std::size_t comp_num_;
    std::size_t comp_num_old_;

    double log_prior_;
    double log_prior_old_;

    double log_likelihood_;
    double log_likelihood_old_;

    double log_target_;
    double log_target_old_;

    double alpha_;
    double alpha_inc_;

    double beta_;
    double beta_inc_;

    double mu_sd_;
    double lambda_sd_;
    double weight_sd_;

    std::vector<double> mu_;
    std::vector<double> mu_old_;

    std::vector<double> lambda_;
    std::vector<double> lambda_old_;

    std::vector<double> log_lambda_;
    std::vector<double> log_lambda_old_;

    std::vector<double> weight_;
    std::vector<double> weight_old_;
};

template <typename OutputStream>
inline OutputStream &operator<<(OutputStream &os, const gmm_param &param)
{
    for (std::size_t d = 0; d != param.comp_num(); ++d) {
        os << param.mu(d) << ' ';
        os << param.lambda(d) << ' ';
        os << param.weight(d) << ' ';
    }
    os << param.log_prior() << ' ' << param.log_likelihood();

    return os;
}

#endif // VSMC_EXAMPLE_GMM_PARAM_HPP
