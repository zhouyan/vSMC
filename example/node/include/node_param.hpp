//============================================================================
// vSMC/example/node/include/node_param.hpp
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

#ifndef VSMC_EXAMPLE_NODE_PARAM_HPP
#define VSMC_EXAMPLE_NODE_PARAM_HPP

struct node_model {
    std::size_t comp_num;
    double a0, a1, a2, time;
    std::vector<double> k;
    static const int rho = 10;

    void operator()(
        const std::vector<double> &x, std::vector<double> &dxdt) const
    {
        double p = 1;
        double b = x[comp_num - 1];
        for (int i = 0; i != rho; ++i)
            p *= b;
        dxdt[0] = a1 / (1 + a2 * p) - a0 * x[0];
        for (std::size_t i = 1; i != comp_num; ++i)
            dxdt[i] = k[i - 1] * x[i - 1] - a0 * x[i];
    }
};

class node_param
{
    public:
    node_param()
        : comp_num_(0),
          a0_(0),
          a0_old_(0),
          a0_sd_(1),
          a1_(0),
          a1_old_(0),
          a1_sd_(1),
          a2_(0),
          a2_old_(0),
          a2_sd_(1),
          log_prior_(0),
          log_prior_old_(0),
          log_likelihood_(0),
          log_likelihood_old_(0),
          log_target_(0),
          log_target_old_(0),
          alpha_(0),
          alpha_inc_(0)
    {
    }

    node_param &operator=(const node_param &other)
    {
        if (this != &other) {
            comp_num(other.comp_num_);
            a0_ = other.a0_;
            a1_ = other.a1_;
            a2_ = other.a2_;
            for (std::size_t i = 0; i != comp_num_ - 1; ++i) {
                k_[i] = other.k_[i];
                k_sd_[i] = other.k_sd_[i];
            }
            log_prior_ = other.log_prior_;
            log_likelihood_ = other.log_likelihood_;
            log_target_ = other.log_target_;
        }

        return *this;
    }

    // Read only access
    double a0() const { return a0_; }
    double a1() const { return a1_; }
    double a2() const { return a2_; }
    double k(std::size_t d) const { return k_[d]; }
    double log_prior() const { return log_prior_; }
    double log_likelihood() const { return log_likelihood_; }
    double log_target() const { return log_target_; }
    double alpha() const { return alpha_; }
    double alpha_inc() const { return alpha_inc_; }
    double a0_sd() const { return a0_sd_; }
    double a1_sd() const { return a1_sd_; }
    double a2_sd() const { return a2_sd_; }
    double k_sd(std::size_t d) const { return k_sd_[d]; }

    // Read and write access
    double &a0() { return a0_; }
    double &a1() { return a1_; }
    double &a2() { return a2_; }
    double &k(std::size_t d) { return k_[d]; }
    double &log_prior() { return log_prior_; }
    double &log_likelihood() { return log_likelihood_; }
    double &log_target() { return log_target_; }
    double &alpha() { return alpha_; }
    double &alpha_inc() { return alpha_inc_; }
    double &a0_sd() { return a0_sd_; }
    double &a1_sd() { return a1_sd_; }
    double &a2_sd() { return a2_sd_; }
    double &k_sd(std::size_t d) { return k_sd_[d]; }

    // Temp used for fitting data
    double fit(std::size_t d) const { return fit_[d]; }
    double &fit(std::size_t d) { return fit_[d]; }
    const node_model &model() const { return model_; }
    node_model &model() { return model_; }

    std::size_t comp_num() const { return comp_num_; }

    void comp_num(std::size_t num)
    {
        grow(num - 1, k_);
        grow(num - 1, k_sd_);
        grow(num - 1, k_old_);
        grow(num - 1, model_.k);
        grow(num, fit_);

        model_.comp_num = num;
        comp_num_ = num;
    }

    void swap(node_param &other)
    {
        using std::swap;

        if (this != &other) {
            swap(a0_, other.a0_);
            swap(a1_, other.a1_);
            swap(a2_, other.a2_);
            swap(k_, other.k_);
            swap(k_sd_, other.k_sd_);
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
        a0_old_ = a0_;
        a1_old_ = a1_;
        a2_old_ = a2_;
        for (std::size_t i = 0; i != comp_num_ - 1; ++i)
            k_old_[i] = k_[i];

        log_prior_old_ = log_prior_;
        log_likelihood_old_ = log_likelihood_;
        log_target_old_ = log_prior_ + alpha_ * log_likelihood_;
    }

    double log_target_diff() const { return log_target_ - log_target_old_; }

    double log_a0_diff() const { return std::log(a0_) - std::log(a0_old_); }

    double log_a1_diff() const { return std::log(a1_) - std::log(a1_old_); }

    double log_a2_diff() const { return std::log(a2_) - std::log(a2_old_); }

    double log_k_diff(std::size_t d) const
    {
        return std::log(k_[d]) - std::log(k_old_[d]);
    }

    unsigned mh_reject_a0(double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            a0_ = a0_old_;
            return 0;
        } else {
            a0_old_ = a0_;
            return 1;
        }
    }

    unsigned mh_reject_a1(double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            a1_ = a1_old_;
            return 0;
        } else {
            a1_old_ = a1_;
            return 1;
        }
    }

    unsigned mh_reject_a2(double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            a2_ = a2_old_;
            return 0;
        } else {
            a2_old_ = a2_;
            return 1;
        }
    }

    unsigned mh_reject_k(double p, double u, std::size_t d)
    {
        mh_reject_common(p, u);
        if (p < u) {
            k_[d] = k_old_[d];
            return 0;
        } else {
            k_old_[d] = k_[d];
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

    void update_model()
    {
        model_.time = 0;
        model_.a0 = a0_;
        model_.a1 = a1_;
        model_.a2 = a2_;
        for (std::size_t i = 0; i != comp_num_ - 1; ++i)
            model_.k[i] = k_[i];
        for (std::size_t i = 0; i != comp_num_; ++i)
            fit_[i] = 0;
    }

    void fit_model(double time)
    {
        double delta = std::min VSMC_MNE(Resolution, time - model_.time);
        do {
            update_fit(delta);
            delta = std::min VSMC_MNE(Resolution, time - model_.time);
        } while (delta > 1e-10);
    }

    private:
    std::size_t comp_num_;

    double a0_;
    double a0_old_;
    double a0_sd_;

    double a1_;
    double a1_old_;
    double a1_sd_;

    double a2_;
    double a2_old_;
    double a2_sd_;

    double log_prior_;
    double log_prior_old_;

    double log_likelihood_;
    double log_likelihood_old_;

    double log_target_;
    double log_target_old_;

    double alpha_;
    double alpha_inc_;

    std::vector<double> k_;
    std::vector<double> k_old_;
    std::vector<double> k_sd_;

    std::vector<double> fit_;
    std::vector<double> ode_k1_fit_;
    std::vector<double> ode_k2_fit_;
    std::vector<double> ode_k3_fit_;
    std::vector<double> ode_k4_fit_;
    std::vector<double> ode_k1_inc_;
    std::vector<double> ode_k2_inc_;
    std::vector<double> ode_k3_inc_;
    std::vector<double> ode_k4_inc_;

    node_model model_;

    void update_fit(double delta)
    {
        std::size_t n = fit_.size();
        ode_k1_fit_.resize(n);
        ode_k2_fit_.resize(n);
        ode_k3_fit_.resize(n);
        ode_k4_fit_.resize(n);
        ode_k1_inc_.resize(n);
        ode_k2_inc_.resize(n);
        ode_k3_inc_.resize(n);
        ode_k4_inc_.resize(n);

        for (std::size_t i = 0; i != n; ++i)
            ode_k1_fit_[i] = fit_[i];
        model_(ode_k1_fit_, ode_k1_inc_);
        for (std::size_t i = 0; i != n; ++i)
            ode_k1_inc_[i] *= delta;

        for (std::size_t i = 0; i != n; ++i)
            ode_k2_fit_[i] = fit_[i] + 0.5 * ode_k1_inc_[i];
        model_(ode_k2_fit_, ode_k2_inc_);
        for (std::size_t i = 0; i != n; ++i)
            ode_k2_inc_[i] *= delta;

        for (std::size_t i = 0; i != n; ++i)
            ode_k3_fit_[i] = fit_[i] + 0.5 * ode_k2_inc_[i];
        model_(ode_k3_fit_, ode_k3_inc_);
        for (std::size_t i = 0; i != n; ++i)
            ode_k3_inc_[i] *= delta;

        for (std::size_t i = 0; i != n; ++i)
            ode_k4_fit_[i] = fit_[i] + ode_k3_inc_[i];
        model_(ode_k4_fit_, ode_k4_inc_);
        for (std::size_t i = 0; i != n; ++i)
            ode_k4_inc_[i] *= delta;

        for (std::size_t i = 0; i != n; ++i) {
            fit_[i] += (ode_k1_inc_[i] + 2 * ode_k2_inc_[i] +
                           2 * ode_k3_inc_[i] + ode_k4_inc_[i]) /
                6;
        }
        model_.time += delta;
    }
};

template <typename OutputStream>
inline OutputStream &operator<<(OutputStream &os, const node_param &param)
{
    os << param.a0() << ' ' << param.a1() << ' ' << param.a2() << ' ';
    for (std::size_t d = 0; d != param.comp_num() - 1; ++d)
        os << param.k(d) << ' ';
    os << param.log_prior() << ' ' << param.log_likelihood();

    return os;
}

#endif // VSMC_EXAMPLE_NODE_PARAM_HPP
