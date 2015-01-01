//============================================================================
// vSMC/example/pet/include/pet_state.hpp
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

#ifndef VSMC_EXAMPLE_PET_STATE_HPP
#define VSMC_EXAMPLE_PET_STATE_HPP

enum PETModel {Normal, StudentT};

#ifdef __INTEL_COMPILER
#pragma warning(push)
#pragma warning(disable:411)
#endif

struct data_info
{
    const std::size_t data_num;
    const double *data_value;
};

struct time_info
{
    const std::size_t time_num;
    const double *time_value;
};

struct conv_info
{
    const std::size_t conv_num;
    const double conv_mul;
    const double *conv_value;
};

struct prior_info
{
    const std::size_t prior_num;
    const double *prior_value;
};

struct sd_info
{
    const std::size_t sd_num;
    const double *sd_value;
};

struct model_info
{
    const double decay;
    const PETModel model;
};

struct pet_info
{
    const data_info  *data;  bool read_data;
    const time_info  *time;  bool read_time;
    const conv_info  *conv;  bool read_conv;
    const prior_info *prior; bool read_prior;
    const sd_info    *sd;    bool read_sd;
    const model_info *model; bool read_model;
};

#ifdef __INTEL_COMPILER
#pragma warning(pop)
#endif

class pet_state :
    public BASE_STATE<vsmc::StateMatrix<vsmc::RowMajor, 1, pet_param> >
{
    public :

    pet_state (size_type N) :
        BASE_STATE<vsmc::StateMatrix<vsmc::RowMajor, 1, pet_param> >(N) {}

    double log_likelihood_const () const
    {
        return model_ == Normal ?
            log_likelihood_const_normal():
            log_likelihood_const_studentt();
    }

    double log_prior (pet_param &state) const
    {
        using std::log;

        double lp = -std::numeric_limits<double>::infinity();
        for (std::size_t d = 1; d != state.comp_num(); ++d) {
            if (state.theta(d) < state.theta(d - 1))
                return state.log_prior() = lp;
        }
        for (std::size_t d = 0; d != state.comp_num(); ++d) {
            if (
                    state.phi(d) < phi_lb0_[d] ||
                    state.phi(d) > phi_ub0_[d] ||
                    state.theta(d) < theta_lb0_[d] ||
                    state.theta(d) > theta_ub0_[d]
               ) {
                return state.log_prior() = lp;
            }
        }
        if (state.lambda() <= 0)
            return state.log_prior() = lp;
        if (model_ == StudentT && state.nu() <= 2)
            return state.log_prior() = lp;

        lp = theta_ub0_[0] - theta_lb0_[0];
        for (std::size_t d = 0; d != state.comp_num(); ++d)
            lp *= phi_ub0_[d] - phi_lb0_[d];
        for (std::size_t d = 1; d != state.comp_num(); ++d)
            lp *= theta_ub0_[d] - state.theta(d - 1);
        lp = -log(lp);
        lp += (lambda_a0_ - 1) * log(state.lambda())
            - state.lambda() / lambda_b0_;
        if (model_ == StudentT)
            lp -= 2 * log(state.nu());

        return state.log_prior() = lp;
    }

    double log_likelihood (pet_param &state) const
    {
        double ll = model_ == Normal ?
            log_likelihood_normal(state):
            log_likelihood_studentt(state);

        return state.log_likelihood() = ll;
    }

    double log_target (pet_param &state, bool compute = true) const
    {
        double lt = compute ?
            log_prior(state) + state.alpha() * log_likelihood(state):
            state.log_prior() + state.alpha() * state.log_likelihood();

        return state.log_target() = lt;
    }

    void read_data (const pet_info *info)
    {
        // read data
        if (info->read_data) {
            obs_.resize(info->data->data_num);
            for (std::size_t i = 0; i != info->data->data_num; ++i)
                obs_[i] = info->data->data_value[i];
        }

        // read time
        if (info->read_time) {
            if (info->time->time_num != info->data->data_num)
                throw std::runtime_error("Time steps not equal data number");
            time_.resize(info->time->time_num);
            for (std::size_t i = 0; i != info->time->time_num; ++i)
                time_[i] = info->time->time_value[i];
        }

        // read convolution matrix
        if (info->read_conv) {
            conv_mul_ = info->conv->conv_mul;
            conv_.resize(info->conv->conv_num * info->time->time_num);
            for (std::size_t r = 0; r != info->conv->conv_num; ++r)
                for (std::size_t c = 0; c != info->time->time_num; ++c)
                    conv_elem(r, c) = info->conv->conv_value[
                        r * info->time->time_num + c];
        }

        // read priors
        if (info->read_prior) {
            std::size_t n = info->prior->prior_num;
            phi_lb0_.resize(n); theta_lb0_.resize(n);
            phi_ub0_.resize(n); theta_ub0_.resize(n);

            const double *v = info->prior->prior_value;
            std::size_t offset = 0;
            for (std::size_t i = 0; i != n; ++i) phi_lb0_[i] = v[offset++];
            for (std::size_t i = 0; i != n; ++i) phi_ub0_[i] = v[offset++];
            for (std::size_t i = 0; i != n; ++i) theta_lb0_[i] = v[offset++];
            for (std::size_t i = 0; i != n; ++i) theta_ub0_[i] = v[offset++];
            lambda_a0_ = v[offset++];
            lambda_b0_ = v[offset++];
            nu_a0_ = v[offset++];
            nu_b0_ = v[offset++];
        }

        // read sds
        if (info->read_sd) {
            std::size_t n = info->sd->sd_num;
            phi_sd_.resize(n);
            theta_sd_.resize(n);

            const double *v = info->sd->sd_value;
            std::size_t offset = 0;
            for (std::size_t i = 0; i != n; ++i) phi_sd_[i] = v[offset++];
            for (std::size_t i = 0; i != n; ++i) theta_sd_[i] = v[offset++];
            lambda_sd_ = v[offset++];
            nu_sd_ = v[offset++];
        }

        // read model
        if (info->read_model) {
            for (std::size_t i = 0; i != theta_lb0_.size(); ++i)
                theta_lb0_[i] += info->model->decay - decay_;
            decay_ = info->model->decay;
            model_ = info->model->model;
        }
    }

    void alpha (double a)
    {
        a = a < 1 ? a : 1;
        a = a > 0 ? a : 0;
        double a_inc = a > 0 ? a - this->state(0, 0).alpha() : 0;
        for (size_type i = 0; i != this->size(); ++i) {
            this->state(i, 0).alpha() = a;
            this->state(i, 0).alpha_inc() = a_inc;
        }
    }

    void alpha_inc (double a_inc)
    {
        for (size_type i = 0; i != this->size(); ++i)
            this->state(i, 0).alpha_inc() = a_inc;
    }

    void comp_num (std::size_t num)
    {
        for (size_type i = 0; i != this->size(); ++i)
            this->state(i, 0).comp_num(num);
    }

    PETModel model       ()              const {return model_;}
    double decay         ()              const {return decay_;}
    std::size_t data_num ()              const {return obs_.size();}
    double phi_lb0       (std::size_t d) const {return phi_lb0_[d];}
    double phi_ub0       (std::size_t d) const {return phi_ub0_[d];}
    double theta_lb0     (std::size_t d) const {return theta_lb0_[d];}
    double theta_ub0     (std::size_t d) const {return theta_ub0_[d];}
    double lambda_a0     ()              const {return lambda_a0_;}
    double lambda_b0     ()              const {return lambda_b0_;}
    double nu_a0         ()              const {return nu_a0_;}
    double nu_b0         ()              const {return nu_b0_;}
    double phi_sd        (std::size_t d) const {return phi_sd_[d];}
    double theta_sd      (std::size_t d) const {return theta_sd_[d];}
    double lambda_sd     ()              const {return lambda_sd_;}
    double nu_sd         ()              const {return nu_sd_;}
    double zconst        ()              const {return zconst_;}
    double &zconst       ()                    {return zconst_;}

    private :

    PETModel model_;
    double decay_;
    double conv_mul_;
    double lambda_a0_; // shape of lambda
    double lambda_b0_; // scale of lambda
    double nu_a0_;     // lower bound of 1 / nu
    double nu_b0_;     // upper bound of 1 / nu
    double lambda_sd_;
    double nu_sd_;
    double zconst_;

    std::vector<double> phi_lb0_;
    std::vector<double> phi_ub0_;
    std::vector<double> theta_lb0_;
    std::vector<double> theta_ub0_;
    std::vector<double> phi_sd_;
    std::vector<double> theta_sd_;
    std::vector<double> obs_;
    std::vector<double> time_;
    std::vector<double> conv_;

    double conv_elem (size_type r, size_type c) const
    {
        return conv_[r * time_.size() + c];
    }

    double &conv_elem (size_type r, size_type c)
    {
        return conv_[r * time_.size() + c];
    }

    double log_likelihood_const_normal () const
    {return -0.5 * data_num() * vsmc::math::ln_pi_2<double>();}

    double log_likelihood_const_studentt () const
    {return -0.5 * data_num() * vsmc::math::ln_pi<double>();}

    double log_likelihood_normal (pet_param &state) const
    {
        using std::log;

        double ll = 0.5 * data_num() * log(state.lambda());
        for (std::size_t i = 0; i != data_num(); ++i) {
            double fv = fit_value(i, state);
            double resid = obs_[i] - fv;
            fv = time_[i] / fv;
            ll += 0.5 * (log(fv) - state.lambda() * fv * resid * resid);
        }

        return ll;
    }

    double log_likelihood_studentt (pet_param &state) const
    {
        using std::log;

        double ll = data_num() * (
                vsmc::cxx11::lgamma(0.5 * (state.nu() + 1)) -
                vsmc::cxx11::lgamma(0.5 * state.nu()) +
                0.5 * log(state.lambda()) - 0.5 * log(state.nu()));
        for (std::size_t i = 0; i != data_num(); ++i) {
            double fv = fit_value(i, state);
            double resid = obs_[i] - fv;
            fv = time_[i] / fv;
            ll += 0.5 * log(fv) - 0.5 * (state.nu() + 1) * log(
                    1 + fv * state.lambda() / state.nu() * resid * resid);
        }

        return ll;
    }

    double fit_value (std::size_t i, const pet_param &state) const
    {
        double fv = 0;

        for (std::size_t d = 0; d != state.comp_num(); ++d) {
            double phi_cur = state.phi(d);
            double theta_cur = state.theta(d) * conv_mul_;
            std::size_t theta_low = static_cast<std::size_t>(theta_cur);
            std::size_t theta_upp = theta_low + 1;
            double low_multi = phi_cur * (theta_cur - theta_low);
            double upp_multi = phi_cur * (theta_upp - theta_cur);
            fv += low_multi * conv_elem(theta_upp, i);
            fv += upp_multi * conv_elem(theta_low, i);
        }

        return std::abs(fv);
    }
};

#endif // VSMC_EXAMPLE_PET_STATE_HPP
