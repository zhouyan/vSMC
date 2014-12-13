//============================================================================
// vSMC/example/gmm/include/gmm_state.hpp
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

#ifndef VSMC_EXAMPLE_GMM_STATE_HPP
#define VSMC_EXAMPLE_GMM_STATE_HPP

struct data_info
{
    const std::size_t data_num;
    const char *file_name;
    const double *data_value;

    data_info (std::size_t num, const char *file) :
        data_num(num), file_name(file), data_value(VSMC_NULLPTR) {}

    data_info (std::size_t num, const double *data) :
        data_num(num), file_name(VSMC_NULLPTR), data_value(data) {}
};

class gmm_state :
    public BASE_STATE<vsmc::StateMatrix<vsmc::RowMajor, 1, gmm_param> >
{
    public :

    gmm_state (size_type N) :
        BASE_STATE<vsmc::StateMatrix<vsmc::RowMajor, 1, gmm_param> >(N),
        ordered_(false) {}

    double log_prior_const (std::size_t comp_num) const
    {
        using std::log;

        double lp = 0;
        lp -= comp_num * log(sd0_);
        lp -= 0.5 * comp_num * vsmc::math::ln_pi_2<double>();
        lp -= comp_num * (shape0_ * log(scale0_) +
                vsmc::cxx11::lgamma(shape0_));

        return lp;
    }

    double log_likelihood_const () const
    {return -0.5 * obs_.size() * vsmc::math::ln_pi_2<double>();}

    double log_prior (gmm_param &state) const
    {
        using std::log;

        double lp = 0;
        for (std::size_t d = 0; d != state.comp_num(); ++d) {
            double resid = state.mu(d) - mu0_;
            lp += -0.5 * (resid * resid) / (sd0_ * sd0_);
            lp += (shape0_ - 1) * log(state.lambda(d)) -
                state.lambda(d) / scale0_;
        }

        return state.log_prior() = lp;
    }

    double log_likelihood (gmm_param &state) const
    {
        using std::exp;
        using std::log;
        using std::sqrt;

        state.log_lambda();

        double ll = 0;
        for (std::size_t i = 0; i != obs_.size(); ++i) {
            double lli = 0;
            for (std::size_t d = 0; d != state.comp_num(); ++d) {
                double resid = obs_[i] - state.mu(d);
                lli += state.weight(d) * exp(
                        0.5 * state.log_lambda(d) -
                        0.5 * state.lambda(d) * resid * resid);
            }
            ll += log(lli + 1e-13); // lli can be numerically zero!
        }

        return state.log_likelihood() = ll;
    }

    double log_target (gmm_param &state) const
    {
        double lt = log_prior(state) + state.alpha() * log_likelihood(state);

        return state.log_target() = lt;
    }

    void read_data (const data_info *info)
    {
        using std::sqrt;

        obs_.resize(info->data_num);

        if (info->data_value) {
            // read data from c-array
            for (std::size_t i = 0; i != info->data_num; ++i)
                obs_[i] = info->data_value[i];
        } else if (info->file_name) {
            // read data frome file
            std::ifstream data;
            data.open(info->file_name);
            if (!data) {
                data.close();
                data.clear();
                throw std::runtime_error("Failed to open data file");
            }
            for (std::size_t i = 0; i != info->data_num; ++i)
                data >> obs_[i];
            data.close();
            data.clear();
        } else {
            throw std::runtime_error("Failed to read data");
        }

        // find min and max of data
        double xmax = obs_[0];
        double xmin = obs_[0];
        for (std::size_t i = 0; i != obs_.size(); ++i) {
            if (obs_[i] > xmax)
                xmax = obs_[i];
            if (obs_[i] < xmin)
                xmin = obs_[i];
        }
        double kappa = 1 / ((xmax - xmin) * (xmax - xmin));

        // store them for future use
        mu0_    = 0.5 * (xmin + xmax);
        sd0_    = 1 / sqrt(kappa);
        shape0_ = 2;
        scale0_ = 50 * kappa;
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

    void beta (double a)
    {
        a = a < 1 ? a : 1;
        a = a > 0 ? a : 0;
        double a_inc = a > 0 ? a - this->state(0, 0).beta() : 0;
        for (size_type i = 0; i != this->size(); ++i) {
            this->state(i, 0).beta() = a;
            this->state(i, 0).beta_inc() = a_inc;
        }
    }

    void beta_inc (double a_inc)
    {
        for (size_type i = 0; i != this->size(); ++i)
            this->state(i, 0).beta_inc() = a_inc;
    }

    void comp_num (std::size_t num)
    {
        for (size_type i = 0; i != this->size(); ++i)
            this->state(i, 0).comp_num(num);
    }

    bool ordered         () const {return ordered_;}
    bool &ordered        ()       {return ordered_;}
    std::size_t data_num () const {return obs_.size();}
    double mu0           () const {return mu0_;}
    double sd0           () const {return sd0_;}
    double shape0        () const {return shape0_;}
    double scale0        () const {return scale0_;}
    double zconst        () const {return zconst_;}
    double &zconst       ()       {return zconst_;}

    private :

    bool ordered_;
    double mu0_;    // prior mean for mu
    double sd0_;    // prior sd for mu
    double shape0_; // prior shape for lambda
    double scale0_; // prior scale for lambda
    double zconst_;
    std::vector<double> obs_;
};

#endif // VSMC_EXAMPLE_GMM_STATE_HPP
