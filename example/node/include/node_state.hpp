//============================================================================
// vSMC/example/node/include/node_state.hpp
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

#ifndef VSMC_EXAMPLE_NODE_STATE_HPP
#define VSMC_EXAMPLE_NODE_STATE_HPP

struct data_info
{
    const std::size_t data_num;
    double resolution;
    const char *file_name;
    const double *data_value;

    data_info (std::size_t num, double r, const char *file) :
        data_num(num), resolution(r),
        file_name(file), data_value(VSMC_NULLPTR) {}

    data_info (std::size_t num, double r, const double *data) :
        data_num(num), resolution(r),
        file_name(VSMC_NULLPTR), data_value(data) {}
};

class node_state :
    public BASE_STATE<vsmc::StateMatrix<vsmc::RowMajor, 1, node_param> >
{
    public :

    node_state (size_type N) :
        BASE_STATE<vsmc::StateMatrix<vsmc::RowMajor, 1, node_param> >(N) {}

    double log_likelihood_const () const
    {
        using std::log;

        return -(data_num() * (vsmc::math::ln_2pi<double>() + 2 * log(sd0_)));
    }

    double log_prior (node_param &state) const
    {
        using std::log;

        double lp = 0;
        double coeff = shape0_ - 1;
        double rate = 1 / scale0_;
        lp += coeff * log(state.a0()) - rate * state.a0();
        lp += coeff * log(state.a1()) - rate * state.a1();
        lp += coeff * log(state.a2()) - rate * state.a2();
        std::size_t cn = state.comp_num();
        for (std::size_t d = 0; d != cn - 1; ++d)
            lp += coeff * log(state.k(d)) - rate * state.k(d);

        return state.log_prior() = lp;
    }

    double log_likelihood (node_param &state) const
    {
        using std::log;

        double ll = 0;
        state.update_model();
        for (std::size_t i = 0; i != data_num(); ++i) {
            state.fit_model(time_[i]);
            double resid1 = obs1_[i] - state.fit(0);
            double resid2 = obs2_[i] - state.fit(1);
            ll += resid1 * resid1 + resid2 * resid2;
        }
        ll *= -0.5 / (sd0_ * sd0_);

        return state.log_likelihood() = ll;
    }

    double log_target (node_param &state) const
    {
        double lt =
            log_prior(state) + state.alpha() * log_likelihood(state);

        return state.log_target() = lt;
    }

    void read_data (const data_info *info)
    {
        obs1_.resize(info->data_num);
        obs2_.resize(info->data_num);
        time_.resize(info->data_num);

        resolution_ = info->resolution;

        if (info->data_value) {
            for (std::size_t i = 0; i != info->data_num; ++i) {
                time_[i] = info->data_value[i * 3 + 0];
                obs1_[i] = info->data_value[i * 3 + 1];
                obs2_[i] = info->data_value[i * 3 + 2];
            }
        } else if (info->file_name) {
            std::ifstream data;
            data.open(info->file_name);
            if (!data) {
                data.close();
                data.clear();
                throw std::runtime_error("Failed to open data file");
            }
            for (std::size_t i = 0; i != info->data_num; ++i)
                data >> time_[i] >> obs1_[i] >> obs2_[i];
        } else {
            throw std::runtime_error("Failed to red data");
        }

        shape0_ = Shape0;
        scale0_ = Scale0;
        sd0_ = 0.2;
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

    std::size_t data_num () const {return obs1_.size();}
    double shape0        () const {return shape0_;}
    double scale0        () const {return scale0_;}
    double sd0           () const {return sd0_;}
    double zconst        () const {return zconst_;}
    double &zconst       ()       {return zconst_;}

    private :

    double shape0_; // prior shape
    double scale0_; // prior scale
    double sd0_;    // likelihood sd
    double resolution_;
    double zconst_;
    std::vector<double> obs1_;
    std::vector<double> obs2_;
    std::vector<double> time_;
};

#endif // VSMC_EXAMPLE_NODE_STATE_HPP
