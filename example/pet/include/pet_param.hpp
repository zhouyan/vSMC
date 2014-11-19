//============================================================================
// vSMC/vSMCExample/pet/include/pet_param.hpp
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

#ifndef VSMC_EXAMPLE_PET_PARAM_HPP
#define VSMC_EXAMPLE_PET_PARAM_HPP

class pet_param
{
    public :

    pet_param () :
        comp_num_(0), lambda_(1), lambda_old_(1), lambda_sd_(1),
        nu_(3), nu_old_(3), nu_sd_(1),
        log_prior_(0), log_prior_old_(0), log_likelihood_(0),
        log_likelihood_old_(0), log_target_(0), log_target_old_(0),
        alpha_(0), alpha_inc_(0) {}

    // assignment only manipulate parameters and log likelihood etc.
    pet_param &operator= (const pet_param &other)
    {
        if (this != &other) {
            comp_num(other.comp_num_);
            for (std::size_t i = 0; i != comp_num_; ++i) {
                phi_[i] = other.phi_[i];
                theta_[i] = other.theta_[i];
            }
            lambda_         = other.lambda_;
            nu_             = other.nu_;
            log_prior_      = other.log_prior_;
            log_likelihood_ = other.log_likelihood_;
            log_target_     = other.log_target_;
        }

        return *this;
    }

    // Read only access
    double phi            (std::size_t d) const {return phi_[d];}
    double theta          (std::size_t d) const {return theta_[d];}
    double lambda         ()              const {return lambda_;}
    double nu             ()              const {return nu_;}
    double log_prior      ()              const {return log_prior_;}
    double log_likelihood ()              const {return log_likelihood_;}
    double log_target     ()              const {return log_target_;}
    double alpha          ()              const {return alpha_;}
    double alpha_inc      ()              const {return alpha_inc_;}
    double phi_sd         (std::size_t d) const {return phi_sd_[d];}
    double theta_sd       (std::size_t d) const {return theta_sd_[d];}
    double lambda_sd      ()              const {return lambda_sd_;}
    double nu_sd          ()              const {return nu_sd_;}

    // Read and write access
    double &phi            (std::size_t d) {return phi_[d];}
    double &theta          (std::size_t d) {return theta_[d];}
    double &lambda         ()              {return lambda_;}
    double &nu             ()              {return nu_;}
    double &log_prior      ()              {return log_prior_;}
    double &log_likelihood ()              {return log_likelihood_;}
    double &log_target     ()              {return log_target_;}
    double &alpha          ()              {return alpha_;}
    double &alpha_inc      ()              {return alpha_inc_;}
    double &phi_sd         (std::size_t d) {return phi_sd_[d];}
    double &theta_sd       (std::size_t d) {return theta_sd_[d];}
    double &lambda_sd      ()              {return lambda_sd_;}
    double &nu_sd          ()              {return nu_sd_;}

    std::size_t comp_num () const {return comp_num_;}

    // Minimize dynamic memory allocation
    // Not suitable if many particles will occaciaonally grow very large
    void comp_num (std::size_t num)
    {
        grow(num, phi_);
        grow(num, theta_);
        grow(num, phi_sd_);
        grow(num, theta_sd_);
        grow(num, phi_old_);
        grow(num, theta_old_);
        comp_num_ = num;
    }

    // swap is an expansive operation, only population MCMC shall use it
    void swap (pet_param &other)
    {
        using std::swap;

        if (this != &other) {
            swap(phi_,            other.phi_);
            swap(theta_,          other.theta_);
            swap(lambda_,         other.lambda_);
            swap(nu_,             other.nu_);
            swap(log_prior_,      other.log_prior_);
            swap(log_likelihood_, other.log_likelihood_);
            swap(log_target_,     other.log_target_);
            std::size_t cn = comp_num_;
            comp_num(other.comp_num_);
            other.comp_num(cn);
        }
    }

    void save_old ()
    {
        for (std::size_t i = 0; i != comp_num_; ++i) {
            phi_old_[i] = phi_[i];
            theta_old_[i] = theta_[i];
        }
        lambda_old_ = lambda_;
        nu_old_ = nu_;

        log_prior_old_      = log_prior_;
        log_likelihood_old_ = log_likelihood_;
        log_target_old_     = log_prior_ + alpha_ * log_likelihood_;
    }

    double log_target_diff () const {return log_target_ - log_target_old_;}

    double log_lambda_diff () const
    {return std::log(lambda_) - std::log(lambda_old_);}

    double log_nu_diff () const {return std::log(nu_) - std::log(nu_old_);}

    unsigned mh_reject_phi (double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            for (std::size_t i = 0; i != comp_num_; ++i)
                phi_[i] = phi_old_[i];
            return 0;
        } else {
            for (std::size_t i = 0; i != comp_num_; ++i)
                phi_old_[i] = phi_[i];
            return 1;
        }
    }

    unsigned mh_reject_theta (double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            for (std::size_t i = 0; i != comp_num_; ++i)
                theta_[i] = theta_old_[i];
            return 0;
        } else {
            for (std::size_t i = 0; i != comp_num_; ++i)
                theta_old_[i] = theta_[i];
            return 1;
        }
    }

    unsigned mh_reject_lambda (double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            lambda_ = lambda_old_;
            return 0;
        } else {
            lambda_old_ = lambda_;
            return 1;
        }
    }

    unsigned mh_reject_nu (double p, double u)
    {
        mh_reject_common(p, u);
        if (p < u) {
            nu_ = nu_old_;
            return 0;
        } else {
            nu_old_ = nu_;
            return 1;
        }
    }

    void mh_reject_common (double p, double u)
    {
        if (p < u) {
            log_prior_      = log_prior_old_;
            log_likelihood_ = log_likelihood_old_;
            log_target_     = log_target_old_;
        } else {
            log_prior_old_      = log_prior_;
            log_likelihood_old_ = log_likelihood_;
            log_target_old_     = log_target_;
        }
    }

    private :

    std::size_t comp_num_;

    double lambda_;
    double lambda_old_;
    double lambda_sd_;

    double nu_;
    double nu_old_;
    double nu_sd_;

    double log_prior_;
    double log_prior_old_;

    double log_likelihood_;
    double log_likelihood_old_;

    double log_target_;
    double log_target_old_;

    double alpha_;
    double alpha_inc_;


    std::vector<double> phi_;
    std::vector<double> phi_old_;
    std::vector<double> phi_sd_;

    std::vector<double> theta_;
    std::vector<double> theta_old_;
    std::vector<double> theta_sd_;
};

template <typename OutputStream>
inline OutputStream &operator<< (OutputStream &os, const pet_param &param)
{
    for (std::size_t d = 0; d != param.comp_num(); ++d)
        os << param.phi(d) << ' ' << param.theta(d) << ' ';
    os << param.lambda() << ' ' << param.nu() << ' ';
    os << param.log_prior() << ' ' << param.log_likelihood();

    return os;
}

#endif // VSMC_EXAMPLE_PET_PARAM_HPP
