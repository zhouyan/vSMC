//============================================================================
// vSMC/example/include/move_alpha.hpp
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

#ifndef VSMC_EXAMPLE_MOVE_ALPHA_HPP
#define VSMC_EXAMPLE_MOVE_ALPHA_HPP

template <typename T> class alpha_null
{
    public:
    typedef double value_type;
    alpha_null() {}
    alpha_null(value_type) {}
    void alpha_iter(std::size_t, vsmc::Particle<T> &) {}
    void alpha_init(vsmc::Particle<T> &) {}
};

template <typename T> class alpha_fixed
{
    public:
    typedef std::vector<double> *value_type;

    alpha_fixed(value_type config = 0) : alpha_(*config) {}

    void alpha_iter(std::size_t iter, vsmc::Particle<T> &particle)
    {
        if (iter < alpha_.size())
            particle.value().alpha(alpha_[iter]);
    }

    void alpha_init(vsmc::Particle<T> &particle)
    {
        assert(static_cast<typename vsmc::Particle<T>::size_type>(
                   alpha_.size()) >= particle.size());
        const std::size_t nchain = static_cast<std::size_t>(particle.size());
        for (std::size_t i = 0; i != nchain; ++i)
            particle.value().state(i, 0).alpha() = alpha_[i];
    }

    private:
    std::vector<double> alpha_;
};

template <typename T> class alpha_mh
{
    public:
    typedef double value_type;

    alpha_mh(value_type alpha = 1) : alpha_(alpha) {}

    void alpha_iter(std::size_t, vsmc::Particle<T> &particle)
    {
        particle.value().alpha(alpha_);
    }

    void alpha_init(vsmc::Particle<T> &particle)
    {
        particle.value().alpha(alpha_);
    }

    private:
    double alpha_;
};

template <typename T> class alpha_linear
{
    public:
    typedef std::size_t value_type;

    alpha_linear(value_type iter_num = 100) : iter_num_(iter_num) {}

    void alpha_iter(std::size_t iter, vsmc::Particle<T> &particle)
    {
        double a = static_cast<double>(iter) / iter_num_;
        particle.value().alpha(a);
    }

    void alpha_init(vsmc::Particle<T> &particle)
    {
        const std::size_t nchain = static_cast<std::size_t>(particle.size());
        for (std::size_t i = 0; i != nchain; ++i)
            particle.value().state(i, 0).alpha() = i / (nchain - 1.0);
    }

    private:
    std::size_t iter_num_;
};

template <typename T, std::size_t Power> class alpha_prior
{
    public:
    typedef std::size_t value_type;

    alpha_prior(value_type iter_num = 100) : iter_num_(iter_num) {}

    void alpha_iter(std::size_t iter, vsmc::Particle<T> &particle)
    {
        double b = static_cast<double>(iter) / iter_num_;
        double a = 1;
        for (std::size_t p = 0; p != Power; ++p)
            a *= b;
        particle.value().alpha(a);
    }

    void alpha_init(vsmc::Particle<T> &particle)
    {
        const std::size_t nchain = static_cast<std::size_t>(particle.size());
        for (std::size_t i = 0; i != nchain; ++i) {
            double b = i / (nchain - 1.0);
            double a = 1;
            for (std::size_t p = 0; p != Power; ++p)
                a *= b;
            particle.value().state(i, 0).alpha() = a;
        }
    }

    private:
    std::size_t iter_num_;
};

template <typename T, std::size_t Power> class alpha_posterior
{
    public:
    typedef std::size_t value_type;

    alpha_posterior(value_type iter_num = 100) : iter_num_(iter_num) {}

    void alpha_iter(std::size_t iter, vsmc::Particle<T> &particle)
    {
        double b = static_cast<double>(iter_num_ - iter) / iter_num_;
        double a = 1;
        for (std::size_t p = 0; p != Power; ++p)
            a *= b;
        particle.value().alpha(1 - a);
    }

    void alpha_init(vsmc::Particle<T> &particle)
    {
        const std::size_t nchain = static_cast<std::size_t>(particle.size());
        for (std::size_t i = 0; i != nchain; ++i) {
            double b = (nchain - 1.0 - i) / (nchain - 1.0);
            double a = 1;
            for (std::size_t p = 0; p != Power; ++p)
                a *= b;
            particle.value().state(i, 0).alpha() = 1 - a;
        }
    }

    private:
    std::size_t iter_num_;
};

template <typename T> class ess01
{
    public:
    double operator()(const vsmc::Particle<T> &particle,
                      const double *mul_weight) const
    {
        return particle.weight_set().ess(mul_weight, false);
    }
};

template <typename T> class cess01
{
    public:
    double operator()(const vsmc::Particle<T> &particle,
                      const double *mul_weight) const
    {
        return particle.weight_set().cess(mul_weight, false);
    }
};

template <typename T, typename ESS> class ess_new;

template <typename T> class ess_new<T, ess01<T>>
{
    public:
    double operator()(const vsmc::Particle<T> &particle, double drop) const
    {
        return (1 - drop) * particle.weight_set().ess();
    }
};

template <typename T> class ess_new<T, cess01<T>>
{
    public:
    double operator()(const vsmc::Particle<T> &, double drop) const
    {
        return 1 - drop;
    }
};

template <typename T, typename ESS> class alpha_ess
{
    public:
    typedef double value_type;

    alpha_ess(value_type ess_drop = 0.1) : ess_drop_(ess_drop) {}

    void alpha_iter(std::size_t, vsmc::Particle<T> &particle)
    {
        using std::exp;

        log_likelihood_.resize(particle.size());
        mul_weight_.resize(particle.size());
        for (typename vsmc::Particle<T>::size_type i = 0;
             i != particle.size();
             ++i)
            log_likelihood_[i] =
                particle.value().state(i, 0).log_likelihood();

        double ess_new = ess_new_(particle, ess_drop_);

        double e = 1e-10 > 1e-3 * ess_drop_ ? 1e-10 : 1e-3 * ess_drop_;
        double l = particle.value().state(0, 0).alpha();
        double u = 1.05;
        double a = l + 0.05;
        double a_old = l;
        double ess = 0;
        while (std::abs(u - l) > e && l <= 1) {
            double a_diff = a - a_old;
            for (typename vsmc::Particle<T>::size_type i = 0;
                 i != particle.size();
                 ++i) {
                mul_weight_[i] = a_diff * log_likelihood_[i];
            }
            for (typename vsmc::Particle<T>::size_type i = 0;
                 i != particle.size();
                 ++i)
                mul_weight_[i] = exp(mul_weight_[i]);
            ess = ess_(particle, &mul_weight_[0]);
            if (ess < ess_new) {
                u = a;
                a = 0.5 * (a + l);
            } else {
                l = a;
                a = 0.5 * (a + u);
            }
        }
        particle.value().alpha(a);
    }

    private:
    std::vector<double> log_likelihood_;
    std::vector<double> mul_weight_;
    ESS ess_;
    ess_new<T, ESS> ess_new_;
    double ess_drop_;
};

#endif  // VSMC_EXAMPLE_MOVE_ALPHA_HPP
