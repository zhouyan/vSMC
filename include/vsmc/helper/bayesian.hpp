#ifndef VSMC_HELPER_BAYESIAN_HPP
#define VSMC_HELPER_BAYESIAN_HPP

namespace vsmc {

template <unsigned ID>
struct BayesianParamID
{
    enum {value = ID};
}; // class ParamID

class BayesianParam
{
    public :

    BayesianParam () :
        log_prior_(0), log_likelihood_(0), log_target_(0),
        log_prior_old_(0), log_likelihood_old_(0), log_target_old_(0) {}

    double &param (unsigned d)
    {
        return param_[d];
    }

    double param (unsigned d) const
    {
        return param_[d];
    }

    template <typename Param>
    double &param ()
    {
        return param_[Param::value];
    }

    template <typename Param>
    double param () const
    {
        return param_[Param::value];
    }

    double &log_prior ()
    {
        return log_prior_;
    }

    double log_prior () const
    {
        return log_prior_;
    }

    double &log_likelihood ()
    {
        return log_likelihood_;
    }

    double log_likelihood () const
    {
        return log_likelihood_;
    }

    double &log_target ()
    {
        return log_target_;
    }

    double log_target () const
    {
        return log_target_;
    }

    double log_target_diff () const
    {
        return log_target_ - log_target_old_;
    }

    void param_num (unsigned num)
    {
        param_.resize(num);
        param_old_.resize(num);
    }

    unsigned param_num () const
    {
        return param_.size();
    }

    template <typename InputIter>
    void mh_save (InputIter first, InputIter last)
    {
        log_prior_old_       = log_prior_;
        log_likelihood_old_  = log_likelihood_;
        log_target_old_      = log_target_;
        for (InputIter i = first; i != last; ++i) {
            unsigned d = *i;
            param_old_[d] = param_[d];
        }
    }

    template <typename InputIter>
    unsigned mh_reject (double p, double u, InputIter first, InputIter last)
    {
        if (p < u) {
            log_prior_       = log_prior_old_;
            log_likelihood_  = log_likelihood_old_;
            log_target_      = log_target_old_;
            for (InputIter i = first; i != last; ++i) {
                unsigned d = *i;
                param_[d] = param_old_[d];
            }
            return 0;
        }

        return 1;
    }

    private :

    Eigen::VectorXd param_;
    double log_prior_;
    double log_likelihood_;
    double log_target_;

    Eigen::VectorXd param_old_;
    double log_prior_old_;
    double log_likelihood_old_;
    double log_target_old_;
}; // class BayesianParam

template <typename T, template <unsigned, typename, typename> class State,
         typename Timer>
class BayesianState : public State<1, T, Timer>
{
    public :

    typedef State<1, T, Timer> state_base_type;
    typedef typename state_base_type::size_type size_type;
    typedef typename state_base_type::state_type state_type;
    typedef internal::function<double (const state_type &)>
        log_prior_eval_type;
    typedef internal::function<double (const state_type &)>
        log_likelihood_eval_type;

    explicit BayesianState (size_type N) : state_base_type(N) {}

    /// Set new alpha, change both alpha and alpha_inc
    void alpha (double a)
    {
        a = std::max(a, 0);
        a = std::min(a, 1);
        alpha_inc_ = a > 0 ? a - alpha_ : 0;
        alpha_ = a;
    }

    double alpha () const
    {
        return alpha_;
    }

    double alpha_inc () const
    {
        return alpha_inc_;
    }

    double &scale (unsigned d)
    {
        return scale_[d];
    }

    double scale (unsigned d) const
    {
        return scale_[d];
    }

    template <typename Scale>
    double &scale ()
    {
        return scale_[Scale::value];
    }

    template <typename Scale>
    double scale () const
    {
        return scale_[Scale::value];
    }

    void param_num (unsigned num)
    {
        for (size_type i = 0; i != this->size(); ++i)
            this->state(i, 0).param_num(num);
        scale_.resize(num);
    }

    unsigned param_num () const
    {
        return this->state(0, 0).param_num();
    }

    void log_prior (const log_prior_eval_type &eval)
    {
        log_prior_eval_ = eval;
    }

    void log_likelihood (const log_likelihood_eval_type &eval)
    {
        log_likelihood_eval_ = eval;
    }

    double log_prior (state_type &param) const
    {
        return param.log_prior() = log_prior_eval_(param);
    }

    double log_likelihood (state_type &param) const
    {
        return param.log_likelihood() = log_likelihood_eval_(param);
    }

    double log_target (state_type &param) const
    {
        return param.log_target() =
            log_prior(param) + alpha() * log_likelihood(param);
    }

    private :

    log_prior_eval_type log_prior_eval_;
    log_likelihood_eval_type log_likelihood_eval_;

    double alpha_;
    double alpha_inc_;
    Eigen::VectorXd scale_;
}; // class BayesianState

template <typename T,
         template <typename> class Alpha, template <typename> class Scale>
class BayesianSMCMove
{
    public :

    typedef internal::function<void (unsigned, Particle<T> &)> config_type;

    typedef typename Alpha<T>::value_type alpha_value_type;
    typedef typename Scale<T>::value_type scale_value_type;

    explicit BayesianSMCMove (
            const alpha_value_type &alpha_config,
            const scale_value_type &scale_config) :
        alpha_(alpha_config), scale_(scale_config) {}

    unsigned operator() (unsigned iter, vsmc::Particle<T> &particle)
    {
        alpha_(iter, particle);
        scale_(iter, particle);

        llh_.resize(particle.size());
        for (typename vsmc::Particle<T>::size_type i = 0;
                i != particle.size(); ++i) {
            llh_[i] = particle.value().state(i, 0).log_likelihood();
        }
        particle.add_log_weight(llh_, particle.value().alpha_inc());

        return 0;
    }

    private :

    Alpha<T> alpha_;
    Scale<T> scale_;
    Eigen::VectorXd llh_;
};

template <typename T, template <typename, typename> class PathEval>
class BayesianPathEval : public PathEval<T, BayesianPathEval<T, PathEval> >
{
    public :

    double path_state (unsigned iter, vsmc::ConstSingleParticle<T> part)
    {
        return part.state(0).log_likelihood();
    }

    double path_width (unsigned iter, const vsmc::Particle<T> &particle)
    {
        return particle.value().alpha_inc();
    }
}; // class BayesianPath

} // namespace vsmc

#endif // VSMC_HELPER_BAYESIAN_HPP
