#ifndef VSMC_HELPER_BAYESIAN_RANDOM_WALK_HPP
#define VSMC_HELPER_BAYESIAN_RANDOM_WALK_HPP

namespace vsmc { namespace rwm {

template <typename T, template <typename, typename> class Move>
class Normal : public Move<T, Normal<T, Move> >
{
    public :

    template <typename InputIter>
    Normal (InputIter first, InputIter last) : index_(first, last) {}

    unsigned move_state (unsigned iter, SingleParticle<T> sp)
    {
        using std::log;

        sp.state(0).save_old(index_.begin(); index_.end());

        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end(); ++d) {
            rng::normal_distribution<double> r(0, sp.state(0).scale(*d));
            sp.state(0).param(*d) += r(sp.rng());
        }

        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff();

        vsmc::rng::uniform_real_distribution<double> runif(0, 1);
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject(p, u, index_.begin(), index_.end());
    }

    private :

    std::vector<unsigned> index_;
}; // class Normal

template <typename T, template <typename, typename> class Move>
class LogNormal : public Move<T, LogNormal<T, Move> >
{
    public :

    template <typename InputIter>
    LogNormal (InputIter first, InputIter last) : index_(first, last) {}

    unsigned move_state (unsigned iter, SingleParticle<T> sp)
    {
        using std::log;


        sp.state(0).save_old(index_.begin(); index_.end());

        double log_diff = 0;
        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end(); ++d) {
            log_diff -= log(sp.state(0).param(*d));
            rng::log_normal_distribution<double> r(0, sp.state(0).scale(*d));
            sp.state(0).param(*d) *= rnorm(sp.rng());
            log_diff += log(sp.state(0).param(*d));
        }

        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() + log_diff;

        vsmc::rng::uniform_real_distribution<double> runif(0, 1);
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject(p, u, index_.begin(), index_.end());
    }

    private :

    std::vector<unsigned> index_;
}; // class LogNormal

template <typename T, template <typename, typename> class Move>
class LogitNormal : public Move<T, LogitNormal<T, Move> >
{
    public :

    template <typename InputIter>
    LogitNormal (InputIter first, InputIter last) : index_(first, last) {}

    unsigned move_state (unsigned iter, SingleParticle<T> sp)
    {
        using std::log;
        using std::exp;

        sp.state(0).save_old(index_.begin(); index_.end());

        double logit_sw = 1;
        double logit_slw = 0;
        double logit_sw_old = 1;
        double logit_slw_old = 0;
        double sw = 1;
        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end() - 1; ++d) {
            double w = sp.state(0).param(*d) / sp.state(0).param(index.back());
            double lw = log(w);

            logit_sw_old += w;
            logit_slw_old += lw;
            rng::normal_distribution<double> r(0, sp.state(0).scale(*d));
            lw += r(sp.rng());
            w = exp(lw);
            logit_sw += w;
            logit_slw += lw;

            sp.state(0).param(*d) = w;
            sw += w;
        }
        logit_slw     -= index_.size() * log(logit_sw);
        logit_slw_old -= index_.size() * log(logit_sw_old);
        double logit_diff = logit_slw - logit_slw_old;
        double sw_inv = 1 / sw;
        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end() - 1; ++d) {
            sp.state(0).param(*d) *= sw_inv;
        }
        sp.state(0).param(index_.back()) = sw_inv;

        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() + logit_diff;

        vsmc::rng::uniform_real_distribution<double> runif(0, 1);
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject(p, u, index_.begin(), index_.end());
    }

    private :

    std::vector<unsigned> index_;
}; // class LogitNormal

} } // namespace vsmc::rwm

#endif // VSMC_HELPER_BAYESIAN_RANDOM_WALK_HPP
