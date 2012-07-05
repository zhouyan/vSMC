#ifndef VSMC_HELPER_BAYESIAN_RANDOM_WALK_HPP
#define VSMC_HELPER_BAYESIAN_RANDOM_WALK_HPP

namespace vsmc { namespace rwm {

template <typename T>
class Normal
{
    public :

    template <typename InputIter>
    Normal (InputIter first, InputIter last) : index_(first, last) {}

    unsigned operator () (unsigned iter, SingleParticle<T> sp)
    {
        using std::log;

        sp.state(0).save_old(index_.begin(); index_.end());

        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end(); ++d)
        {
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

template <typename T>
class LogNormal
{
    public :

    template <typename InputIter>
    LogNormal (InputIter first, InputIter last) : index_(first, last) {}

    unsigned operator () (unsigned iter, SingleParticle<T> sp)
    {
        using std::log;

        double log_diff = 0;
        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end(); ++d)
            log_diff -= log(sp.state(0).param(*d));

        sp.state(0).save_old(index_.begin(); index_.end());

        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end(); ++d)
        {
            rng::log_normal_distribution<double> r(0, sp.state(0).scale(*d));
            sp.state(0).param(*d) *= rnorm(sp.rng());
        }

        for (std::vector<unsigned>::const_iterator d = index_.begin();
                d != index_.end(); ++d)
            log_diff += log(sp.state(0).param(*d));

        sp.particle().value().log_target(sp.state(0));
        double p = sp.state(0).log_target_diff() + log_diff;

        vsmc::rng::uniform_real_distribution<double> runif(0, 1);
        double u = log(runif(sp.rng()));

        return sp.state(0).mh_reject(p, u, index_.begin(), index_.end());
    }

    private :

    std::vector<unsigned> index_;
}; // class LogNormal

} }

#endif // VSMC_HELPER_BAYESIAN_RANDOM_WALK_HPP
