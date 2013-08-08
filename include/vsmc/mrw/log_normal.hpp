#ifndef VSMC_MRW_LOG_NORMAL_HPP
#define VSMC_MRW_LOG_NORMAL_HPP

#include <vsmc/mrw/base.hpp>

namespace vsmc {

/// \brief Metropolis random walk with a Normal kernel on log scale
/// \ingroup MRW
class LogNormalRW : public BaseRW
{
    public :

    LogNormalRW () : rnorm_(0, 1) {}

    template <typename URNG, typename Func>
    bool operator() (double &val, double sd, URNG &eng, const Func &func,
            double lb = 0)
    {
        using std::exp;
        using std::log;

        double new_val = exp(log(val) + rnorm_(eng) * sd);
        if (new_val <= lb)
            return false;

        double log_prob = func(new_val) - func(val);
        log_prob += log(new_val) - log(val);

        return update(val, new_val, log_prob, eng);
    }

    template <typename URNG, typename Func>
    bool operator() (std::size_t dim, double *val, double sd,
            URNG &eng, const Func &func, double lb = 0)
    {
        using std::exp;
        using std::log;

        new_val_vec_.resize(dim);
        for (std::size_t i = 0; i != dim; ++i) {
            new_val_vec_[i] = exp(log(val[i]) + rnorm_(eng) * sd);
            if (new_val_vec_[i] <= lb)
                return false;
        }

        const double *const new_val = &new_val_vec_[0];
        double log_prob = func(dim, new_val) - func(dim, val);
        for (std::size_t i = 0; i != dim; ++i)
            log_prob += log(new_val[i]) - log(val[i]);

        return update(dim, val, new_val, log_prob, eng);
    }

    template <typename URNG, typename Func>
    bool operator() (std::size_t dim, double *val, const double *sd,
            URNG &eng, const Func &func, double lb = 0)
    {
        using std::exp;
        using std::log;

        new_val_vec_.resize(dim);
        for (std::size_t i = 0; i != dim; ++i) {
            new_val_vec_[i] = exp(log(val[i]) + rnorm_(eng) * sd[i]);
            if (new_val_vec_[i] <= lb)
                return false;
        }

        const double *const new_val = &new_val_vec_[0];
        double log_prob = func(dim, new_val) - func(dim, val);
        for (std::size_t i = 0; i != dim; ++i)
            log_prob += log(new_val[i]) - log(val[i]);

        return update(dim, val, new_val, log_prob, eng);
    }

    private :

    cxx11::normal_distribution<double> rnorm_;
    std::vector<double> new_val_vec_;
}; // class LogNormalRW

} // namespace vsmc

#endif // VSMC_MRW_LOG_NORMAL_HPP
