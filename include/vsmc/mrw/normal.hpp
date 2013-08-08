#ifndef VSMC_MRW_NORMAL_HPP
#define VSMC_MRW_NORMAL_HPP

#include <vsmc/mrw/base.hpp>

namespace vsmc {

/// \brief Metropolis random walk with a Normal kernel
/// \ingroup MRW
class NormalRW : public BaseRW
{
    public :

    NormalRW () : rnorm_(0, 1) {}

    template <typename URNG, typename Func>
    bool operator() (double &val, double sd, URNG &eng, const Func &func)
    {
        double new_val = val + rnorm_(eng) * sd;
        double log_prob = func(new_val) - func(val);

        return update(val, new_val, log_prob, eng);
    }

    template <typename URNG, typename Func>
    bool operator() (std::size_t dim, double *val, double sd,
            URNG &eng, const Func &func)
    {
        new_val_vec_.resize(dim);
        for (std::size_t i = 0; i != dim; ++i)
            new_val_vec_[i] = val[i] + rnorm_(eng) * sd;
        const double *const new_val = &new_val_vec_[0];
        double log_prob = func(dim, new_val) - func(dim, val);

        return update(dim, val, new_val, log_prob, eng);
    }

    template <typename URNG, typename Func>
    bool operator() (std::size_t dim, double *val, const double *sd,
            URNG &eng, const Func &func)
    {
        new_val_vec_.resize(dim);
        for (std::size_t i = 0; i != dim; ++i)
            new_val_vec_[i] = val[i] + rnorm_(eng) * sd[i];
        const double *const new_val = &new_val_vec_[0];
        double log_prob = func(dim, new_val) - func(dim, val);

        return update(dim, val, new_val, log_prob, eng);
    }

    private :

    cxx11::normal_distribution<double> rnorm_;
    std::vector<double> new_val_vec_;
}; // class NormalRW

} // namespace vsmc

#endif // VSMC_MRW_NORMAL_HPP
