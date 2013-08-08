#ifndef VSMC_MRW_UNIFORM_HPP
#define VSMC_MRW_UNIFORM_HPP

#include <vsmc/mrw/base.hpp>

namespace vsmc {

/// \brief Metropolis random walk with a uniform kernel
/// \ingroup MRW
class UniformRW : public BaseRW
{
    public :

    template <typename URNG, typename Func>
    bool operator() (double &val, double scale, URNG &eng, const Func &func)
    {
        double new_val = val + runif(eng) * scale;
        double log_prob = func(new_val) - func(val);

        return update(val, new_val, log_prob, eng);
    }

    template <typename URNG, typename Func>
    bool operator() (std::size_t dim, double *val, double scale,
            URNG &eng, const Func &func)
    {
        new_val_vec_.resize(dim);
        for (std::size_t i = 0; i != dim; ++i)
            new_val_vec_[i] = val[i] + runif(eng) * scale;
        const double *const new_val = &new_val_vec_[0];
        double log_prob = func(dim, new_val) - func(dim, val);

        return update(dim, val, new_val, log_prob, eng);
    }

    template <typename URNG, typename Func>
    bool operator() (std::size_t dim, double *val, const double *scale,
            URNG &eng, const Func &func)
    {
        new_val_vec_.resize(dim);
        for (std::size_t i = 0; i != dim; ++i)
            new_val_vec_[i] = val[i] + runif(eng) * scale[i];
        const double *const new_val = &new_val_vec_[0];
        double log_prob = func(dim, new_val) - func(dim, val);

        return update(dim, val, new_val, log_prob, eng);
    }

    private :

    std::vector<double> new_val_vec_;
}; // class UniformRW

} // namespace vsmc

#endif // VSMC_MRW_UNIFORM_HPP
