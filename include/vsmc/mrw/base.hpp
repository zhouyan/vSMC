#ifndef VSMC_MRW_BASE_HPP
#define VSMC_MRW_BASE_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Base class of Metropolis random walks
/// \ingroup MRW
class BaseRW
{
    public :

    BaseRW () : runif_(0, 1) {}

    protected :

    template <typename URNG>
    double runif (URNG &eng)
    {
        return runif_(eng);
    }

    template <typename URNG>
    bool update (double &val, double new_val, double log_prob, URNG &eng)
    {
        using std::log;

        double u = log(runif_(eng));
        if (u < log_prob) {
            val = new_val;
            return true;
        }

        return false;
    }

    template <typename URNG>
    bool update (std::size_t dim, double *val, const double *new_val,
            double log_prob, URNG &eng)
    {
        using std::log;
        double u = log(runif_(eng));
        if (u < log_prob) {
            std::memcpy(val, new_val, sizeof(double) * dim);
            return true;
        }

        return false;
    }

    private :

    cxx11::uniform_real_distribution<double> runif_;
}; // class BaseRW

} // namespace vsmc

#endif // VSMC_MRW_BASE_HPP
