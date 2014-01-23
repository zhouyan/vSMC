#ifndef VSMC_RANDOM_WALK_BASE_HPP
#define VSMC_RANDOM_WALK_BASE_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_RANDOM_WALK_BASE_INVALID_MEMCPY(diff, size, func)\
    VSMC_RUNTIME_ASSERT((std::abs(diff) > static_cast<std::ptrdiff_t>(size)),\
            ("THE DESTINATION AND SOURCE OF **"#func"** OVERLAPPING"))

namespace vsmc {

/// \brief Base class of Metropolis random walks
/// \ingroup RandomWalk
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
        VSMC_RUNTIME_ASSERT_RANDOM_WALK_BASE_INVALID_MEMCPY(
                val - new_val, dim, BaseRW::update);
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

#endif // VSMC_RANDOM_WALK_BASE_HPP
