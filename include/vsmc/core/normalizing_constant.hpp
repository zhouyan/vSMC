//============================================================================
// include/vsmc/core/normalizing_constant.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_CORE_NORMALIZING_CONSTANT_HPP
#define VSMC_CORE_NORMALIZING_CONSTANT_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Calcualting normalizing constant ratio
/// \ingroup Core
class NormalizingConstant
{
    public :

    NormalizingConstant (std::size_t N) :
        size_(N), log_zconst_(0), weight_(N), inc_weight_(N) {}

    virtual ~NormalizingConstant () {}

    double zconst () const {return std::exp(log_zconst_);}

    double log_zconst () const {return log_zconst_;}

    void initialize () {log_zconst_ = 0;}

    template <typename InputIter, typename WeightSetType>
    void mul_weight (InputIter first, WeightSetType weight_set)
    {
        double *const wptr = &weight_[0];
        double *const iptr = &inc_weight_[0];
        weight_set.read_weight(wptr);
        for (std::size_t i = 0; i != size_; ++i, ++first)
            iptr[i] = *first;
        add_log_zconst();
    }

    template <typename RandomIter, typename WeightSetType>
    void mul_weight (RandomIter first, int stride,
            WeightSetType weight_set)
    {
        double *const wptr = &weight_[0];
        double *const iptr = &inc_weight_[0];
        weight_set.read_weight(wptr);
        for (std::size_t i = 0; i != size_; ++i, first += stride)
            iptr[i] = *first;
        add_log_zconst();
    }

    template <typename InputIter, typename WeightSetType>
    void add_log_weight (InputIter first, WeightSetType weight_set)
    {
        double *const wptr = &weight_[0];
        double *const iptr = &inc_weight_[0];
        weight_set.read_weight(wptr);
        for (std::size_t i = 0; i != size_; ++i, ++first)
            iptr[i] = *first;
        vd_exp(size_, iptr);
        add_log_zconst();
    }

    template <typename RandomIter, typename WeightSetType>
    void add_log_weight (RandomIter first, int stride,
            WeightSetType weight_set)
    {
        double *const wptr = &weight_[0];
        double *const iptr = &inc_weight_[0];
        weight_set.read_weight(wptr);
        for (std::size_t i = 0; i != size_; ++i, first += stride)
            iptr[i] = *first;
        vd_exp(size_, iptr);
        add_log_zconst();
    }

    protected :

    virtual void vd_exp (std::size_t N, double *inc_weight) const
    {
        using std::exp;

        for (std::size_t i = 0; i != N; ++i)
            inc_weight[i] = exp(inc_weight[i]);
    }

    virtual double inc_zconst (std::size_t N,
            const double *weight, const double *inc_weight) const
    {
        double inc = 0;
        for (std::size_t i = 0; i != N; ++i)
            inc += weight[i] * inc_weight[i];

        return inc;
    }

    private :

    std::size_t size_;
    double log_zconst_;
    std::vector<double> weight_;
    std::vector<double> inc_weight_;

    void add_log_zconst ()
    {
        using std::log;

        const double *const wptr = &weight_[0];
        const double *const iptr = &inc_weight_[0];
        log_zconst_ += log(inc_zconst(size_, wptr, iptr));
    }
}; // class NormalizingConstant

} // namespace vsmc

#endif // VSMC_CORE_NORMALIZING_CONSTANT_HPP
