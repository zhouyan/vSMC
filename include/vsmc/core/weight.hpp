#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

#if VSMC_USE_MKL
#include <mkl_vml.h>
#endif

namespace vsmc {

/// \brief Weight set class
/// \ingroup Core
template <typename T>
class WeightSet
{
    public :

    typedef typename traits::SizeTypeTrait<T>::type size_type;

    explicit WeightSet (size_type N) :
        size_(N), ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

    size_type size () const {return size_;}

    size_type resample_size () const {return size_;}

    template <typename OutputIter>
    OutputIter read_resample_weight (OutputIter first) const
    {return read_weight(first);}

    template <typename RandomIter>
    RandomIter read_resample_weight (RandomIter first, int stride) const
    {return read_weight(first, stride);}

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = weight_[i];

        return first;
    }

    /// \brief Read normalized weights through a random access iterator with
    /// (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_weight (RandomIter first, int stride) const
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = weight_[i];

        return first;
    }

    /// \brief Read normalized weights through a pointer
    double *read_weight (double *first) const
    {
        const double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT_INVALID_MEMCPY_OUT(
                first - wptr, size_, WeightSet::read_weight);
        std::memcpy(first, wptr, sizeof(double) * size_);

        return first + size_;
    }

    /// \brief Read unnormalized logarithm weights through an output iterator
    template <typename OutputIter>
    OutputIter read_log_weight (OutputIter first) const
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = log_weight_[i];

        return first;
    }

    /// \brief Read unnormalized logarithm weights through a random access
    /// iterator with (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_log_weight (RandomIter first, int stride) const
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = log_weight_[i];

        return first;
    }

    /// \brief Read unnormalized logarithm weights through a pointer
    double *read_log_weight (double *first) const
    {
        const double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT_INVALID_MEMCPY_OUT(
                first - lwptr, size_, WeightSet::read_log_weight);
        std::memcpy(first, lwptr, sizeof(double) * size_);

        return first + size_;
    }

    /// \brief Get the normalized weight of the id'th particle
    double weight (size_type id) const {return weight_[id];}

    /// \brief Get the unnormalized logarithm weight of the id'th particle
    double log_weight (size_type id) const {return log_weight_[id];}

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight ()
    {
        ess_ = static_cast<double>(size_);
        double ew = 1 / ess_;
        for (size_type i = 0; i != size_; ++i) {
            weight_[i] = ew;
            log_weight_[i] = 0;
        }
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            weight_[i] = *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            weight_[i] = *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a pointer
    void set_weight (const double *first)
    {
        double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT_INVALID_MEMCPY_IN(
                first - wptr, size_, WeightSet::set_weight);
        std::memcpy(wptr, first, sizeof(double) * size_);
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through an input iterator
    template <typename InputIter>
    void mul_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            weight_[i] *= *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    void mul_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            weight_[i] *= *first;
        weight2log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// an input iterator
    template <typename InputIter>
    void set_log_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            log_weight_[i] = *first;
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_log_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            log_weight_[i] = *first;
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a pointer
    void set_log_weight (const double *first)
    {
        double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT_INVALID_MEMCPY_IN(
                first - lwptr, size_, WeightSet::set_log_weight);
        std::memcpy(lwptr, first, sizeof(double) * size_);
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    void add_log_weight (InputIter first)
    {
        for (size_type i = 0; i != size_; ++i, ++first)
            log_weight_[i] += *first;
        log_weight2weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void add_log_weight (RandomIter first, int stride)
    {
        for (size_type i = 0; i != size_; ++i, first += stride)
            log_weight_[i] += *first;
        log_weight2weight();
    }

    /// \brief Get the ESS of the particle collection based on the current
    /// weights
    double ess () const {return ess_;}

    protected :

    std::vector<double> &weight_vec () {return weight_;}

    const std::vector<double> &weight_vec () const {return weight_;}

    std::vector<double> &log_weight_vec () {return log_weight_;}

    const std::vector<double> &log_weight_vec () const {return log_weight_;}

    private :

    size_type size_;
    double ess_;
    std::vector<double> weight_;
    std::vector<double> log_weight_;

    void log_weight2weight ()
    {
        using std::exp;

        normalize_log_weight();
#if VSMC_USE_MKL
        ::vdExp(static_cast<MKL_INT>(size_), &log_weight_[0], &weight_[0]);
#else
        for (size_type i = 0; i != size_; ++i)
            weight_[i] = exp(log_weight_[i]);
#endif
        normalize_weight();
    }

    void weight2log_weight ()
    {
        using std::log;

        normalize_weight();
#if VSMC_USE_MKL
        ::vdLn(static_cast<MKL_INT>(size_), &weight_[0], &log_weight_[0]);
#else
        for (size_type i = 0; i != size_; ++i)
            log_weight_[i] = log(weight_[i]);
#endif
        normalize_log_weight();
    }

    virtual void normalize_log_weight ()
    {
        double max_weight = log_weight_[0];
        for (size_type i = 0; i != size_; ++i)
            if (log_weight_[i] > max_weight)
                max_weight = log_weight_[i];
        for (size_type i = 0; i != size_; ++i)
            log_weight_[i] -= max_weight;
    }

    virtual void normalize_weight ()
    {
        double coeff = 0;
        for (size_type i = 0; i != size_; ++i)
            coeff += weight_[i];
        coeff = 1 / coeff;
        for (size_type i = 0; i != size_; ++i)
            weight_[i] *= coeff;
        ess_ = 0;
        for (size_type i = 0; i != size_; ++i)
            ess_ += weight_[i] * weight_[i];
        ess_ = 1 / ess_;
    }
}; // class WeightSet

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_HPP
