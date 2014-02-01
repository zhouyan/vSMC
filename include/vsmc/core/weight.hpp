#ifndef VSMC_CORE_WEIGHT_HPP
#define VSMC_CORE_WEIGHT_HPP

#include <vsmc/internal/common.hpp>

#if VSMC_USE_MKL
#include <mkl_vml.h>
#endif

#define VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_IN(diff, size, func) \
    VSMC_RUNTIME_ASSERT((std::abs(diff) >= static_cast<std::ptrdiff_t>(size)),\
            ("THE DESTINATION OF **"#func"** OVERLAPPING WITH THE SOURCE"))

#define VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_OUT(diff, size, func) \
    VSMC_RUNTIME_ASSERT((std::abs(diff) >= static_cast<std::ptrdiff_t>(size)),\
            ("THE SOURCE OF **"#func"** OVERLAPPING WITH THE DESTINATION"))

namespace vsmc {

/// \brief Weight set class
/// \ingroup Core
class WeightSet
{
    public :

    typedef std::size_t size_type;

    explicit WeightSet (size_type N) :
        size_(N), ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

    size_type size () const {return size_;}

    /// \brief ESS of the particle collection based on the current weights
    double ess () const {return ess_;}

    /// \brief Compute ESS given (log) incremental weights
    template <typename InputIter>
    double ess (InputIter first, bool use_log) const
    {
        buffer_.resize(size_);
        double *const bptr = &buffer_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            bptr[i] = *first;

        return compute_ess(bptr, use_log);
    }

    /// \brief Compute ESS given (log) incremental weights
    template <typename RandomIter>
    double ess (RandomIter first, int stride, bool use_log) const
    {
        buffer_.resize(size_);
        double *const bptr = &buffer_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            bptr[i] = *first;

        return compute_ess(bptr, use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename InputIter>
    double cess (InputIter first, bool use_log) const
    {
        buffer_.resize(size_);
        double *const bptr = &buffer_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            bptr[i] = *first;

        return compute_cess(bptr, use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename RandomIter>
    double cess (RandomIter first, int stride, bool use_log) const
    {
        buffer_.resize(size_);
        double *const bptr = &buffer_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            bptr[i] = *first;

        return compute_cess(bptr, use_log);
    }

    /// \brief Size of the weight set for the purpose of resampling
    size_type resample_size () const {return size_;}

    /// \brief Read normalized weights through an output iterator for the
    /// purpose of resampling
    template <typename OutputIter>
    OutputIter read_resample_weight (OutputIter first) const
    {return read_weight(first);}

    /// \brief Read normalized weights through a random access iterator for
    /// the purpose of resampling
    template <typename RandomIter>
    RandomIter read_resample_weight (RandomIter first, int stride) const
    {return read_weight(first, stride);}

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const
    {
        const double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = wptr[i];

        return first;
    }

    /// \brief Read normalized weights through a random access iterator with
    /// (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_weight (RandomIter first, int stride) const
    {
        const double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = wptr[i];

        return first;
    }

    /// \brief Read normalized weights through a pointer
    double *read_weight (double *first) const
    {
        const double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_OUT(
                first - wptr, size_, WeightSet::read_weight);
        std::memcpy(first, wptr, sizeof(double) * size_);

        return first + size_;
    }

    /// \brief Read unnormalized logarithm weights through an output iterator
    template <typename OutputIter>
    OutputIter read_log_weight (OutputIter first) const
    {
        const double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            *first = lwptr[i];

        return first;
    }

    /// \brief Read unnormalized logarithm weights through a random access
    /// iterator with (possible non-uniform stride)
    template <typename RandomIter>
    RandomIter read_log_weight (RandomIter first, int stride) const
    {
        const double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = lwptr[i];

        return first;
    }

    /// \brief Read unnormalized logarithm weights through a pointer
    double *read_log_weight (double *first) const
    {
        const double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_OUT(
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
        double *const wptr = &weight_[0];
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i) {
            wptr[i] = ew;
            lwptr[i] = 0;
        }
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set_weight (InputIter first)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            wptr[i] = *first;
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_weight (RandomIter first, int stride)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            wptr[i] = *first;
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a pointer
    void set_weight (const double *first)
    {
        double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_IN(
                first - wptr, size_, WeightSet::set_weight);
        std::memcpy(wptr, first, sizeof(double) * size_);
        post_set_weight();
    }

    void set_weight (double *first)
    {set_weight(static_cast<const double *>(first));}

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through an input iterator
    template <typename InputIter>
    void mul_weight (InputIter first)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            wptr[i] *= *first;
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    void mul_weight (RandomIter first, int stride)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            wptr[i] *= *first;
        post_set_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// an input iterator
    template <typename InputIter>
    void set_log_weight (InputIter first)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            lwptr[i] = *first;
        post_set_log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void set_log_weight (RandomIter first, int stride)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            lwptr[i] = *first;
        post_set_log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a pointer
    void set_log_weight (const double *first)
    {
        double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_IN(
                first - lwptr, size_, WeightSet::set_log_weight);
        std::memcpy(lwptr, first, sizeof(double) * size_);
        post_set_log_weight();
    }

    void set_log_weight (double *first)
    {set_log_weight(static_cast<const double *>(first));}

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    void add_log_weight (InputIter first)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            lwptr[i] += *first;
        post_set_log_weight();
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    void add_log_weight (RandomIter first, int stride)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            lwptr[i] += *first;
        post_set_log_weight();
    }

    /// \brief Draw a sample according to the weights
    template <typename URNG>
    size_type draw (URNG &eng) const
    {
        cxx11::discrete_distribution<size_type>
            rsample(weight_.begin(), weight_.end());

        return rsample(eng);
    }

    protected :

    void set_ess (double e) {ess_ = e;}

    double *weight_ptr () {return &weight_[0];}

    const double *weight_ptr () const {return &weight_[0];}

    double *log_weight_ptr () {return &log_weight_[0];}

    const double *log_weight_ptr () const {return &log_weight_[0];}

    std::vector<double> &weight_vec () {return weight_;}

    const std::vector<double> &weight_vec () const {return weight_;}

    std::vector<double> &log_weight_vec () {return log_weight_;}

    const std::vector<double> &log_weight_vec () const {return log_weight_;}

    virtual void log_weight2weight ()
    {
        using std::exp;

        double *const wptr = weight_ptr();
        const double *const lptr = log_weight_ptr();
#if VSMC_USE_MKL
        ::vdExp(static_cast<MKL_INT>(size_), lptr, wptr);
#else
        for (size_type i = 0; i != size_; ++i)
            wptr[i] = exp(lptr[i]);
#endif
    }

    virtual void weight2log_weight ()
    {
        using std::log;

        const double *const wptr = weight_ptr();
        double *const lptr = log_weight_ptr();
#if VSMC_USE_MKL
        ::vdLn(static_cast<MKL_INT>(size_), wptr, lptr);
#else
        for (size_type i = 0; i != size_; ++i)
            lptr[i] = log(wptr[i]);
#endif
    }

    virtual void normalize_log_weight ()
    {
        double *const lptr = log_weight_ptr();
        double max_weight = lptr[0];
        for (size_type i = 0; i != size_; ++i)
            if (lptr[i] > max_weight)
                max_weight = lptr[i];
        for (size_type i = 0; i != size_; ++i)
            lptr[i] -= max_weight;
    }

    virtual void normalize_weight ()
    {
        double *const wptr = weight_ptr();
        double coeff = 0;
        for (size_type i = 0; i != size_; ++i)
            coeff += wptr[i];
        coeff = 1 / coeff;
        for (size_type i = 0; i != size_; ++i)
            wptr[i] *= coeff;
        ess_ = 0;
        for (size_type i = 0; i != size_; ++i)
            ess_ += wptr[i] * wptr[i];
        ess_ = 1 / ess_;
    }

    virtual double compute_ess (const double *first, bool use_log) const
    {
        using std::exp;

        if (first != &buffer_[0]) {
            buffer_.resize(size_);
            double *const ptr = &buffer_[0];
            VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_IN(
                    first - ptr, size_, WeightSet::ess);
            std::memcpy(ptr, first, sizeof(double) * size_);
        }
        double *const bptr = &buffer_[0];
        const double *const wptr = weight_ptr();

        if (use_log) {
#if VSMC_USE_MKL
            ::vdExp(static_cast<MKL_INT>(size_), bptr, bptr);
#else
            for (size_type i = 0; i != size_; ++i)
                bptr[i] = exp(bptr[i]);
#endif
        }

        for (size_type i = 0; i != size_; ++i)
            bptr[i] *= wptr[i];
        double coeff = 0;
        for (size_type i = 0; i != size_; ++i)
            coeff += bptr[i];
        coeff = 1 / coeff;
        for (size_type i = 0; i != size_; ++i)
            bptr[i] *= coeff;

        double res = 0;
        for (size_type i = 0; i != size_; ++i)
            res += bptr[i] * bptr[i];
        res = 1 / res;

        return res;
    }

    virtual double compute_cess (const double *first, bool use_log) const
    {
        using std::exp;

        if (first != &buffer_[0]) {
            buffer_.resize(size_);
            double *const ptr = &buffer_[0];
            VSMC_RUNTIME_ASSERT_CORE_WEIGHT_INVALID_MEMCPY_IN(
                    first - ptr, size_, WeightSet::cess);
            std::memcpy(ptr, first, sizeof(double) * size_);
        }
        double *const bptr = &buffer_[0];
        const double *const wptr = weight_ptr();

        if (use_log) {
#if VSMC_USE_MKL
            ::vdExp(static_cast<MKL_INT>(size_), bptr, bptr);
#else
            for (size_type i = 0; i != size_; ++i)
                bptr[i] = exp(bptr[i]);
#endif
        }

        double above = 0;
        double below = 0;
        for (size_type i = 0; i != size_; ++i) {
            above += wptr[i] * bptr[i];
            below += wptr[i] * bptr[i] * bptr[i];
        }

        return above * above / below;
    }

    private :

    size_type size_;
    double ess_;
    std::vector<double> weight_;
    std::vector<double> log_weight_;
    mutable std::vector<double> buffer_;

    void post_set_log_weight ()
    {
        normalize_log_weight();
        log_weight2weight();
        normalize_weight();
    }

    void post_set_weight ()
    {
        normalize_weight();
        weight2log_weight();
        normalize_log_weight();
    }
}; // class WeightSet

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_HPP
