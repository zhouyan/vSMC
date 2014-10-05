//============================================================================
// include/vsmc/core/weight_set.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_CORE_WEIGHT_SET_HPP
#define VSMC_CORE_WEIGHT_SET_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/cxx11/random.hpp>
#include <vsmc/math/vmath.hpp>
#include <cstring>
#include <limits>

#define VSMC_RUNTIME_ASSERT_CORE_WEIGHT_SET_INVALID_MEMCPY_IN(diff, size, f) \
    VSMC_RUNTIME_ASSERT((std::abs(diff) >= static_cast<std::ptrdiff_t>(size)),\
            ("THE DESTINATION OF **"#f"** OVERLAPPING WITH THE SOURCE"))

#define VSMC_RUNTIME_ASSERT_CORE_WEIGHT_SET_INVALID_MEMCPY_OUT(diff, size, f) \
    VSMC_RUNTIME_ASSERT((std::abs(diff) >= static_cast<std::ptrdiff_t>(size)),\
            ("THE SOURCE OF **"#f"** OVERLAPPING WITH THE DESTINATION"))

namespace vsmc {

/// \brief Weight set class
/// \ingroup Core
class WeightSet
{
    public :

    typedef std::size_t size_type;

    explicit WeightSet (size_type N) :
        size_(N), ess_(static_cast<double>(N)), weight_(N), log_weight_(N) {}

#if VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS
    WeightSet (const WeightSet &) = default;
    WeightSet &operator= (const WeightSet &) = default;
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    WeightSet (WeightSet &&) = default;
    WeightSet &operator= (WeightSet &&) = default;
#endif
#else // VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS
    WeightSet (const WeightSet &other) :
        size_(other.size_), ess_(other.ess_),
        weight_(other.weight_), log_weight_(other.log_weight_) {}

    WeightSet &operator= (const WeightSet &other) {
        if (this != &other) {
            size_       = other.size_;
            ess_        = other.ess_;
            weight_     = other.weight_;
            log_weight_ = other.log_weight_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    WeightSet (WeightSet &&other) :
        size_(other.size_), ess_(other.ess_),
        weight_(cxx11::move(other.weight_)),
        log_weight_(cxx11::move(other.log_weight_)) {}

    WeightSet &operator= (WeightSet &&other) {
        if (this != &other) {
            size_       = other.size_;
            ess_        = other.ess_;
            weight_     = cxx11::move(other.weight_);
            log_weight_ = cxx11::move(other.log_weight_);
        }

        return *this;
    }
#endif
#endif // VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS

    virtual ~WeightSet () {}

    size_type size () const {return size_;}

    /// \brief ESS of the particle collection based on the current weights
    double ess () const {return ess_;}

    /// \brief Compute ESS given (log) incremental weights
    template <typename InputIter>
    double ess (InputIter first, bool use_log) const
    {
        std::vector<double> buffer(size_);
        double *const bptr = &buffer[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            bptr[i] = *first;

        return compute_ess(bptr, use_log);
    }

    /// \brief Compute ESS given (log) incremental weights
    template <typename RandomIter>
    double ess (RandomIter first, int stride, bool use_log) const
    {
        std::vector<double> buffer(size_);
        double *const bptr = &buffer[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            bptr[i] = *first;

        return compute_ess(bptr, use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename InputIter>
    double cess (InputIter first, bool use_log) const
    {
        std::vector<double> buffer(size_);
        double *const bptr = &buffer[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            bptr[i] = *first;

        return compute_cess(bptr, use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename RandomIter>
    double cess (RandomIter first, int stride, bool use_log) const
    {
        std::vector<double> buffer(size_);
        double *const bptr = &buffer[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            bptr[i] = *first;

        return compute_cess(bptr, use_log);
    }

    /// \brief Size of the weight set for the purpose of resampling
    virtual size_type resample_size () const {return size_;}

    /// \brief Read normalized weights through an output iterator for the
    /// purpose of resampling
    virtual double *read_resample_weight (double *first) const
    {return read_weight(first);}

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
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_SET_INVALID_MEMCPY_OUT(
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
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_SET_INVALID_MEMCPY_OUT(
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
    InputIter set_weight (InputIter first)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            wptr[i] = *first;
        post_set_weight();

        return first;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a random
    /// access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    RandomIter set_weight (RandomIter first, int stride)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            wptr[i] = *first;
        post_set_weight();

        return first;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through a pointer
    const double *set_weight (const double *first)
    {
        double *const wptr = &weight_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_SET_INVALID_MEMCPY_IN(
                first - wptr, size_, WeightSet::set_weight);
        std::memcpy(wptr, first, sizeof(double) * size_);
        post_set_weight();

        return first + size_;
    }

    double *set_weight (double *first)
    {
        set_weight(static_cast<const double *>(first));

        return first + size_;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through an input iterator
    template <typename InputIter>
    InputIter mul_weight (InputIter first)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            wptr[i] *= *first;
        post_set_weight();

        return first;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// multiply the normalized weight with (possible unnormalized) incremental
    /// weights through a random access iterator with (possible non-uniform)
    /// stride
    template <typename RandomIter>
    RandomIter mul_weight (RandomIter first, int stride)
    {
        double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            wptr[i] *= *first;
        post_set_weight();

        return first;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// an input iterator
    template <typename InputIter>
    InputIter set_log_weight (InputIter first)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            lwptr[i] = *first;
        post_set_log_weight();

        return first;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a random access iterator with (possible non-uniform) stride
    template <typename RandomIter>
    RandomIter set_log_weight (RandomIter first, int stride)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            lwptr[i] = *first;
        post_set_log_weight();

        return first;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) logarithm weights directly through
    /// a pointer
    const double *set_log_weight (const double *first)
    {
        double *const lwptr = &log_weight_[0];
        VSMC_RUNTIME_ASSERT_CORE_WEIGHT_SET_INVALID_MEMCPY_IN(
                first - lwptr, size_, WeightSet::set_log_weight);
        std::memcpy(lwptr, first, sizeof(double) * size_);
        post_set_log_weight();

        return first + size_;
    }

    double *set_log_weight (double *first)
    {
        set_log_weight(static_cast<const double *>(first));

        return first + size_;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through an input iterator
    template <typename InputIter>
    InputIter add_log_weight (InputIter first)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, ++first)
            lwptr[i] += *first;
        post_set_log_weight();

        return first;
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// adding to the unnormalized logarithm weights with (possible
    /// unormalized) logarithm incremental weights through a ranodm access
    /// iterator with (possible non-uniform) stride
    template <typename RandomIter>
    RandomIter add_log_weight (RandomIter first, int stride)
    {
        double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            lwptr[i] += *first;
        post_set_log_weight();

        return first;
    }

    /// \brief Draw a sample according to the weights
    template <typename URNG>
    size_type draw (URNG &eng) const
    {
        cxx11::uniform_real_distribution<double> runif(0, 1);
        double u = runif(eng);
        size_type i = 0;
        const double *w = &weight_[0];
        double sum = w[0];
        while (sum < u) {
            ++i;
            sum += w[i];
        }

        return i;
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

    /// \brief Compute unormalized logarithm weights from normalized weights
    virtual void log_weight2weight ()
    {math::vexp(size_, log_weight_ptr(), weight_ptr());}

    /// \brief Compute unormalized weights from normalized logarithm weights
    virtual void weight2log_weight ()
    {math::vlog(size_, weight_ptr(), log_weight_ptr());}

    /// \brief Normalize logarithm weights such that the maximum is zero
    virtual void normalize_log_weight ()
    {
        double *const lwptr = log_weight_ptr();
        double max_weight = lwptr[0];
        for (size_type i = 0; i != size_; ++i)
            if (max_weight < lwptr[i])
                max_weight = lwptr[i];
        for (size_type i = 0; i != size_; ++i)
            lwptr[i] -= max_weight;
    }

    /// \brief Normalize weights such that the summation is one
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

    /// \brief Compute ESS given (logarithm) unormalzied incremental weights
    virtual double compute_ess (const double *first, bool use_log) const
    {
        std::vector<double> buffer(size_);
        double *const bptr = &buffer[0];

        if (use_log) {
            math::vadd(size_, log_weight_ptr(), first, bptr);
            double max_weight = bptr[0];
            for (size_type i = 0; i != size_; ++i)
                if (max_weight < bptr[i])
                    max_weight = bptr[i];
            for (size_type i = 0; i != size_; ++i)
                bptr[i] -= max_weight;
            math::vexp(size_, bptr, bptr);
        } else {
            math::vmul(size_, weight_ptr(), first, bptr);
        }

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

    /// \brief Compute CESS given (logarithm) unormalized incremental weights
    virtual double compute_cess (const double *first, bool use_log) const
    {
        const double *bptr = first;
        const double *const wptr = weight_ptr();
        std::vector<double> buffer;
        if (use_log) {
            buffer.resize(size_);
            math::vexp(size_, first, &buffer[0]);
            bptr = &buffer[0];
        }

        double above = 0;
        double below = 0;
        for (size_type i = 0; i != size_; ++i) {
            double wb = wptr[i] * bptr[i];
            above += wb;
            below += wb * bptr[i];
        }

        return above * above / below;
    }

    private :

    size_type size_;
    double ess_;
    std::vector<double> weight_;
    std::vector<double> log_weight_;

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

/// \brief An empty weight set class
/// \ingroup Core
///
/// \details
/// This class provides all the interfaces of WeightSet, while they do nothing
/// at all and the class cost no memory usage. This is primarily to be used in
/// algorithms where weights are irrelevant. Any attempt of using member
/// functions of this class will not result in compile time or runtime errors,
/// but the results might not be what one will be expecting.
class WeightSetEmpty
{
    public :

    typedef std::size_t size_type;

    explicit WeightSetEmpty (size_type) {}

    size_type size () const {return 0;}

    double ess () const {return max_ess();}

    template <typename InputIter>
    double ess (InputIter, bool) const {return max_ess();}

    template <typename RandomIter>
    double ess (RandomIter, int, bool) const {return max_ess();}

    template <typename InputIter>
    double cess (InputIter, bool) const {return max_ess();}

    template <typename RandomIter>
    double cess (RandomIter, int, bool) const {return max_ess();}

    size_type resample_size () const {return 0;}

    double *read_resample_weight (double *) const {return VSMC_NULLPTR;}

    template <typename OutputIter>
    OutputIter read_weight (OutputIter first) const {return first;}

    template <typename RandomIter>
    RandomIter read_weight (RandomIter first, int) const {return first;}

    template <typename OutputIter>
    OutputIter read_log_weight (OutputIter first) const {return first;}

    template <typename RandomIter>
    RandomIter read_log_weight (RandomIter first, int) const {return first;}

    double weight (size_type) const {return 1;}

    double log_weight (size_type) const {return 0;}

    void set_equal_weight () {}

    template <typename InputIter>
    void set_weight (InputIter) {}

    template <typename RandomIter>
    void set_weight (RandomIter, int) {}

    template <typename InputIter>
    void mul_weight (InputIter) {}

    template <typename RandomIter>
    void mul_weight (RandomIter, int) {}

    template <typename InputIter>
    void set_log_weight (InputIter) {}

    template <typename RandomIter>
    void set_log_weight (RandomIter, int) {}

    template <typename InputIter>
    void add_log_weight (InputIter) {}

    template <typename RandomIter>
    void add_log_weight (RandomIter, int) {}

    template <typename URNG>
    size_type draw (URNG &) const {return 0;}

    private :

    static double max_ess ()
    {return std::numeric_limits<double>::max VSMC_MNE ();}
}; // class WeightSetEmtpy

} // namespace vsmc

#endif // VSMC_CORE_WEIGHT_SET_HPP
