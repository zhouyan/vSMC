//============================================================================
// vSMC/include/vsmc/core/weight_set.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_CORE_WEIGHT_SET_HPP
#define VSMC_CORE_WEIGHT_SET_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/discrete_distribution.hpp>
#include <vsmc/utility/aligned_memory.hpp>

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
        std::vector<double, AlignedAllocator<double> > buffer(size_);
        double *const bptr = &buffer[0];
#if VSMC_HAS_CXX11LIB_ALGORITHM
        std::copy_n(first, size_, bptr);
#else
        for (size_type i = 0; i != size_; ++i, ++first)
            bptr[i] = *first;
#endif

        return compute_ess(bptr, use_log);
    }

    /// \brief Compute ESS given (log) incremental weights
    template <typename RandomIter>
    double ess (RandomIter first, int stride, bool use_log) const
    {
        std::vector<double, AlignedAllocator<double> > buffer(size_);
        double *const bptr = &buffer[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            bptr[i] = *first;

        return compute_ess(bptr, use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename InputIter>
    double cess (InputIter first, bool use_log) const
    {
        std::vector<double, AlignedAllocator<double> > buffer(size_);
        double *const bptr = &buffer[0];
#if VSMC_HAS_CXX11LIB_ALGORITHM
        std::copy_n(first, size_, bptr);
#else
        for (size_type i = 0; i != size_; ++i, ++first)
            bptr[i] = *first;
#endif

        return compute_cess(bptr, use_log);
    }

    /// \brief Compute CESS given (log) incremental weights
    template <typename RandomIter>
    double cess (RandomIter first, int stride, bool use_log) const
    {
        std::vector<double, AlignedAllocator<double> > buffer(size_);
        double *const bptr = &buffer[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            bptr[i] = *first;

        return compute_cess(bptr, use_log);
    }

    /// \brief Size of the weight set for the purpose of resampling
    virtual size_type resample_size () const {return size_;}

    /// \brief Read normalized weights through an output iterator for the
    /// purpose of resampling
    virtual void read_resample_weight (double *first) const
    {read_weight(first);}

    /// \brief Read normalized weights through an output iterator
    template <typename OutputIter>
    void read_weight (OutputIter first) const
    {std::copy(weight_.begin(), weight_.end(), first);}

    /// \brief Read normalized weights through a random access iterator with
    /// (possible non-uniform stride)
    template <typename RandomIter>
    void read_weight (RandomIter first, int stride) const
    {
        const double *const wptr = &weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = wptr[i];
    }

    /// \brief Read unnormalized logarithm weights through an output iterator
    template <typename OutputIter>
    void read_log_weight (OutputIter first) const
    {std::copy(log_weight_.begin(), log_weight_.end(), first);}

    /// \brief Read unnormalized logarithm weights through a random access
    /// iterator with (possible non-uniform stride)
    template <typename RandomIter>
    void read_log_weight (RandomIter first, int stride) const
    {
        const double *const lwptr = &log_weight_[0];
        for (size_type i = 0; i != size_; ++i, first += stride)
            *first = lwptr[i];
    }

    /// \brief Get the normalized weight of the id'th particle
    double weight (size_type id) const {return weight_[id];}

    /// \brief Get the unnormalized logarithm weight of the id'th particle
    double log_weight (size_type id) const {return log_weight_[id];}

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS
    /// such that each particle has a equal weight
    void set_equal_weight ()
    {
        ess_ = static_cast<double>(resample_size());
        std::fill_n(&weight_[0], size_, 1 / ess_);
        std::memset(&log_weight_[0], 0, sizeof(double) * size_);
    }

    /// \brief Set normalized weight, unnormalized logarithm weight and ESS by
    /// changing the (possible unnormalized) weights directly through an input
    /// iterator
    template <typename InputIter>
    void set_weight (InputIter first)
    {
        double *const wptr = &weight_[0];
#if VSMC_HAS_CXX11LIB_ALGORITHM
        std::copy_n(first, size_, wptr);
#else
        for (size_type i = 0; i != size_; ++i, ++first)
            wptr[i] = *first;
#endif
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
#if VSMC_HAS_CXX11LIB_ALGORITHM
        std::copy_n(first, size_, lwptr);
#else
        for (size_type i = 0; i != size_; ++i, ++first)
            lwptr[i] = *first;
#endif
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
    {return draw_(eng, weight_.begin(), weight_.end(), true);}

    /// \brief Read only access to the resampling weights
    virtual const double *resample_weight_data () const {return &weight_[0];}

    /// \brief Read only access to the raw data of weight
    const double *weight_data () const {return &weight_[0];}

    /// \brief Read only access to the raw data of logarithm weight
    const double *log_weight_data () const {return &log_weight_[0];}

    protected :

    void set_ess (double e) {ess_ = e;}

    double *mutable_weight_data () {return &weight_[0];}

    double *mutable_log_weight_data () {return &log_weight_[0];}

    /// \brief Compute unormalized logarithm weights from normalized weights
    virtual void log_weight2weight ()
    {math::vExp(size_, &log_weight_[0], &weight_[0]);}

    /// \brief Compute unormalized weights from normalized logarithm weights
    virtual void weight2log_weight ()
    {math::vLn(size_, &weight_[0], &log_weight_[0]);}

    /// \brief Normalize logarithm weights such that the maximum is zero
    virtual void normalize_log_weight ()
    {
        double *const lwptr = &log_weight_[0];
        double dmax = lwptr[0];
        for (size_type i = 0; i != size_; ++i)
            if (dmax < lwptr[i])
                dmax = lwptr[i];
        dmax = -dmax;
        for (size_type i = 0; i != size_; ++i)
            lwptr[i] += dmax;
    }

    /// \brief Normalize weights such that the summation is one
    virtual void normalize_weight ()
    {
        double *const wptr = &weight_[0];
        double coeff = 1 / math::asum(size_, wptr);
        math::scal(size_, coeff, wptr);
        ess_ = 1 / math::dot(size_, wptr, wptr);
    }

    /// \brief Compute ESS given (logarithm) unormalzied incremental weights
    virtual double compute_ess (const double *first, bool use_log) const
    {
        std::vector<double, AlignedAllocator<double> > buffer(size_);
        double *const bptr = &buffer[0];

        if (use_log) {
            math::vAdd(size_, &log_weight_[0], first, bptr);
            double dmax = bptr[0];
            for (size_type i = 0; i != size_; ++i)
                if (dmax < bptr[i])
                    dmax = bptr[i];
            dmax = -dmax;
            for (size_type i = 0; i != size_; ++i)
                bptr[i] += dmax;
            math::vExp(size_, bptr, bptr);
        } else {
            math::vMul(size_, &weight_[0], first, bptr);
        }

        double coeff = 1 / math::asum(size_, bptr);
        math::scal(size_, coeff, bptr);

        return 1 / math::dot(size_, bptr, bptr);
    }

    /// \brief Compute CESS given (logarithm) unormalized incremental weights
    virtual double compute_cess (const double *first, bool use_log) const
    {
        const double *bptr = first;
        const double *const wptr = &weight_[0];
        std::vector<double, AlignedAllocator<double> > buffer;
        if (use_log) {
            buffer.resize(size_);
            math::vExp(size_, first, &buffer[0]);
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
    std::vector<double, AlignedAllocator<double> > weight_;
    std::vector<double, AlignedAllocator<double> > log_weight_;
    DiscreteDistribution<size_type> draw_;

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
class WeightSetNull
{
    public :

    typedef std::size_t size_type;

    explicit WeightSetNull (size_type) {}

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

    void read_resample_weight (double *) const {}

    template <typename OutputIter>
    void read_weight (OutputIter) const {}

    template <typename RandomIter>
    void read_weight (RandomIter, int) const {}

    template <typename OutputIter>
    void read_log_weight (OutputIter) const {}

    template <typename RandomIter>
    void read_log_weight (RandomIter, int) const {}

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
