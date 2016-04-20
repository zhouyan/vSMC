//============================================================================
// vSMC/include/vsmc/core/monitor.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_CORE_MONITOR_HPP
#define VSMC_CORE_MONITOR_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(func)                             \
    VSMC_RUNTIME_ASSERT(                                                      \
        (id < dim()), "**Monitor::" #func "** INVALID ID NUMBER ARGUMENT")

#define VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(func)                           \
    VSMC_RUNTIME_ASSERT((iter < iter_size()),                                 \
        "**Monitor::" #func "** INVALID ITERATION NUMBER ARGUMENT")

#define VSMC_RUNTIME_ASSERT_CORE_MONITOR_EVAL                                 \
    VSMC_RUNTIME_ASSERT(eval_, "**Monitor::eval** INVALID EVALUATION OBJECT")

namespace vsmc
{

/// \brief Monitor stage
/// \ingroup Definitions
enum MonitorStage {
    MonitorMove,     ///< Monitor evaluated after moves
    MonitorResample, ///< Monitor evaluated after resampling
    MonitorMCMC      ///< Monitor evaluated after MCMC moves
};                   // enum MonitorStage

/// \brief Monitor for Monte Carlo integration
/// \ingroup Core
template <typename T>
class Monitor
{
    public:
    using value_type = T;
    using eval_type =
        std::function<void(std::size_t, std::size_t, Particle<T> &, double *)>;

    Monitor(std::size_t dim, const eval_type &eval, bool record_only = false,
        MonitorStage stage = MonitorMCMC)
        : dim_(dim)
        , eval_(eval)
        , recording_(true)
        , record_only_(record_only)
        , stage_(stage)
        , name_(dim)
    {
        internal::size_check<VSMC_CBLAS_INT>(dim_, "Monitor::Monitor");
    }

    /// \brief The dimension of the Monitor
    std::size_t dim() const { return dim_; }

    /// \brief If this is a record only Monitor
    bool record_only() const { return record_only_; }

    /// \brief The stage of the Montior
    MonitorStage stage() const { return stage_; }

    /// \brief The number of iterations has been recorded
    std::size_t iter_size() const { return index_.size(); }

    /// \brief Reserve space for a specified number of iterations
    void reserve(std::size_t num)
    {
        index_.reserve(num);
        record_.reserve(dim_ * num);
    }

    /// \brief Whether the evaluation object is valid
    bool empty() const { return !eval_; }

    /// \brief Read and write access to the names of variables
    ///
    /// \details
    /// By default, each variable of a Monitor is unnamed and the returned
    /// string is empty. However, the user can selectively set the names of
    /// each variable. This effect how Sampler will print the headers of the
    /// summary table.
    std::string &name(std::size_t id)
    {
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(name);

        return name_[id];
    }

    /// \brief Read only access to the names of variables
    const std::string &name(std::size_t id) const
    {
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(name);

        return name_[id];
    }

    /// \brief Get the latest iteration index of the sampler
    std::size_t index() const
    {
        std::size_t iter = iter_size() > 0 ? iter_size() - 1 : iter_size();
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(index);

        return index_[iter];
    }

    /// \brief Get the iteration index of the sampler of a given Monitor
    /// iteration
    ///
    /// \details
    /// For example, if a Monitor is only added to the sampler at the
    /// sampler's iteration `siter`. Then `index(0)` will be `siter` and so
    /// on. If the Monitor is added before the sampler's initialization and
    /// continued to be evaluated during the iterations without calling
    /// `turnoff()`, then iter(iter) shall just be `iter`.
    std::size_t index(std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(index);

        return index_[iter];
    }

    /// \brief Read the index history through an output iterator
    template <typename OutputIter>
    OutputIter read_index(OutputIter first) const
    {
        return std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Get the latest Monte Carlo integration record of a given
    /// variable
    double record(std::size_t id) const
    {
        std::size_t iter = iter_size() > 0 ? iter_size() - 1 : iter_size();
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(record);
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(record);

        return record_[iter * dim_ + id];
    }

    /// \brief Get the Monte Carlo integration record of a given variable and
    /// the Monitor iteration
    double record(std::size_t id, std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(record);
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(record);

        return record_[iter * dim_ + id];
    }

    /// \brief Read the record history for a given variable through an output
    /// iterator
    template <typename OutputIter>
    OutputIter read_record(std::size_t id, OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(record);

        const std::size_t N = iter_size();
        const double *riter = record_.data() + id;
        for (std::size_t i = 0; i != N; ++i, ++first, riter += dim_)
            *first = *riter;

        return first;
    }

    /// \brief Read the record history of all variables as a matrix through an
    /// output iterator
    template <typename OutputIter>
    OutputIter read_record_matrix(MatrixLayout layout, OutputIter first) const
    {
        if (layout == RowMajor)
            return std::copy(record_.begin(), record_.end(), first);

        if (layout == ColMajor) {
            const std::size_t N = iter_size();
            for (std::size_t d = 0; d != dim_; ++d) {
                const double *riter = record_.data() + d;
                for (std::size_t i = 0; i != N; ++i, ++first, riter += dim_)
                    *first = *riter;
            }
        }
        return first;
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval(const eval_type &new_eval) { eval_ = new_eval; }

    /// \brief Perform the evaluation for a given iteration and a Particle<T>
    /// object.
    void eval(std::size_t iter, Particle<T> &particle, MonitorStage stage)
    {
        internal::size_check<VSMC_CBLAS_INT>(particle.size(), "Monitor::eval");

        if (!recording_)
            return;

        if (stage != stage_)
            return;

        VSMC_RUNTIME_ASSERT_CORE_MONITOR_EVAL;

        result_.resize(dim_);
        if (record_only_) {
            eval_(iter, dim_, particle, result_.data());
            push_back(iter);

            return;
        }

        const std::size_t N = static_cast<std::size_t>(particle.size());
        buffer_.resize(N * dim_);
        eval_(iter, dim_, particle, buffer_.data());
        ::cblas_dgemv(::CblasColMajor, ::CblasNoTrans,
            static_cast<VSMC_CBLAS_INT>(dim_), static_cast<VSMC_CBLAS_INT>(N),
            1.0, buffer_.data(), static_cast<VSMC_CBLAS_INT>(dim_),
            particle.weight().data(), 1, 0.0, result_.data(), 1);
        push_back(iter);
    }

    /// \brief Clear all records of the index and integrations
    void clear()
    {
        index_.clear();
        record_.clear();
    }

    /// \brief Whether the Monitor is actively recording results
    bool recording() const { return recording_; }

    /// \brief Turn on the recording
    void turn_on() { recording_ = true; }

    /// \brief Turn off the recording
    void turn_off() { recording_ = false; }

    private:
    std::size_t dim_;
    eval_type eval_;
    bool recording_;
    bool record_only_;
    MonitorStage stage_;
    Vector<std::string> name_;
    Vector<std::size_t> index_;
    Vector<double> record_;
    Vector<double> result_;
    Vector<double> buffer_;

    void push_back(std::size_t iter)
    {
        index_.push_back(iter);
        record_.insert(record_.end(), result_.begin(), result_.end());
    }
}; // class Monitor

} // namespace vsmc

#endif // VSMC_CORE_MONITOR_HPP
