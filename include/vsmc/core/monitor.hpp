#ifndef VSMC_CORE_MONITOR_HPP
#define VSMC_CORE_MONITOR_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/cxxblas.hpp>

namespace vsmc {

/// \brief Monitor for Monte Carlo integration
/// \ingroup Core
template <typename T>
class Monitor
{
    public :

    typedef T value_type;
    typedef cxx11::function<void (
            unsigned, unsigned, const Particle<T> &, double *)> eval_type;
    typedef typename traits::DGemvTypeTrait<T>::type dgemv_type;

    explicit Monitor (unsigned dim = 1, const eval_type &eval = eval_type(),
            MonitorMethod method = ImportanceSampling) :
        dim_(dim), eval_(eval), method_(method) {}

    Monitor (const Monitor<T> &other) :
        dim_(other.dim_), eval_(other.eval_), method_(other.method_),
        index_(other.index_), record_(other.record_) {}

    Monitor<T> &operator= (const Monitor<T> &other)
    {
        if (&other != this) {
            dim_    = other.dim_;
            eval_   = other.eval_;
            method_ = other.method_;
            index_  = other.index_;
            record_ = other.record_;
        }

        return *this;
    }

    /// \brief The dimension of the monitor
    unsigned dim () const
    {
        return dim_;
    }

    /// \brief The number of iterations has been recorded
    ///
    /// \note This is not necessarily the same as Sampler<T>::iter_size. For
    /// example, a monitor can be added only after a certain time point of the
    /// sampler's iterations.
    unsigned iter_size () const
    {
        return static_cast<unsigned>(index_.size());
    }

    /// \brief Whether the evaluation object is valid
    VSMC_EXPLICIT_OPERATOR operator bool () const
    {
        return bool(eval_);
    }

    /// \brief Get the iteration index of the sampler of a given monitor
    /// iteration
    ///
    /// \details
    /// For example, if a monitor is only added to the sampler at the sampler's
    /// iteration `siter`. Then index(0) will be `siter` and so on. If the
    /// monitor is added before the sampler's initialization and continued to
    /// be evaluated during the iterations, then iter(iter) shall just be
    /// `iter`.
    unsigned index (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Monitor::index** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return index_[iter];
    }

    /// \brief Get the latest Monte Carlo integration record of a given
    /// variable
    ///
    /// \details
    /// For a `Dim` dimension monitor, `id` shall be 0 to `Dim` - 1
    double record (unsigned id) const
    {
        VSMC_RUNTIME_ASSERT((id >= 0 && id < dim()),
                ("CALL **Monitor::record** WITH AN INVALID "
                 "ID NUMBER"));
        VSMC_RUNTIME_ASSERT((iter_size() > 0),
                ("CALL **Monitor::record** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return record_[(iter_size() - 1) * dim_ + id];
    }

    /// \brief Get the Monte Carlo integration record of a given variable and
    /// the monitor iteration
    ///
    /// \details
    /// For a `Dim` dimension monitor, `id` shall be 0 to `Dim` - 1
    double record (unsigned id, unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((id >= 0 && id < dim()),
                ("CALL **Monitor::record** WITH AN INVALID "
                 "ID NUMBER"));
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Monitor::record** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return record_[iter * dim_ + id];
    }

    /// \brief Read the index history through an output iterator
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        return std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Read the record history for a given variable through an output
    /// iterator
    template <typename OutputIter>
    OutputIter read_record (unsigned id, OutputIter first) const
    {
        const double *riter = &record_[id];
        for (unsigned i = 0; i != iter_size(); ++i, ++first, riter += dim_)
            *first = *riter;

        return first;
    }

    /// \brief Read the record history of all variables through an array of
    /// output iterators
    ///
    /// \param first A pointer to an array of output iterators
    ///
    /// \details
    /// The record of variable `id` will be read through `first[id]`
    template <typename OutputIter>
    void read_record_matrix (OutputIter *first) const
    {
        for (unsigned d = 0; d != dim_; ++d)
            read_record(d, first[d]);
    }

    /// \brief Read the record history of all variables through an output
    /// iterator
    ///
    /// \param order Either ColMajor or RowMajor
    /// \param first The output iterator
    ///
    /// \note For example, say `first` is of type `double *`, then if `order ==
    /// ColMajor`, then, `first[j * iter_size() + i] == record(i, j)`.
    /// Otherwise, if `order == RowMajor`, then `first[i * dim() + j] ==
    /// record(i, j)`. That is, the output is an `iter_size()` by `dim()`
    /// matrix, with the usual meaning of column or row major order.
    template <typename OutputIter>
    OutputIter read_record_matrix (MatrixOrder order, OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT((order == ColMajor || order == RowMajor),
                "CALL **Monitor::read_record_matrix** with and INVALID "
                "MatrixOrder");

        if (order == ColMajor)
            for (unsigned d = 0; d != dim_; ++d)
                first = read_record(d, first);

        if (order == RowMajor)
            first = std::copy(record_.begin(), record_.end(), first);

        return first;
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval (const eval_type &new_eval,
            MonitorMethod method = ImportanceSampling)
    {
        eval_ = new_eval;
        method_ = method;
    }

    /// \brief Perform the evaluation for a given iteration and a Particle<T>
    /// object
    void eval (unsigned iter, const Particle<T> &particle)
    {
        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("CALL **Monitor::eval** WITH AN INVALID "
                 "EVALUATION FUNCTOR"));

        result_.resize(dim_);
        switch (method_) {
            case ImportanceSampling :
                weight_.resize(particle.size());
                buffer_.resize(dim_ * particle.size());
                eval_(iter, dim_, particle, &buffer_[0]);
                particle.read_weight(&weight_[0]);
                dgemv_(RowMajor, Trans,
                        static_cast<typename dgemv_type::size_type>(
                            particle.size()),
                        static_cast<typename dgemv_type::size_type>(dim_),
                        1, &buffer_[0], dim_, &weight_[0], 1, 0,
                        &result_[0], 1);
                break;
            case Simple :
                eval_(iter, dim_, particle, &result_[0]);
                break;
        }

        index_.push_back(iter);
        for (unsigned d = 0; d != dim_; ++d)
            record_.push_back(result_[d]);
    }

    /// \brief Clear all records of the index and integrations
    void clear ()
    {
        index_.clear();
        record_.clear();
    }

    private :

    unsigned dim_;
    eval_type eval_;
    MonitorMethod method_;
    std::vector<unsigned> index_;
    std::vector<double> record_;
    std::vector<double> result_;
    std::vector<double> weight_;
    std::vector<double> buffer_;

    dgemv_type dgemv_;
}; // class Monitor

} // namespace vsmc

#endif // VSMC_CORE_MONITOR_HPP
