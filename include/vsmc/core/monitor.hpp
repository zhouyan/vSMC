#ifndef VSMC_CORE_MONITOR_HPP
#define VSMC_CORE_MONITOR_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/core/integral.hpp>

namespace vsmc {

/// \brief Monitor for Monte Carlo integration
/// \ingroup Core
template <typename T>
class Monitor
{
    public :

    typedef T value_type;
    typedef cxx11::function<
        void (std::size_t, std::size_t, const Particle<T> &, double *)>
        eval_type;
    typedef typename traits::Integral1TypeTrait<T>::type integral1_type;
    typedef typename traits::IntegralDTypeTrait<T>::type integrald_type;

    /// \brief Construct a Monitor with an evaluation object
    ///
    /// \param dim The dimension of the Monitor, i.e., the number of variables
    /// \param eval The evaluation object of type Monitor::eval_type
    /// \param method The method of the Monitor evaluation.
    ///
    /// The evaluation object has the signature
    /// \code
    /// void eval (std::size_t iter, std::size_t dim, const Particle<T> &particle, double *result)
    /// \endcode
    /// where the first three arguments are passed in by the Sampler at the
    /// end of each iteration. The evaluation occurs after the possible MCMC
    /// moves. The output parameter `result` shall contain the results of the
    /// evaluation, whose meaning depend on the Monitor's constructor
    /// parameter `method`, which can have two possible values
    ///
    /// - _ImportanceSampling_ The array `result` is of length
    /// `particle.size() * dim`, and it represents a row major matrix of
    /// dimension `particle.size()` by `dim`, say \f$R\f$. Let \f$W\f$ be
    /// the vector of the normalized weights. The Monitor will be respoinsible
    /// to compute the importance sampling estimate \f$r = R^TW\f$ and record
    /// it. For example, say the purpose of the Monitor is to record the
    /// importance sampling estimates of \f$E[h(X)]\f$ where
    /// \f$h(X) = (h_1(X),\dots,h_d(X))\f$. Then `result` shall contain the
    /// evaluation of \f$h(X_i)\f$ for each \f$i\f$ from `0` to
    /// `particle.size() - 1` in the order
    /// \f$(h_1(X_0), \dots, h_d(X_0), h_1(X_1), \dots, h_d(X_1), \dots)\f$.
    ///
    /// - _Simple_ The evalution object returnt the `dim` length record
    /// directly.
    ///
    /// After each evaluation, the iteration number `iter` and the imporatance
    /// sampling estimates (`method == ImportanceSampling`) or `result`
    /// (`method == `Simple`) are recorded and can be retrived by `index()`
    /// and `record()`.
    explicit Monitor (std::size_t dim, const eval_type &eval,
            MonitorMethod method = ImportanceSampling) :
        dim_(dim), eval_(eval), method_(method), recording_(true),
        var_name_(dim_) {}

    Monitor (const Monitor<T> &other) :
        dim_(other.dim_), eval_(other.eval_), method_(other.method_),
        recording_(other.recording_), var_name_(other.var_name_),
        index_(other.index_), record_(other.record_) {}

    Monitor<T> &operator= (const Monitor<T> &other)
    {
        if (&other != this) {
            dim_       = other.dim_;
            eval_      = other.eval_;
            method_    = other.method_;
            recording_ = other.recording_;
            var_name_  = other.var_name_;
            index_     = other.index_;
            record_    = other.record_;
        }

        return *this;
    }

    /// \brief The dimension of the Monitor
    std::size_t dim () const
    {
        return dim_;
    }

    /// \brief The number of iterations has been recorded
    ///
    /// \details
    /// This is not necessarily the same as Sampler<T>::iter_size. For
    /// example, a Monitor can be added only after a certain time point of the
    /// sampler's iterations. Also the Monitor can be turned off for a period
    /// during the iterations.
    std::size_t iter_size () const
    {
        return index_.size();
    }

    /// \brief Whether the evaluation object is valid
    VSMC_EXPLICIT_OPERATOR operator bool () const
    {
        return bool(eval_);
    }

    /// \brief Read and write access to the variable names of the Monitor
    std::string &var_name (std::size_t id)
    {
        VSMC_RUNTIME_ASSERT((id >= 0 && id < dim_),
                "**vsmc::Monitor::var_name** INVALID ID");

        return var_name_[id];
    }

    /// \brief Read only access to the variable names of the Monitor
    const std::string &var_name (std::size_t id) const
    {
        VSMC_RUNTIME_ASSERT((id >= 0 && id < dim_),
                "**vsmc::Monitor::var_name** INVLID ID");

        return var_name_[id];
    }

    /// \brief Set all variable names
    template <typename InputIter>
    void var_name (InputIter first, InputIter last)
    {
        var_name_.clear();
        for (std::size_t i = 0; i != dim_ && first != last; ++i, ++first)
            var_name_.push_back(*first);
        std::size_t diff = dim_ - var_name_.size();
        for (std::size_t i = 0; i != diff; ++i)
            var_name_.push_back(std::string());
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
    std::size_t index (std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("**Monitor::index** INVALID ITERATION NUMBER"));

        return index_[iter];
    }

    /// \brief Get the latest Monte Carlo integration record of a given
    /// variable
    ///
    /// \details
    /// For a `dim` dimension Monitor, `id` shall be 0 to `dim` - 1
    double record (std::size_t id) const
    {
        VSMC_RUNTIME_ASSERT((id >= 0 && id < dim()),
                ("**Monitor::record** INVALID ID"));
        VSMC_RUNTIME_ASSERT((iter_size() > 0),
                ("**Monitor::record** INVALID ITERATION"));

        return record_[(iter_size() - 1) * dim_ + id];
    }

    /// \brief Get the Monte Carlo integration record of a given variable and
    /// the Monitor iteration
    ///
    /// \details
    /// For a `dim` dimension Monitor, `id` shall be 0 to `dim` - 1
    double record (std::size_t id, std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT((id >= 0 && id < dim()),
                ("**Monitor::record** INVALID ID"));
        VSMC_RUNTIME_ASSERT((iter_size() > 0),
                ("**Monitor::record** INVALID ITERATION"));

        return record_[iter * dim_ + id];
    }

    /// \brief Read the index history through an output iterator
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        for (std::size_t i = 0; i != index_.size(); ++i, ++first)
            *first = index_[i];

        return first;
    }

    /// \brief Read the record history for a given variable through an output
    /// iterator
    template <typename OutputIter>
    OutputIter read_record (std::size_t id, OutputIter first) const
    {
        const double *riter = &record_[id];
        for (std::size_t i = 0; i != iter_size(); ++i, ++first, riter += dim_)
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
        for (std::size_t d = 0; d != dim_; ++d)
            read_record(d, first[d]);
    }

    /// \brief Read the record history of all variables through an output
    /// iterator
    ///
    /// \param order Either ColMajor or RowMajor
    /// \param first The output iterator
    ///
    /// For example, say `first` is of type `double *`, then if `order ==
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
            for (std::size_t d = 0; d != dim_; ++d)
                first = read_record(d, first);

        if (order == RowMajor)
            for (std::size_t i = 0; i != record_.size(); ++i, ++first)
                *first = record_[i];

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
    /// object.
    ///
    /// \details
    /// This functin is called by a `Sampler` at the end of each
    /// iteration. It does nothing if `recording()` returns `false`. Otherwise
    /// it use the user defined evaluation object to compute results. When a
    /// Monitor is constructed, `recording()` always returns `true`. It can be
    /// turned off by `turnoff()` and turned on later by `turnon()`.
    void eval (std::size_t iter, const Particle<T> &particle)
    {
        if (!recording_)
            return;

        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("**Monitor::eval** INVALID EVALUATION OBJECT"));

        result_.resize(dim_);
        if (method_ ==  ImportanceSampling) {
            weight_.resize(particle.size());
            buffer_.resize(particle.size() * dim_);
            eval_(iter, dim_, particle, &buffer_[0]);
            particle.read_weight(&weight_[0]);
            if (dim_ == 1) {
                result_[0] = integral1_(static_cast<typename
                        traits::SizeTypeTrait<integral1_type>::type>(
                            particle.size()), &buffer_[0], &weight_[0]);
            } else {
                integrald_(static_cast<typename
                        traits::SizeTypeTrait<integrald_type>::type>(
                            particle.size()), static_cast<typename
                        traits::SizeTypeTrait<integrald_type>::type>(
                            dim_), &buffer_[0], &weight_[0], &result_[0]);
            }
        } else if (method_ == Simple) {
            eval_(iter, dim_, particle, &result_[0]);
        } else {
            VSMC_RUNTIME_ASSERT(false, "**vsmc::Monitor** INVALID METHOD");
        }

        index_.push_back(iter);
        for (std::size_t d = 0; d != dim_; ++d)
            record_.push_back(result_[d]);
    }

    /// \brief Clear all records of the index and integrations
    void clear ()
    {
        index_.clear();
        record_.clear();
    }

    /// \brief Whether the Montior is actively recording restuls
    bool recording () const
    {
        return recording_;
    }

    /// \brief Turn on the recording
    void turnon ()
    {
        recording_ = true;
    }

    /// \brief Turn off the recording
    void turnoff ()
    {
        recording_ = false;
    }

    private :

    std::size_t dim_;
    eval_type eval_;
    MonitorMethod method_;
    bool recording_;
    std::vector<std::string> var_name_;
    std::vector<std::size_t> index_;
    std::vector<double> record_;
    std::vector<double> result_;
    std::vector<double> weight_;
    std::vector<double> buffer_;
    integral1_type integral1_;
    integrald_type integrald_;
}; // class Monitor

} // namespace vsmc

#endif // VSMC_CORE_MONITOR_HPP
