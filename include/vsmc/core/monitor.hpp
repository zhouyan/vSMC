#ifndef VSMC_CORE_MONITOR_HPP
#define VSMC_CORE_MONITOR_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/integrate/is_integrate.hpp>

#define VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(func) \
    VSMC_RUNTIME_ASSERT((id >= 0 && id < this->dim()),                       \
            ("**Monitor::"#func"** INVALID ID NUMBER ARGUMENT"))

#define VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(func) \
    VSMC_RUNTIME_ASSERT((iter >= 0 && iter < this->iter_size()),             \
            ("**Monitor::"#func"** INVALID ITERATION NUMBER ARGUMENT"))

#define VSMC_RUNTIME_ASSERT_CORE_MONITOR_FUNCTOR(func, caller, name) \
    VSMC_RUNTIME_ASSERT(bool(func),                                          \
            ("**Monitor::"#caller"** INVALID "#name" OBJECT"))               \

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

    /// \brief Construct a Monitor with an evaluation object
    ///
    /// \param dim The dimension of the Monitor, i.e., the number of variables
    /// \param eval The evaluation object of type Monitor::eval_type
    ///
    /// The evaluation object has the signature
    /// \code
    /// void eval (std::size_t iter, std::size_t dim, const Particle<T> &particle, double *result)
    /// \endcode
    /// where the first three arguments are passed in by the Sampler at the
    /// end of each iteration. The evaluation occurs after the possible MCMC
    /// moves. The output parameter `result` shall contain the results of the
    /// evaluation.
    ///
    /// The array `result` is of length `particle.size() * dim`, and it
    /// represents a row major matrix of dimension `particle.size()` by `dim`,
    /// say \f$R\f$. Let \f$W\f$ be the vector of the normalized weights. The
    /// Monitor will be respoinsible to compute the importance sampling
    /// estimate \f$r = R^TW\f$ and record it. For example, say the purpose of
    /// the Monitor is to record the importance sampling estimates of
    /// \f$E[h(X)]\f$ where \f$h(X) = (h_1(X),\dots,h_d(X))\f$. Then `result`
    /// shall contain the evaluation of \f$h(X_i)\f$ for each \f$i\f$ from `0`
    /// to `particle.size() - 1` in the order
    /// \f$(h_1(X_0), \dots, h_d(X_0), h_1(X_1), \dots, h_d(X_1), \dots)\f$.
    ///
    /// After each evaluation, the iteration number `iter` and the imporatance
    /// sampling estimates are recorded and can be retrived by `index()` and
    /// `record()`.
    explicit Monitor (std::size_t dim, const eval_type &eval) :
        dim_(dim), eval_(eval), recording_(true) {}

    Monitor (const Monitor<T> &other) :
        dim_(other.dim_), eval_(other.eval_), recording_(other.recording_),
        index_(other.index_), record_(other.record_),
        is_integrate_(other.is_integrate_) {}

    Monitor<T> &operator= (const Monitor<T> &other)
    {
        if (this != &other) {
            dim_          = other.dim_;
            eval_         = other.eval_;
            recording_    = other.recording_;
            index_        = other.index_;
            record_       = other.record_;
            is_integrate_ = other.is_integrate_;
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    Monitor (Monitor<T> &&other) :
        dim_(other.dim_), eval_(cxx11::move(other.eval_)),
        recording_(other.recording_),
        index_(cxx11::move(other.index_)),
        record_(cxx11::move(other.record_)),
        is_integrate_(other.is_integrate_) {}

    Monitor<T> &operator= (Monitor<T> &&other)
    {
        if (this != &other) {
            dim_          = other.dim_;
            eval_         = cxx11::move(other.eval_);
            recording_    = other.recording_;
            index_        = cxx11::move(other.index_);
            record_       = cxx11::move(other.record_);
            is_integrate_ = cxx11::move(other.is_integrate_);
        }

        return *this;
    }
#endif

    virtual ~Monitor () {}

    /// \brief The dimension of the Monitor
    std::size_t dim () const {return dim_;}

    /// \brief The number of iterations has been recorded
    ///
    /// \details
    /// This is not necessarily the same as Sampler<T>::iter_size. For
    /// example, a Monitor can be added only after a certain time point of the
    /// sampler's iterations. Also the Monitor can be turned off for a period
    /// during the iterations.
    std::size_t iter_size () const {return index_.size();}

    /// \brief Reserve space for a specified number of iterations
    void reserve (std::size_t num)
    {
        index_.reserve(num);
        record_.reserve(dim_ * num);
    }

    /// \brief Whether the evaluation object is valid
    bool empty () const {return !bool(eval_);}

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
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(index);

        return index_[iter];
    }

    /// \brief Get the latest Monte Carlo integration record of a given
    /// variable
    ///
    /// \details
    /// For a `dim` dimension Monitor, `id` shall be 0 to `dim` - 1
    double record (std::size_t id) const
    {
        std::size_t iter = iter_size() ? iter_size() - 1 : iter_size();
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(record);
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(record);

        return record_[iter * dim_ + id];
    }

    /// \brief Get the Monte Carlo integration record of a given variable and
    /// the Monitor iteration
    ///
    /// \details
    /// For a `dim` dimension Monitor, `id` shall be 0 to `dim` - 1
    double record (std::size_t id, std::size_t iter) const
    {
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ID(record);
        VSMC_RUNTIME_ASSERT_CORE_MONITOR_ITER(record);

        return record_[iter * dim_ + id];
    }

    /// \brief Read the index history through an output iterator
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        const std::size_t N = index_.size();
        const std::size_t *const iptr = &index_[0];
        for (std::size_t i = 0; i != N; ++i, ++first)
            *first = iptr[i];

        return first;
    }

    /// \brief Read the record history for a given variable through an output
    /// iterator
    template <typename OutputIter>
    OutputIter read_record (std::size_t id, OutputIter first) const
    {
        const std::size_t N = iter_size();
        const double *riter = &record_[id];
        for (std::size_t i = 0; i != N; ++i, ++first, riter += dim_)
            *first = *riter;

        return first;
    }

    /// \brief Read the record history of all variables through an array of
    /// output iterators
    ///
    /// \param first An iterator of container of output iterators
    template <typename OutputIterIter>
    void read_record_matrix (OutputIterIter first) const
    {
        for (std::size_t d = 0; d != dim_; ++d, ++first)
            read_record(d, *first);
    }

    /// \brief Read the record history of all variables through an output
    /// iterator
    ///
    /// \param first The output iterator
    ///
    /// For example, say `first` is of type `double *`, then if `order ==
    /// ColMajor`, then, `first[j * iter_size() + i] == record(i, j)`.
    /// Otherwise, if `order == RowMajor`, then `first[i * dim() + j] ==
    /// record(i, j)`. That is, the output is an `iter_size()` by `dim()`
    /// matrix, with the usual meaning of column or row major order.
    template <MatrixOrder Order, typename OutputIter>
    OutputIter read_record_matrix (OutputIter first) const
    {
        if (Order == ColMajor)
            for (std::size_t d = 0; d != dim_; ++d)
                first = read_record(d, first);

        if (Order == RowMajor) {
            const std::size_t N = record_.size();
            const double *const rptr = &record_[0];
            for (std::size_t i = 0; i != N; ++i, ++first)
                *first = rptr[i];
        }

        return first;
    }

    /// \brief Set a new evaluation object of type eval_type
    void set_eval (const eval_type &new_eval) {eval_ = new_eval;}

    /// \brief Perform the evaluation for a given iteration and a Particle<T>
    /// object.
    ///
    /// \details
    /// This function is called by a Sampler at the end of each
    /// iteration. It does nothing if `recording()` returns `false`. Otherwise
    /// it use the user defined evaluation object to compute results. When a
    /// Monitor is constructed, `recording()` always returns `true`. It can be
    /// turned off by `turnoff()` and turned on later by `turnon()`.
    void eval (std::size_t iter, const Particle<T> &particle)
    {
        if (!recording_)
            return;

        VSMC_RUNTIME_ASSERT_CORE_MONITOR_FUNCTOR(eval_, eval, EVALUATION);

        const std::size_t N = static_cast<std::size_t>(particle.size());
        double *const result = malloc_result(dim_);
        double *const buffer = malloc_eval_integrand(N * dim_);
        double *const weight = malloc_weight(N);
        particle.read_weight(weight);

        eval_(iter, dim_, particle, buffer);
        if (dim_ == 1) {
            double res = 0;
            for (std::size_t i = 0; i != N; ++i)
                res += buffer[i] * weight[i];
            result[0] = res;
        } else {
            is_integrate_(static_cast<ISIntegrate::size_type>(N),
                    static_cast<ISIntegrate::size_type>(dim_),
                    buffer, weight, result);
        }

        index_.push_back(iter);
        for (std::size_t d = 0; d != dim_; ++d)
            record_.push_back(result[d]);
    }

    /// \brief Clear all records of the index and integrations
    void clear ()
    {
        index_.clear();
        record_.clear();
    }

    /// \brief Whether the Monitor is actively recording results
    bool recording () const {return recording_;}

    /// \brief Turn on the recording
    void turnon () {recording_ = true;}

    /// \brief Turn off the recording
    void turnoff () {recording_ = false;}

    protected :

    virtual double *malloc_result (std::size_t N)
    {
        result_.resize(N);

        return &result_[0];
    }

    virtual double *malloc_weight (std::size_t N)
    {
        weight_.resize(N);

        return &weight_[0];
    }

    virtual double *malloc_eval_integrand (std::size_t N)
    {
        buffer_.resize(N);

        return &buffer_[0];
    }

    private :

    std::size_t dim_;
    eval_type eval_;
    bool recording_;
    std::vector<std::size_t> index_;
    std::vector<double> record_;
    std::vector<double> result_;
    std::vector<double> weight_;
    std::vector<double> buffer_;
    ISIntegrate is_integrate_;
}; // class Monitor

} // namespace vsmc

#endif // VSMC_CORE_MONITOR_HPP
