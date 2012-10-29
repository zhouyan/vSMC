#ifndef VSMC_CORE_MONITOR_HPP
#define VSMC_CORE_MONITOR_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief A simple GEMV functor for Monitor
///
/// \details
/// The actual GEMV used by Monitor<T> is based in GEMVTypeTrait<T>::type,
/// which default to this one. A replacement shall provide the same interface
/// documented below. The functor does not have to be \c const. However it
/// does need to have a default constructor
class GEMVSimple
{
    public :

    typedef VSMC_SIZE_TYPE size_type;

    /// \brief Simple GEMV operator
    ///
    /// \param N Number of columns of A and elements of X (number of particles)
    /// \param M Number of rows of A (Monitor's dimension)
    /// \param A [in] The matrix
    /// \param X [in] The vector
    /// \param res [out] Results
    ///
    /// \note `A` is assumed to be column major in the sense that `A[0]` is
    /// A_{0,0}, `A[1]` is A_{1,0}, etc. That is, the if you iterate over
    /// pointer `A` linearly with increments `1`, then it will first travese
    /// column one, then column two, and so on. If you want to think `A` as an
    /// `N` by `M` instead of `M` by `N` matrix, then `A` is assumed to be row
    /// major.
    void operator() (size_type N, size_type M,
            const double *A, const double *X, double *res) const
    {
        for (size_type m = 0; m != M; ++m) {
            double r = 0;
            const double *a = &A[m];
            const double *x = X;
            for (size_type n = 0; n != N; ++n, ++x, a += M)
                r += (*a) * (*x);
            res[m] = r;
        }
    }
}; // class GEMVSimple

} // namespace vsmc

VSMC_DEFINE_TYPE_DISPATCH_TRAIT(GEMVType, gemv_type, GEMVSimple);

namespace vsmc {

/// \brief Monitor for Monte Carlo integration
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
///
/// The primary use of Monitor is to record the importance sampling integration
/// along the way of iterations. So say one want to monitor two parameters,
/// \f$x = E[g(\theta)]\f$ and \f$y = E[h(\theta)]\f$, and this is done through
/// importance sampling integration. Then one need first compute two vectors,
/// \f$\{x_i\}\f$ and \f$\{y_i\}\f$ where \f$x_i = g(\theta_i)\f$ and \f$y_i =
/// h(\theta_i)\f$, and then compute the weighted sum. With Monitor one can
/// create a 2 dimension monitor with an evaluation functor. When the
/// evaluation functor is called, the last output arguments, say \c buffer,
/// shall be a row major matrix of dimension N by 2 where N is the number of
/// particles. That is, <tt>buffer[i * 2] = xi</tt> and <tt>buffer[i * 2 + 1] =
/// yi</tt>. After that, the Monitor will take care of the imporatance
/// sampling.
///
/// A Monitor is sually not used as a standalone object, but added to a
/// Sampler. It is possible to have the monitor only exist for parts of the
/// iterations of a Sampler. Therefore, the iteration in a monitor is not the
/// same as the iteration number of a Sampler. Which itertions of the Sampler
/// are recored with a particular Monitor can be queried by `index()`. For each
/// `i`, `index(i)` is the iteration number of the Sampler being monitored.
template <typename T>
class Monitor
{
    public :

    /// The type of the particle values
    typedef T value_type;

    /// The type of evaluation functor
    typedef cxx11::function<void (
            unsigned, unsigned, const Particle<T> &, double *)> eval_type;

    /// The type of the GEMV functor
    typedef typename GEMVTypeTrait<T>::type gemv_type;

    /// \brief Construct a Monitor with an evaluation functor
    ///
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param eval The functor used to evaluate the results
    explicit Monitor (unsigned dim = 1, const eval_type &eval = VSMC_NULLPTR) :
        dim_(dim), eval_(eval) {}

    Monitor (const Monitor<T> &other) :
        dim_(other.dim_), eval_(other.eval_),
        index_(other.index_), record_(other.record_) {}

    Monitor<T> &operator= (const Monitor<T> &other)
    {
        if (&other != this) {
            dim_    = other.dim_;
            eval_   = other.eval_;
            index_  = other.index_;
            record_ = other.record_;
        }

        return *this;
    }

    /// Dimension of the monitor
    unsigned dim () const
    {
        return dim_;
    }

    /// The size of records
    unsigned iter_size () const
    {
        return static_cast<unsigned>(index_.size());
    }

    /// \brief Test if the monitor is valid
    ///
    /// \note This operator will be \c explicit if the C++11 feature is enabled
#if VSMC_HAS_CXX11_EXPLICIT_CONVERSIONS
    explicit
#endif
        operator bool () const
    {
        return bool(eval_);
    }

    unsigned index (unsigned iter) const
    {
        VSMC_RUNTIME_ASSERT((iter >= 0 && iter < iter_size()),
                ("CALL **Monitor::index** WITH AN INVALID "
                 "ITERATION NUMBER"));

        return index_[iter];
    }

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

    /// \brief Read only access to iteration index
    ///
    /// \param first The beginning of the destination range
    ///
    /// \return Output iterator to the element in the destination range, one
    /// past the last element copied
    template <typename OutputIter>
    OutputIter read_index (OutputIter first) const
    {
        return std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Read only access to record of a specific variable
    ///
    /// \param id The ID of the variable, 0 to dim() - 1
    /// \param first The beginning of the destination range
    ///
    /// \return Output iterator to the element in the destination range, one
    /// past the last element copied
    template <typename OutputIter>
    OutputIter read_record (unsigned id, OutputIter first) const
    {
        const double *riter = &record_[id];
        for (unsigned i = 0; i != iter_size(); ++i, ++first, riter += dim_)
            *first = *riter;

        return first;
    }

    /// \brief Read only access to record of all variables
    ///
    /// \param first A pointer to an array of the beginning of the destination
    /// range. For example, say \c OutpuIiter is \c double \c *, then
    /// first[c][r] will be the r'th record of the c'th variable. In general,
    /// first[c] will be the begin of the reading of the record of the c'th
    /// variable.
    template <typename OutputIter>
    void read_record_matrix (OutputIter *first) const
    {
        for (unsigned d = 0; d != dim_; ++d)
            read_record(d, first[d]);
    }

    /// \brief Read only access to record of all variables
    ///
    /// \param order Either vsmc::ColumnMajor or vsmc::RowMajor. With any other
    /// value, this method does nothing and simply return the original \c
    /// first. In debug mode an assert will also be issued. This parameter
    /// affects how values are stored in the output array. In either case, the
    /// records are considered as an `iter_size()` by `dim()` matrix. That is
    /// if `order = vsmc::RowMajor`, and `first` is a random access iterator
    /// (e.g., a pointer), then `first[0] = record(0, 0)`,
    /// `first[1] = record(1, 0)`, ... ,
    /// `first[dim() - 1] = record(dim() - 1, 0)`,
    /// `first[dim()] = record[0, 1]`, ...
    /// \param first The beginning of the destination range
    ///
    /// \return Output iterator to the element in the destination range, one
    /// past the last element copied
    ///
    /// \note The record is considered as a iter_size() by dim() matrix
    template <typename OutputIter>
    OutputIter read_record_matrix (MatrixOrder order, OutputIter first) const
    {
        VSMC_RUNTIME_ASSERT((order == ColumnMajor || order == RowMajor),
                "CALL **Monitor::read_record_matrix** with and INVALID "
                "MatrixOrder");

        if (order == ColumnMajor)
            for (unsigned d = 0; d != dim_; ++d)
                first = read_record(d, first);

        if (order == RowMajor)
            first = std::copy(record_.begin(), record_.end(), first);

        return first;
    }

    /// Set a new evaluation functor
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// \brief Evaluate
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    ///
    /// \pre The user has to provide the evaluation functor, which will be
    /// called in this member function. The signature of the evaluation functor
    /// is as defined by `eval_type`. The first argument is the iteration
    /// number (`iter`), the second is the dimension of this monitor (`dim()`),
    /// the third is the particle set. The last is the output array, described
    /// below.
    ///
    /// \pre A matrix of size `dim()` by `N` is passed to the evaluation
    /// functor used to construct this monitor or set by set_eval() as its last
    /// output parameter, where N is the number of particles. The array of
    /// matrix elements is of column major. Another interpretation is that the
    /// passed array is a row major `N` by `dim()` matrix, each row stores the
    /// results of a particle. Either way, results for the same particle are
    /// stored in a contiguous memory segment.
    void eval (unsigned iter, const Particle<T> &particle)
    {
        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("CALL **Monitor::eval** WITH AN INVALID "
                 "EVALUATION FUNCTOR"));

        buffer_.resize(dim_ * particle.size());
        result_.resize(dim_);
        weight_.resize(particle.size());
        eval_(iter, dim_, particle, &buffer_[0]);
        particle.read_weight(&weight_[0]);
        gemv_(particle.size(), dim_, &buffer_[0], &weight_[0], &result_[0]);

        index_.push_back(iter);
        for (unsigned d = 0; d != dim_; ++d)
            record_.push_back(result_[d]);
    }

    /// \brief Clear all recorded data
    ///
    /// \note The evaluation functor is not reset
    void clear ()
    {
        index_.clear();
        record_.clear();
    }

    private :

    std::vector<double> buffer_;
    std::vector<double> result_;
    std::vector<double> weight_;
    unsigned dim_;
    eval_type eval_;
    std::vector<unsigned> index_;
    std::vector<double> record_;

    gemv_type gemv_;
}; // class Monitor

} // namespace vsmc

#endif // VSMC_CORE_MONITOR_HPP
