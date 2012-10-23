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
    /// \note \c A and \c X are assumed to be column major.
    void operator() (size_type N, size_type M,
            const double *A, const double *X, double *res) const
    {
        for (size_type m = 0; m != M; ++m) {
            double r = 0;
            for (size_type n = 0; n != N; ++n)
                r += X[n] * A[n * M + m];
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
/// \f$\{x_i\}\f$ and \f$\{y_i\}\f$ where \f$x_i = g(\theta_i)\f$ and
/// \f$y_i = h(\theta_i)\f$, and then compute the weighted sum. With Monitor
/// one can create a 2 dimension monitor with an evaluation functor. When the
/// evaluation functro is called, the last output arguments, say \c buffer will
/// be a row major matrix of dimension N by 2 where N is the number of
/// particles. That is, <tt>buffer[i * 2] = xi</tt> and <tt>buffer[i * 2 + 1] =
/// yi</tt>. After that, the Monitor will take care of the imporatance
/// sampling.
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
        dim_(dim), eval_(eval), record_(dim) {}

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

    /// Read only access to iteration index
    template <typename OutputIter>
    void read_index (OutputIter first) const
    {
        std::copy(index_.begin(), index_.end(), first);
    }

    /// \brief Read only access to record of importance sampling integration
    ///
    /// \param first A pointer to an array of begins of output.
    /// For example, say \c OutpuIiter is \c double \c *, then first[c][r]
    /// will be the r'th record of the c'th variable. In general, first[c]
    /// will be the begin of the reading of the record of the c'th variable.
    template <typename OutputIter>
    void read_record (OutputIter *first) const
    {
        for (unsigned d = 0; d != dim_; ++d)
            std::copy(record_[d].begin(), record_[d].end(), first[d]);
    }

    /// Read only access to record of a specific variable
    template <typename OutputIter>
    void read_record (unsigned id, OutputIter first) const
    {
        std::copy(record_[id].begin(), record_[id].end(), first);
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
    /// \pre A matrix of size Dim by N is passed to the evaluation functor used
    /// to construct this monitor or set by set_eval() as its last output
    /// parameter, where N is the number of particles. The array of matrix
    /// elements is of row major.
    void eval (unsigned iter, const Particle<T> &particle)
    {
        VSMC_RUNTIME_ASSERT((bool(eval_)),
                ("CALL **Monitor::eval** WITH AN INVALID "
                 "EVALUATION FUNCTOR"));

        buffer_.resize(dim_ * particle.size());
        result_.resize(dim_);
        weight_.resize(particle.size());
        eval_(iter, dim_, particle, &buffer_[0]);
        particle.read_weight(weight_.begin());
        gemv_(particle.size(), dim_, &buffer_[0], &weight_[0], &result_[0]);

        index_.push_back(iter);
        for (unsigned d = 0; d != dim_; ++d)
            record_[d].push_back(result_[d]);
    }

    gemv_type &gemv ()
    {
        return gemv_;
    }

    const gemv_type &gemv () const
    {
        return gemv_;
    }

    /// \brief Clear all recorded data
    ///
    /// \note The evaluation functor is not reset
    void clear ()
    {
        index_.clear();
        for (std::vector<std::vector<double> >::iterator r = record_.begin();
                r != record_.end(); ++r) {
            r->clear();
        }
    }

    private :

    std::vector<double> buffer_;
    std::vector<double> result_;
    std::vector<double> weight_;
    unsigned dim_;
    eval_type eval_;
    std::vector<unsigned> index_;
    std::vector<std::vector<double> > record_;

    gemv_type gemv_;
}; // class Monitor

} // namespace vsmc

#endif // VSMC_CORE_MONITOR_HPP
