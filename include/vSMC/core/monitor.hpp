#ifndef VSMC_CORE_MONITOR_HPP
#define VSMC_CORE_MONITOR_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Monitor for Monte Carlo integration
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
///
/// There are two types of evaluation functor, which unfortunately cannot be
/// distinguished by their C++ types. The primary use of Monitor is to record
/// the importance sampling integration along the way of iterations.  So say
/// one want to monitor two parameters, \f$x = E[g(\theta)]\f$ and \f$y =
/// E[h(\theta)]\f$, and this is done through importance sampling
/// integration.  Then one need first compute two vectors, \f$\{x_i\}\f$ and
/// \f$\{y_i\}\f$ where \f$x_i = g(\theta_i)\f$ and \f$y_i = h(\theta_i)\f$,
/// and then compute the weighted sum.  With Monitor one can create a 2
/// dimension monitor. When the evaluation function is called, the last output
/// arguments, say \c buffer will be a row major matrix of dimension N by 2
/// where N is the number of particles. That is, <tt>buffer[i * 2] = xi</tt>
/// and <tt>buffer[i * 2 + 1] = yi</tt>. After that, the Monitor will take
/// care of the imporatance sampling.
template <typename T>
class Monitor
{
    public :

    /// The type of evaluation functor
    typedef internal::function<void (
            unsigned, const Particle<T> &, double *)> eval_type;

    /// The type of the index vector
    typedef std::deque<unsigned> index_type;

    /// The type of the record vector
    typedef std::vector<std::deque<double> > record_type;

    /// \brief Construct a Monitor with an evaluation functor
    ///
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param eval The functor used to evaluate the results
    explicit Monitor (unsigned dim = 1, const eval_type &eval = NULL) :
        dim_(dim), eval_(eval), record_(dim) {}

    /// \brief Copy constructor
    ///
    /// \param other The Monitor to by copied
    Monitor (const Monitor<T> &other) :
        dim_(other.dim_), eval_(other.eval_),
        index_(other.index_), record_(other.record_) {}

    /// \brief Assignment operator
    ///
    /// \param other The Monitor to be assigned
    ///
    /// \return The Monitor after assignemnt
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

    /// \brief Dimension of the monitor
    ///
    /// \return The number of parameters
    unsigned dim () const
    {
        return dim_;
    }

    /// \brief Size of records
    ///
    /// \return The number of iterations recorded
    unsigned iter_size () const
    {
        return index_.size();
    }

    /// \brief Test if the monitor is valid
    ///
    /// \return \b true if the monitor is valid
#ifdef VSMC_HAS_CXX11_EXPLICIT_CONVERSION
    explicit
#endif
        operator bool () const
    {
        return bool(eval_);
    }

    /// \brief Iteration index
    ///
    /// \return A const reference to the index
    const index_type &index () const
    {
        return index_;
    }

    /// \brief Record
    ///
    /// \return A const reference to the record
    ///
    /// \note record()[c][r] will be the r'th record of the c'th variable
    const record_type &record () const
    {
        return record_;
    }

    /// \brief Record of the a specific variable
    ///
    /// \param id The id the variable starting with zero
    ///
    /// \return A const reference to the record or variable id
    const record_type::value_type &record (unsigned id) const
    {
        return record_[id];
    }

    /// \brief Print the index and record matrix
    ///
    /// \param os The ostream to which the contents are printed
    template<typename CharT, typename Traits>
    void print (std::basic_ostream<CharT, Traits> &os = std::cout)
    {
        const char sep = '\t';

        for (unsigned i = 0; i != iter_size(); ++i) {
            os << index_[i] << sep;
            for (unsigned d = 0; d != dim_; ++d)
                os << record_[d][i] << sep;
            if (i + 1 < iter_size())
                os << '\n';
        }
    }

    /// \brief Set a new evaluation functor
    ///
    /// \param new_eval The functor used to evaluate the results
    void set_eval (const eval_type &new_eval)
    {
        eval_ = new_eval;
    }

    /// \brief Evaluate
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    ///
    /// \pre A matrix of size Dim by N is passed to the evaluation functor,
    /// where N is the number of particles. The array of matrix elements is of
    /// row major
    void eval (unsigned iter, const Particle<T> &particle)
    {
        assert(bool(eval_));

        buffer_.resize(dim_, particle.size());
        eval_(iter, particle, buffer_.data());
        result_.noalias() = buffer_ * particle.weight();

        assert(result_.size() == dim_);
        assert(record_.size() == dim_);

        index_.push_back(iter);
        for (unsigned d = 0; d != dim_; ++d)
            record_[d].push_back(result_[d]);
    }

    /// \brief Clear all recorded data
    ///
    /// \note The evaluation functor is not reset
    void clear ()
    {
        index_.clear();
        for (record_type::iterator r = record_.begin();
                r != record_.end(); ++r) {
            r->clear();
        }
    }

    private :

    Eigen::MatrixXd buffer_;
    Eigen::VectorXd result_;
    unsigned dim_;
    eval_type eval_;
    index_type index_;
    record_type record_;
}; // class Monitor

/// \brief Print the Monitor
/// \ingroup Core
///
/// \param os The ostream to which the contents are printed
/// \param monitor The Monitor to be printed
///
/// \note This is the same as <tt>monitor.print(os)</tt>
template<typename CharT, typename Traits, typename T>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const vsmc::Monitor<T> &monitor)
{
    monitor.print(os);
    return os;
}

} // namespace vsmc

#endif // VSMC_CORE_MONITOR_HPP
