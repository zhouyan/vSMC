#ifndef V_SMC_CORE_MONITOR_HPP
#define V_SMC_CORE_MONITOR_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief Monitor for Monte Carlo integration
///
/// \tparam T State state type. Requiment:
/// \li Consturctor: T (IntType N)
/// \li Method: copy (IntType from, IntType to)
template <typename T>
class Monitor
{
    public :

    /// The type of evaluation functor
    typedef function<void (unsigned, const Particle<T> &, double *)> eval_type;

    /// The type of the index vector
    typedef std::deque<unsigned> index_type;

    /// The type of the record vector
    typedef std::vector<std::deque<double> > record_type;

    /// \brief Construct a Monitor with an evaluation functor
    ///
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param eval The functor used to evaluate the results
    /// \param direct Whether or not eval return the integrands or the final
    /// results
    explicit Monitor (unsigned dim = 1,
            const eval_type &eval = NULL, bool direct = false) :
        dim_(dim), direct_(direct), eval_(eval), record_(dim) {}

    /// \brief Copy constructor
    ///
    /// \param other The Monitor to by copied
    Monitor (const Monitor<T> &other) :
        dim_(other.dim_), direct_(other.direct_), eval_(other.eval_),
        index_(other.index_), record_(other.record_) {}

    /// \brief Assignment operator
    ///
    /// \param other The Monitor to be assigned
    ///
    /// \return The Monitor after assignemnt
    Monitor<T> & operator= (const Monitor<T> &other)
    {
        if (&other != this) {
            dim_      = other.dim_;
            direct_   = other.direct_;
            eval_     = other.eval_;
            index_    = other.index_;
            record_   = other.record_;
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

    /// \brief Test if the monitor is empty
    ///
    /// \return \b true if the monitor is empty
    ///
    /// \note A monitor is empty means it has no valid evaluation functions.
    /// However its record may not be empty (for that purpose call iter_size()
    /// to see). Call eval(iter, particle) on an empty monitor is a runtime
    /// error.
    bool empty () const
    {
        return !bool(eval_);
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
    /// \param new_eval The functor used to directly evaluate the results
    /// \param direct Whether or not eval return the integrands or the final
    void set_eval (const eval_type &new_eval, bool direct = false)
    {
        direct_ = direct;
        eval_ = new_eval;
    }

    /// \brief Evaluate
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    ///
    /// \note The evaluation functor has to be set to a non-NULL value before
    /// calling eval(). The direct evaluation functor is prefered when both are
    /// available.
    void eval (unsigned iter, const Particle<T> &particle)
    {
        assert(eval_);

        if (direct_) {
            result_.resize(dim_);
            eval_(iter, particle, result_.data());
        } else {
            buffer_.resize(dim_, particle.size());
            eval_(iter, particle, buffer_.data());
            result_.noalias() = buffer_ * particle.weight();
        }

        assert(result_.size() >= dim_);
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
    bool direct_;
    eval_type eval_;
    index_type index_;
    record_type record_;
}; // class Monitor

/// \brief Print the Monitor
///
/// \param os The ostream to which the contents are printed
/// \param monitor The Monitor to be printed
///
/// \note This is the same as <tt>monitor.print(os)</tt>
template<typename CharT, typename Traits, typename T>
std::basic_ostream<CharT, Traits> &operator<< (
        std::basic_ostream<CharT, Traits> &os, const vSMC::Monitor<T> &monitor)
{
    monitor.print(os);
    return os;
}

} // namespace vSMC

#endif // V_SMC_CORE_MONITOR_HPP
