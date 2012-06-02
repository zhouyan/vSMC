#ifndef V_SMC_CORE_MONITOR_HPP
#define V_SMC_CORE_MONITOR_HPP

#include <vSMC/internal/common.hpp>

namespace vSMC {

/// \brief Monitor for Monte Carlo integration
/// \ingroup Core
///
/// \tparam T Particle<T>::value_type
template <typename T>
class Monitor
{
    public :

    /// \brief The type of evaluation functor
    ///
    /// \details
    /// There are two types of evaluation functor, which unfortunately cannot
    /// be distinguished by their C++ types. The primary use of Monitor is to
    /// record the importance sampling integration along the way of iterations.
    /// So say one want to monitor two parameters, \f$x = E[g(\theta)]\f$ and
    /// \f$y = E[h(\theta)]\f$, and this is done through importance sampling
    /// integration. Then one need first compute two vectors, \f$\{x_i\}\f$ and
    /// \f$\{y_i\}\f$ where \f$x_i = g(\theta_i)\f$
    /// and \f$y_i = h(\theta_i)\f$, and then compute the weighted sum.
    /// With Monitor there are two ways to do this.
    ///
    /// First, called \b non-direct type monitor, one can create a 2 dimension
    /// monitor. When the evaluation function is called, the last output
    /// arguments, say \c buffer will be a row major matrix of dimension N by 2
    /// where N is the number of particles. That is, <tt>buffer[i * 2] =
    /// xi</tt> and <tt>buffer[i * 2 + 1] = yi</tt>. After that, the Monitor
    /// will take care of the imporatance sampling. This is called the
    /// Non-direct type evaluation.
    ///
    /// The other way, called \b direct type monitor, is the user can return
    /// the results final results direclty, and simply return the values
    /// directly and having the job of the Monitor is simply to archive these
    /// information and be ready to provide them after we are done with all the
    /// iterations. In this case, one still create a 2 dimension monitor, but
    /// now the previous \c buffer will be an array of length 2. The other use
    /// of this kind of monitor is that one are not monitoring the importance
    /// sampling estimates at all but some other quantities of the sampler.
    ///
    /// Which kind of evaluation to be used are determined by the way the
    /// monitro is constructed. After the construction, it cannot be changed.
    /// See the documentation of the constructor.
    ///
    /// Ideally we shall create two classes for these two types of Monitors.
    /// This is a much better practice is the view of programming since it
    /// makes the interface more difficult to use wrongly. However, that also
    /// means we need a lot more methods in the Sampler which manages all
    /// monitors and it can be much more difficult. Another solution is to
    /// requires the evaluation functors to have some type traits. However that
    /// means you need to specifialize the traits for your own type of
    /// evluation functors, and more difficult if you want to use a simple
    /// function pointer.
    ///
    /// The current way vSMC do these kind of things is following a "least
    /// surprise" principle. If one come from BUGS, then he knows what \c
    /// monitor does, recording \c x_i and \c y_i as in the non-direct type
    /// monitors. And this is the default behavior of Monitor. In fact, if one
    /// check out the Helper module, all backends only provide non-direct type
    /// monitor.
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
/// \ingroup Core
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
