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

    /// The type of monitor integral functor
    typedef internal::function<void (
            unsigned, const Particle<T> &, double *)> integral_type;

    /// The type of the index vector
    typedef std::vector<unsigned> index_type;

    /// The type of the record vector
    typedef std::vector<Eigen::VectorXd> record_type;

    /// \brief Construct a Monitor with an integral function
    ///
    /// \param dim The dimension of the monitor, i.e., the number of variables
    /// \param integral The functor used to compute the integrands
    explicit Monitor (unsigned dim = 1, const integral_type &integral = NULL) :
        dim_(dim), integral_(integral) {}

    /// \brief Copy constructor
    ///
    /// \param monitor The Monitor to by copied
    Monitor (const Monitor<T> &monitor) :
        dim_(monitor.dim_), integral_(monitor.integral_),
        index_(monitor.index_), record_(monitor.record_) {}

    /// \brief Assignment operator
    ///
    /// \param monitor The Monitor to be assigned
    ///
    /// \return The Monitor after assignemnt
    Monitor<T> & operator= (const Monitor<T> &monitor)
    {
        if (&monitor != this) {
            dim_ = monitor.dim_;
            integral_ = monitor.integral_;
            index_ = monitor.index_;
            record_ = monitor.record_;
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
    index_type::size_type iter_size () const
    {
        return index_.size();
    }

    /// \brief Set the integral functor
    ///
    /// \param dim The dimension of the monitor
    /// \param integral The functor used to compute the integrands
    void integral (unsigned dim, const integral_type &integral)
    {
        dim_ = dim;
        integral_ = integral;
    }

    /// \brief Test if the monitor is empty
    ///
    /// \return \b true if the monitor is empty
    bool empty () const
    {
        return !bool(integral_);
    }

    /// \brief Iteration index
    ///
    /// \return A const reference to the index
    const index_type &index () const
    {
        return index_;
    }

    /// \brief Record of Monte Carlo integration
    ///
    /// \return A const reference to the record
    const record_type &record () const
    {
        return record_;
    }

    /// \brief Evaluate the integration
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    ///
    /// \note The integral function has to be set through either the
    /// constructor or integral() to a non-NULL value before calling eval().
    /// Otherwise exception will be raised when calling eval().
    void eval (unsigned iter, const Particle<T> &particle)
    {
        buffer_.resize(dim_, particle.size());
        integral_(iter, particle, buffer_.data());
        result_.noalias() = buffer_ * particle.weight();

        index_.push_back(iter);
        record_.push_back(result_);
    }

    /// \brief Clear all recorded data
    ///
    /// \note The integral function is not reset
    void clear ()
    {
        index_.clear();
        record_.clear();
    }

    private :

    Eigen::MatrixXd buffer_;
    Eigen::VectorXd result_;
    unsigned dim_;
    integral_type integral_;
    index_type index_;
    record_type record_;
}; // class Monitor

} // namespace vSMC

#endif // V_SMC_CORE_MONITOR_HPP
