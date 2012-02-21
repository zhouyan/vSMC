#ifndef V_SMC_MONITOR_HPP
#define V_SMC_MONITOR_HPP

#include <vSMC/config.hpp>

#include <vector>
#include <cstddef>
#include <mkl_cblas.h>
#include <boost/function.hpp>
#include <vDist/tool/buffer.hpp>
#include <vSMC/particle.hpp>

namespace vSMC {

/// \brief Monitor for Monte Carlo integration
///
/// Monitor record the Monte Carlo integration of certain function width when
/// the Sampler progress. It is also used for retrieve information of
/// integrations after the SMC iterations. A Sampler class can keep a std::map
/// of zero, one or more Monitors
template <typename T>
class Monitor
{
    public :

    /// The type of monitor integral functor
    typedef boost::function<void
        (std::size_t, Particle<T> &, double *)> integral_type;

    /// \brief Construct a Monitor with an integral function
    ///
    /// \param integral The functor used to compute the integrands
    Monitor (const integral_type &integral = NULL) : integral_(integral) {}

    /// \brief Copy constructor
    ///
    /// \param monitor The Monitor to by copied
    Monitor (const Monitor<T> &monitor) :
        integral_(monitor.integral_),
        index_(monitor.index_), record_(monitor.record_) {}

    /// \brief Assignment operator
    ///
    /// \param monitor The Monitor to be assigned
    /// \return The Monitor after assignemnt
    Monitor<T> & operator= (const Monitor<T> &monitor)
    {
        if (&monitor != this) {
            integral_ = monitor.integral_;
            index_ = monitor.index_;
            record_ = monitor.record_;
        }

        return *this;
    }

    /// \brief Size of records
    ///
    /// \return The number of iterations recorded
    std::size_t iter_size () const
    {
        return index_.size();
    }

    /// \brief Set the integral functor
    ///
    /// \param integral The functor used to compute the integrands
    void integral (const integral_type &integral)
    {
        integral_ = integral;
    }

    /// \brief Test if the monitor is empty
    ///
    /// \return \b true if the monitor is empty
    bool empty () const
    {
        return integral_.empty();
    }

    /// \brief Iteration index
    ///
    /// \return A const reference to the index
    const std::vector<std::size_t> &index () const
    {
        return index_;
    }

    /// \brief Iteration index
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void index (OIter first) const
    {
        for (std::vector<std::size_t>::const_iterator iter = index_.begin();
               iter != index_.end(); ++iter)
            *first++ = *iter;
    }

    /// \brief Record of Monte Carlo integration
    ///
    /// \return A const reference to the record
    const std::vector<double> &record () const
    {
        return record_;
    }

    /// \brief Record of Monte Carlo integrtion.
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void record (OIter first) const
    {
        for (std::vector<double>::const_iterator iter = record_.begin();
               iter != record_.end(); ++iter)
            *first++ = *iter;
    }

    /// \brief Evaluate the integration
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    ///
    /// \note The integral function has to be set through either the
    /// constructor or integral() to a non-NULL value before calling eval().
    /// Otherwise runtime_error exception will be raised when calling eval().
    ///
    /// \see Documentation for Boost::function
    void eval (std::size_t iter, Particle<T> &particle)
    {
        buffer_.resize(particle.size());
        integral_(iter, particle, buffer_);
        index_.push_back(iter);
        record_.push_back(cblas_ddot(particle.size(),
                particle.weight_ptr(), 1, buffer_, 1));
    }

    /// \brief Clear all recorded data
    void clear ()
    {
        index_.clear();
        record_.clear();
    }

    private :

    vDist::tool::Buffer<double> buffer_;
    integral_type integral_;
    std::vector<std::size_t> index_;
    std::vector<double> record_;
}; // class Monitor

} // namespace vSMC

#endif // V_SMC_MONITOR_HPP
