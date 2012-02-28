#ifndef V_SMC_CORE_MONITOR_HPP
#define V_SMC_CORE_MONITOR_HPP

#include <vector>
#include <cstddef>
#include <mkl_cblas.h>
#include <boost/function.hpp>
#include <vSMC/core/buffer.hpp>
#include <vSMC/core/particle.hpp>

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
    Monitor (const integral_type &integral = NULL, unsigned dim = 1) :
        integral_(integral), dim_(dim) {}

    /// \brief Copy constructor
    ///
    /// \param monitor The Monitor to by copied
    Monitor (const Monitor<T> &monitor) :
        integral_(monitor.integral_), dim_(monitor.dim_),
        index_(monitor.index_), record_(monitor.record_) {}

    /// \brief Assignment operator
    ///
    /// \param monitor The Monitor to be assigned
    /// \return The Monitor after assignemnt
    Monitor<T> & operator= (const Monitor<T> &monitor)
    {
        if (&monitor != this) {
            integral_ = monitor.integral_;
            dim_ = monitor.dim_;
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
    const std::vector<internal::Buffer<double> > &record () const
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
        buffer_.resize(particle.size() * dim_);
        result_.resize(dim_);
        for (unsigned d = 0; d != dim_; ++d)
            result_[d] = 0;

        integral_(iter, particle, buffer_);
        cblas_dgemv(CblasRowMajor, CblasTrans, particle.size(), dim_,
                1, buffer_, dim_, particle.weight_ptr(), 1, 0, result_, 1);

        index_.push_back(iter);
        record_.push_back(result_);
    }

    /// \brief Clear all recorded data
    void clear ()
    {
        index_.clear();
        record_.clear();
    }

    private :

    internal::Buffer<double> buffer_;
    internal::Buffer<double> result_;
    integral_type integral_;
    unsigned dim_;
    std::vector<std::size_t> index_;
    std::vector<internal::Buffer<double> > record_;
}; // class Monitor

} // namespace vSMC

#endif // V_SMC_CORE_MONITOR_HPP
