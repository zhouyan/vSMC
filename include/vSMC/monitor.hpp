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

template <typename T>
class Monitor
{
    public :

    /// The type of monitor integration
    typedef boost::function<void
        (std::size_t, Particle<T> &, double *)> integral_type;
    /// The type of the index
    typedef std::vector<std::size_t> index_type;
    /// The type of the record
    typedef std::vector<double> record_type;

    /// \brief Construct a Monitor with an integral function
    ///
    /// \param N The size of the particle set
    /// \param integral The function used to compute the integrands
    Monitor (std::size_t N, const integral_type &integral = NULL) :
        buffer_(N), integral_(integral) {}

    /// \brief Size of the particle set
    ///
    /// \return The number of particles
    std::size_t size () const
    {
        return buffer_.size();
    }

    /// \brief Size of records
    ///
    /// \return The number of iterations recorded
    std::size_t iter_size () const
    {
        return index_.size();
    }

    /// \brief Set the integral function
    ///
    /// \param integral The function used to compute the integrands
    void set_integral (const integral_type &integral)
    {
        integral_ = integral;
    }

    /// \brief Test if the monitor is empty
    ///
    /// \return True if the monitor is empty
    bool empty () const
    {
        return integral_.empty();
    }

    /// \brief Evaluate the integration
    ///
    /// \param iter The iteration number
    /// \param particle The particle set to be operated on by eval()
    /// \note The integral function has to be set through either the
    /// constructor or set_integral() to a non-NULL value before calling
    /// eval(). Otherwise runtime_error exception will be raised when calling
    /// eval().
    /// \see Documentation for Boost::function
    void eval (std::size_t iter, Particle<T> &particle)
    {
        integral_(iter, particle, buffer_);
        index_.push_back(iter);
        record_.push_back(cblas_ddot(particle.size(),
                particle.get_weight_ptr(), 1, buffer_, 1));
    }

    /// \brief Get the iteration index
    ///
    /// \return A vector of the index
    index_type get_index () const
    {
        return index_;
    }

    /// \brief Get the iteration index
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void get_index (OIter first) const
    {
        for (index_type::const_iterator i = index_.begin();
               i != index_.end(); ++i)
            *first++ = *i;
    }

    /// \brief Get the record of Monte Carlo integration
    ///
    /// \return A vector of the record
    record_type get_record () const
    {
        return record_;
    }

    /// \brief Get the record of Monte Carlo integrtion.
    ///
    /// \param first An iterator point to where writing starts
    template<typename OIter>
    void get_record (OIter first) const
    {
        for (record_type::const_iterator i = record_.begin();
               i != record_.end(); ++i)
            *first++ = *i;
    }

    /// \brief Clear the index and record
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
