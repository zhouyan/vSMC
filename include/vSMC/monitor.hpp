#ifndef V_SMC_MONITOR_HPP
#define V_SMC_MONITOR_HPP

#include <vSMC/config.hpp>

#include <utility>
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
    /// The type of the values
    typedef std::pair<std::vector<std::size_t>, std::vector<double> > 
        value_type;
    /// The type of the index
    typedef std::vector<std::size_t> index_type;
    /// The type of the record
    typedef std::vector<double> record_type;

    /// \brief Construct a Monitor with an integral function
    ///
    /// \param N The size of the particle set
    /// \param monitor_integral The function used to compute the integrands
    Monitor (std::size_t N, const integral_type &integral = NULL) :
        buffer_(N), integral_(integral) {}

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
    /// \note The integral functor has to be set through either the
    /// constructor or set_integral() to a non-NULL value before calling
    /// eval(). Otherwise runtime_error exception will be raised when calling
    /// eval().
    void eval (std::size_t iter, Particle<T> &particle)
    {
        integral_(iter, particle, buffer_);
        index.push_back(iter);
        record.push_back(cblas_ddot(particle.size(),
                particle.get_weight_ptr(), 1, buffer_, 1));
    }

    /// \brief Get the iteration index
    ///
    /// \return A vector of the index
    index_type get_index () const
    {
        return index;
    }

    /// \brief Get the record of Monte Carlo integrations
    ///
    /// \return A vector of the record
    record_type get_record () const
    {
        return record;
    }

    /// \brief Get both the index and the record of Monte Carlo integrations
    ///
    /// \return A pair of two vectors, index and record
    value_type get_value () const
    {
        return std::make_pair(index, record);
    }

    /// \brief Clear the index and record
    void clear ()
    {
        index.clear();
        record.clear();
    }

    private :

    vDist::tool::Buffer<double> buffer_;
    integral_type integral_;
    std::vector<std::size_t> index;
    std::vector<double> record;
}; // class Monitor

} // namespace vSMC

#endif // V_SMC_MONITOR_HPP
