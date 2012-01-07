#ifndef V_SMC_MONITOR_HPP
#define V_SMC_MONITOR_HPP

#include <utility>
#include <vector>
#include <cstddef>
#include <gsl/gsl_cblas.h>
#include <boost/function.hpp>
#include <vDist/tool/buffer.hpp>
#include <vSMC/particle.hpp>

namespace vSMC {

template <class T>
class Monitor
{

    public :

    /// The type of monitor integration
    typedef boost::function<void
        (std::size_t, const Particle<T> &, double *)> integral_type;
    /// The type of the values
    typedef std::pair<std::vector<std::size_t>, std::vector<double> > 
        value_type;
    /// The type of the index
    typedef std::vector<std::size_t> index_type;
    /// The type of the record
    typedef std::vector<double> record_type;

    Monitor (std::size_t N = 1) : buffer(N) {}

    Monitor (const integral_type &monitor_integral, std::size_t N = 1) :
        integral(monitor_integral), buffer(N) {}

    void eval (std::size_t iter, const Particle<T> &particle)
    {
        buffer.resize(particle.size());
        integral(iter, particle, buffer);
        index.push_back(iter);
        record.push_back(cblas_ddot(particle.size(),
                particle.get_weight_ptr(), 1, buffer, 1));
    }

    index_type get_index () const
    {
        return index;
    }

    record_type get_record () const
    {
        return record;
    }

    value_type get_value () const
    {
        return std::make_pair(index, record);
    }

    void clear ()
    {
        index.clear();
        record.clear();
    }

    private :

    integral_type integral;
    vDist::tool::Buffer<double> buffer;
    std::vector<std::size_t> index;
    std::vector<double> record;
}; // class Monitor

} // namespace vSMC

#endif // V_SMC_MONITOR_HPP
