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

    Monitor () : buffer(1) {}

    Monitor (const integral_type &monitor_integral) :
        integral(monitor_integral), buffer(1) {}

    void eval (std::size_t iter, const Particle<T> &particle)
    {
        buffer.resize(particle.size());
        integral(iter, particle, buffer);
        index.push_back(iter);
        record.push_back(cblas_ddot(particle.size(),
                particle.get_weight_ptr(), 1, buffer, 1));
    }

    std::vector<std::size_t> get_index () const
    {
        return index;
    }

    std::vector<double> get_record () const
    {
        return record;
    }

    std::pair<std::vector<std::size_t>, std::vector<double> > get () const
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
