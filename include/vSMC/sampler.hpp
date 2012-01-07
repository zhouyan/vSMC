#ifndef V_SMC_SAMPLER_HPP
#define V_SMC_SAMPLER_HPP

#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <gsl/gsl_cblas.h>
#include <boost/function.hpp>
#include <vDist/rng/gsl.hpp>
#include <vDist/tool/buffer.hpp>
#include <vSMC/history.hpp>
#include <vSMC/monitor.hpp>
#include <vSMC/particle.hpp>

namespace vSMC {

template <class T>
class Sampler
{
    public :

    /// The type of particle values
    typedef T value_type;
    /// The type of partiles
    typedef Particle<T> particle_type;
    /// The type of initialize callable objects
    typedef boost::function<std::size_t
        (Particle<T> &)> init_type;
    /// The type of move callable objects
    typedef boost::function<std::size_t
        (std::size_t, Particle<T> &)> move_type;
    /// The type of importance sampling integral
    typedef boost::function<void
        (std::size_t, const Particle<T> &, double *, void *)> integral_type;
    /// The type of path sampling integration
    typedef boost::function<double
        (std::size_t, const Particle<T> &, double *)> path_type;

    /// \brief Sampler does not have a default constructor
    ///
    /// \param N The number of particles
    /// \param init The functor used to initialize the particles
    /// \param move The functor used to move the particles and weights
    /// \param hist_mode The history storage mode. See HistoryMode
    /// \param rs_scheme The resampling scheme. See ResampleScheme
    /// \param seed The seed for the reampling RNG. See documentation of vDist
    /// \param brng The basic RNG for resampling RNG. See documentation of GSL
    Sampler (
            std::size_t N,
            const init_type &init, const move_type &move,
            const typename Particle<T>::copy_type &copy,
            HistoryMode hist_mode = HISTORY_RAM,
            ResampleScheme rs_scheme = RESIDUAL,
            double rs_threshold = 0.5,
            const int seed = V_DIST_SEED,
            const gsl_rng_type *brng = V_DIST_GSL_BRNG) :
        initialized(false), init_particle(init), move_particle(move),
        rng(seed, brng), scheme(rs_scheme), threshold(rs_threshold* N),
        particle(N, copy), iter_num(0), mode(hist_mode), history(hist_mode),
        integrate_tmp(N), path_integral(NULL), path_estimate(0) {}

    /// \brief Get ESS
    ///
    /// \return The ESS value of the latest iteration
    double ESS () const
    {
        return ess.back();
    }

    /// \brief Get indicator of resampling
    ///
    /// \return A bool value, \b true if the latest iteration was resampled
    bool wasResampled () const
    {
        return resample.back();
    }

    /// \brief Get accept count
    ///
    /// \return The accept count of the latest iteration
    std::size_t acceptCount () const
    {
        return accept.back();
    }

    /// \brief Read only access to the particle set
    ///
    /// \return A const reference to the latest particle set.
    /// \note Any operations that change the state of the sampler (e.g., an
    /// iteration) may invalidate the reference.
    const Particle<T> &getParticle () const
    {
        return particle;
    }

    /// \brief (Re)initialize the particle set
    void initialize ()
    {
        history.clear();
        ess.clear();
        resample.clear();
        accept.clear();

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor.begin(); imap != monitor.end(); ++imap)
            imap->second.clear();

        path_sample.clear();
        path_width.clear();

        iter_num = 0;
        accept.push_back(init_particle(particle));
        post_move();

        initialized = true;
    }

    /// \brief Perform iteration
    void iterate ()
    {
        if (!initialized)
            throw std::runtime_error(
                    "ERROR: vSMC::Sampler::iterate: "
                    "Sampler has not been initialized yet");

        ++iter_num;
        accept.push_back(move_particle(iter_num, particle));
        post_move();
    }

    /// \brief Perform iteration
    ///
    /// \param n The number of iterations to be performed
    void iterate (std::size_t n)
    {
        for (std::size_t i = 0; i != n; ++i)
            iterate();
    }

    /// \brief Perform importance sampling integration
    ///
    /// \param intgral The functor used to compute the integrands
    /// \param param Additional parameters to be passed to integral
    double integrate (integral_type integral, void *param) const
    {
        std::size_t n = particle.size();
        integral(iter_num, particle, integrate_tmp, param);

        return cblas_ddot(n, particle.get_weight_ptr(), 1, integrate_tmp, 1);
    }

    /// \brief Add a monitor, similar to \b monitor in \b BUGS
    ///
    /// \param name The name of the monitor
    /// \param integral The functor used to compute the integrands
    void add_monitor (const std::string &name,
            const typename Monitor<T>::integral_type &integral)
    {
        monitor.insert(
                typename std::map<std::string, Monitor<T> >::value_type(
                    name, Monitor<T>(integral, particle.size())));
    }

    /// \brief Get the iteration numbers of a monitor by name
    ///
    /// \param The name of the monitor
    /// \return A vector of the monitor index
    typename Monitor<T>::index_type get_monitor_index (
            const std::string &name) const
    {
        return monitor.find(name)->second.get_index();
    }

    /// \brief Get the record of a monitor by name
    ///
    /// \param name The name of the monitor
    /// \return A vector of the monitor record
    typename Monitor<T>::record_type get_monitor_record (
            const std::string &name) const
    {
        return monitor.find(name)->second.get_record();
    }

    /// \brief Get both the iteration numbers and record of a monitor by name
    typename Monitor<T>::value_type get_monitor_value (
            const std::string &name) const
    {
        return monitor.find(name)->second.get();
    }

    /// \brief Erase a monitor by name 
    ///
    /// \param name The name of the monitor
    void erase_monitor (const std::string &name)
    {
        monitor.erase(name);
    }

    /// \brief Clear all monitors
    void clear_monitor ()
    {
        monitor.clear();
    }

    /// \brief Set the path sampling integral
    ///
    /// \param integral The functor used to compute the integrands
    void set_path_sampling (path_type integral)
    {
        path_integral = integral;
    }

    /// \brief Stop path sampling
    void clear_path_sampling ()
    {
        path_integral = NULL;
    }

    /// \brief Get the results of path sampling
    double get_path_sampling () const
    {
    }

    private :

    /// Initialization indicator
    bool initialized;

    /// Initialization and movement
    init_type init_particle;
    move_type move_particle;

    /// Resampling
    vDist::RngGSL rng;
    ResampleScheme scheme;
    double threshold;

    /// Particle sets
    Particle<T> particle;
    std::size_t iter_num;
    std::vector<double> ess;
    std::vector<bool> resample;
    std::vector<std::size_t> accept;

    /// History
    HistoryMode mode;
    History<T> history;

    /// Monte Carlo estimation by integration
    mutable vDist::tool::Buffer<double> integrate_tmp;
    std::map<std::string, Monitor<T> > monitor;

    /// Path sampling
    path_type path_integral;
    std::vector<double> path_sample;
    std::vector<double> path_width;
    double path_estimate;

    void post_move ()
    {
        bool was_resample = false;
        if (particle.ESS() < threshold) {
            was_resample = true;
            particle.resample(scheme, rng.get_rng());
        }
        ess.push_back(particle.ESS());

        if (mode != HISTORY_NONE)
            history.push_back(particle);

        for (typename std::map<std::string, Monitor<T> >::iterator
                imap = monitor.begin(); imap != monitor.end(); ++imap) {
            imap->second.eval(iter_num, particle);
        }

        if (!path_integral.empty()) {
            double width; 
            path_sample.push_back(eval_path(width));
            path_width.push_back(width);
        }
    }

    double eval_path (double &width)
    {
        width = path_integral(iter_num, particle, integrate_tmp);
        return cblas_ddot(particle.size(),
                particle.get_weight_ptr(), 1, integrate_tmp, 1);
    }
}; // class Sampler

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
