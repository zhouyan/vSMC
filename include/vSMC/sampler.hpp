#ifndef V_SMC_SAMPLER_HPP
#define V_SMC_SAMPLER_HPP

#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <vSMC/particle.hpp>
#include <vSMC/history.hpp>
#include <vDist/rng/gsl.hpp>

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
    typedef std::size_t (*init_type) (Particle<T> &);
    /// The type of move callable objects
    typedef std::size_t (*move_type) (std::size_t, Particle<T> &);
    /// The type of monitor integration
    typedef void (*monitor_type) (std::size_t, const Particle<T> &, double *);
    /// The type of path sampling integration
    typedef double (*path_type) (std::size_t, const Particle<T> &, double *);

    Sampler (std::size_t N,
            init_type init, move_type move,
            typename Particle<T>::copy_type copy,
            HistoryMode hist_mode = HISTORY_RAM,
            ResampleScheme rs_scheme = RESIDUAL,
            double rs_threshold = 0.5,
            const int seed = V_DIST_SEED,
            const gsl_rng_type *brng = V_DIST_GSL_BRNG) :
        initialized(false), init_particle(init), move_particle(move),
        rng(seed, brng), scheme(rs_scheme), threshold(rs_threshold* N),
        particle(N, copy), iter_num(0), mode(hist_mode), history(hist_mode),
        integrate_tmp(N), path_integral(NULL), path_estimate(0) {}

    double ESS () const
    {
        return ess.back();
    }

    bool wasResampled () const
    {
        return resample.back();
    }

    std::size_t acceptCount () const
    {
        return accept.back();
    }

    const Particle<T> &getParticle () const
    {
        return particle;
    }

    void initialize ()
    {
        history.clear();
        ess.clear();
        resample.clear();
        accept.clear();

        for (std::map<std::string, std::vector<double> >::iterator
                imap = monitor_record.begin();
                imap != monitor_record.end(); ++imap)
            imap->second.clear();

        path_sample.clear();
        path_width.clear();

        iter_num = 0;
        accept.push_back(init_particle(particle));
        post_move();

        initialized = true;
    }

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

    void iterate (std::size_t n)
    {
        for (std::size_t i = 0; i != n; ++i)
            iterate();
    }

    double integrate (void (*integral) (
                std::size_t iter, const Particle<T> &, double *res, void *),
            void *param) const
    {
        std::size_t n = particle.size();
        integral(iter_num, particle, integrate_tmp, param);

        return cblas_ddot(n, particle.getWeightPtr(), 1, integrate_tmp, 1);
    }

    void addMonitor (const std::string &name, monitor_type integral)
    {
        monitor.insert(
                typename std::map<std::string, monitor_type>::value_type(
                    name, integral));
        monitor_index.insert(
                std::map<std::string, std::vector<std::size_t> >::value_type(
                    name, std::vector<size_t>()));
        monitor_record.insert(
                std::map<std::string, std::vector<double> >::value_type(
                    name, std::vector<double>()));
    }

    void eraseMonitor (const std::string &name)
    {
        monitor.erase(name);
        monitor_index.erase(name);
        monitor_record.erase(name);
    }

    void clearMonitor ()
    {
        monitor.clear();
        monitor_index.clear();
        monitor_record.clear();
    }

    std::vector<std::size_t> getMonitorIndex (const std::string &name) const
    {
        return monitor_index.find(name)->second;
    }

    std::vector<double> getMonitorRecord (const std::string &name) const
    {
        return monitor_record.find(name)->second;
    }

    void setPathIntegral (path_type integral)
    {
        path_integral = integral;
    }

    void clearPathIntegral ()
    {
        path_integral = NULL;
    }

    double getPathIntegral () const
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
    std::map<std::string, monitor_type> monitor;
    std::map<std::string, std::vector<std::size_t> > monitor_index;
    std::map<std::string, std::vector<double> > monitor_record;

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

        for (typename std::map<std::string, monitor_type>::iterator
                imap = monitor.begin(); imap != monitor.end(); ++imap) {
            monitor_index.find(imap->first)->second.push_back(iter_num);
            monitor_record.find(imap->first)->second.push_back(
                    eval_monitor(imap->second));
        }

        if (path_integral) {
            double width; 
            path_sample.push_back(eval_path(path_integral, width));
            path_width.push_back(width);
        }
    }

    double eval_monitor (monitor_type integral)
    {
        integral(iter_num, particle, integrate_tmp);
        return cblas_ddot(particle.size(),
                particle.getWeightPtr(), 1, integrate_tmp, 1);
    }

    double eval_path (path_type integral, double &width)
    {
        width = integral(iter_num, particle, integrate_tmp);
        return cblas_ddot(particle.size(),
                particle.getWeightPtr(), 1, integrate_tmp, 1);
    }
}; // class Sampler

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
