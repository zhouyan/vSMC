#ifndef V_SMC_SAMPLER_HPP
#define V_SMC_SAMPLER_HPP

#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <vSMC/particle.hpp>
#include <vSMC/history.hpp>

namespace vSMC {

template <class T>
class Sampler
{
    public :

    /// The type of particle values
    typedef T value_type;
    /// The type of partiles
    typedef Particle<T> particle_type;
    /// The type of monitors
    typedef void (*monitor_type)
        (std::size_t iter, const Particle<T> &, double *res);

    Sampler (std::size_t N,
            std::size_t (*init) (Particle<T> &),
            std::size_t (*move) (std::size_t iter, Particle<T> &),
            void (*copy) (std::size_t, std::size_t, T &),
            HistoryMode history_mode = HISTORY_RAM,
            ResampleScheme resample_scheme = RESIDUAL,
            double resample_threshold = 0.5) :
        initialized(false), init_particle(init), move_particle(move),
        scheme(resample_scheme), threshold(resample_threshold * N),
        iter_num(0), particle(N, copy),
        ess(0), resample(false), accept(0),
        mode(history_mode), history(history_mode),
        rng(gsl_rng_alloc(gsl_rng_mt19937)), integrate_tmp(N)
    {
        gsl_rng_set(rng, 100);
    }

    ~Sampler ()
    {
        gsl_rng_free(rng);
    }

    double ESS () const
    {
        return ess;
    }

    const Particle<T> &getParticle () const
    {
        return particle;
    }

    void initialize ()
    {
        while (history.size())
            history.pop_back();
        for (std::map<std::string, std::vector<double> >::iterator
                imap = monitor_record.begin();
                imap != monitor_record.end(); ++imap)
        {
            imap->second.clear();
        }

        iter_num = 0;
        accept = init_particle(particle);
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
        accept = move_particle(iter_num, particle);
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
        monitor_record.insert(
                std::map<std::string, std::vector<double> >::value_type(
                    name, std::vector<double>()));
    }

    void removeMonitor (const std::string &name)
    {
        monitor.erase(name);
        monitor_record.erase(name);
    }

    std::vector<double> getMonitor (const std::string &name)
    {
        return monitor_record[name];
    }

    private :

    bool initialized;

    std::size_t (*init_particle) (Particle<T> &);
    std::size_t (*move_particle) (std::size_t, Particle<T> &);

    ResampleScheme scheme;
    double threshold;

    std::size_t iter_num;
    Particle<T> particle;

    double ess;
    bool resample;
    std::size_t accept;

    HistoryMode mode;
    History<T> history;

    gsl_rng *rng;

    mutable vDist::internal::Buffer<double> integrate_tmp;
    std::map<std::string, monitor_type> monitor;
    std::map<std::string, std::vector<double> > monitor_record;

    void post_move ()
    {
        ess = particle.ESS();

        if (ess < threshold) {
            particle.resample(scheme, rng);
            ess = particle.ESS();
        }

        if (mode != HISTORY_NONE) {
            history.push_back(
                    HistoryElement<T>(particle, resample, ess, accept));
        }

        for (typename std::map<std::string, monitor_type>::iterator
                imap = monitor.begin(); imap != monitor.end(); ++imap)
        {
            monitor_record.find(imap->first)->second.push_back(
                    record(imap->second));
        }
    }

    double record (monitor_type integral)
    {
        std::size_t n = particle.size();
        integral(iter_num, particle, integrate_tmp);

        return cblas_ddot(n, particle.getWeightPtr(), 1, integrate_tmp, 1);
    }
};

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
