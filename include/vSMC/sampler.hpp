#ifndef V_SMC_SAMPLER_HPP
#define V_SMC_SAMPLER_HPP

#include <vSMC/particle.hpp>
#include <vSMC/history.hpp>

namespace vSMC {

template <class T>
class Sampler
{
    public :

    Sampler (std::size_t N,
            std::size_t (*init) (Particle<T> &, void *),
            std::size_t (*move) (Particle<T> &, void *),
            void (*copy) (std::size_t, std::size_t, T &),
            ResampleScheme resample_scheme = RESIDUAL,
            double resample_threshold = 0.5,
            HistoryMode history_mode = HISTORY_RAM) :
        init_particle(init), move_particle(move),
        scheme(resample_scheme), threshold(resample_threshold),
        history(history_mode),
        particle(N, copy), resample(false), ess(0), accept(0) {}

    inline void Initialize ()
    {
        accept = init_particle(particle);
        ess = particle.ESS();
        if (ess < threshold) {
            particle.resample(scheme);
            ess = 1;
        }
        history.push_back(HistoryElement<T>(particle, resample, ess, accept));
    }

    inline void Iterate ()
    {
        accept = move_particle(particle);
        ess = particle.ESS();
        if (ess < threshold) {
            particle.resample(scheme);
            ess = 1;
        }
        history.push_back(HistoryElement<T>(particle, resample, ess, accept));
    }

    inline void Iterate (std::size_t n)
    {
        for (std::size_t i = 0; i != n; ++i)
            Iterate();
    }

    private :

    std::size_t (*init_particle) (Particle<T> &, void *);
    std::size_t (*move_particle) (Particle<T> &, void *);
    const ResampleScheme scheme;
    const double threshold;

    Particle<T> particle;
    bool resample;
    double ess;
    std::size_t accept;
    History<T> history;
};

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
