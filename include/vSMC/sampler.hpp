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
            std::size_t (*init) (Particle<T> &),
            std::size_t (*move) (std::size_t iter, Particle<T> &),
            void (*copy) (std::size_t, std::size_t, T &),
            ResampleScheme resample_scheme = RESIDUAL,
            double resample_threshold = 0.5,
            HistoryMode history_mode = HISTORY_RAM) :
        init_particle(init), move_particle(move),
        scheme(resample_scheme), threshold(resample_threshold),
        iter_num(0), particle(N, copy), resample(false), ess(0), accept(0),
        history(history_mode)
    {
        rng = gsl_rng_alloc(gsl_rng_mt19937);
        gsl_rng_set(rng, 100);
    }

    inline void Initialize ()
    {
        iter_num = 0;
        accept = init_particle(particle);
        ess = particle.ESS();
        if (ess < threshold) {
            particle.Resample(scheme, rng);
            ess = 1;
        }
        history.push_back(HistoryElement<T>(particle, resample, ess, accept));
    }

    inline void Iterate ()
    {
        ++iter_num;
        accept = move_particle(iter_num, particle);
        ess = particle.ESS();
        if (ess < threshold) {
            particle.Resample(scheme, rng);
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

    std::size_t (*init_particle) (Particle<T> &);
    std::size_t (*move_particle) (std::size_t, Particle<T> &);
    ResampleScheme scheme;
    double threshold;

    std::size_t iter_num;
    Particle<T> particle;
    bool resample;
    double ess;
    std::size_t accept;
    History<T> history;

    gsl_rng *rng;
};

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
