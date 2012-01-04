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
            HistoryMode history_mode = HISTORY_RAM,
            ResampleScheme resample_scheme = RESIDUAL,
            double resample_threshold = 0.5) :
        init_particle(init), move_particle(move),
        scheme(resample_scheme), threshold(resample_threshold * N),
        iter_num(0), particle(N, copy),
        ess(0), resample(false), accept(0),
        mode(history_mode), history(history_mode),
        rng(gsl_rng_alloc(gsl_rng_mt19937)), integrate_tmp(N)
    {
        gsl_rng_set(rng, 100);
        initialize();
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

        iter_num = 0;
        accept = init_particle(particle);
        post_move();
    }

    void iterate ()
    {
        ++iter_num;
        accept = move_particle(iter_num, particle);
        post_move();
    }

    void iterate (std::size_t n)
    {
        for (std::size_t i = 0; i != n; ++i)
            iterate();
    }

    double integrate (
            void (*integral) (const Particle<T> &, double *res, void *),
            void *param) const
    {
        std::size_t n = particle.size();
        integral(particle, integrate_tmp, param);

        return cblas_ddot(n, particle.getWeightPtr(), 1, integrate_tmp, 1);
    }

    private :

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
    }
};

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
