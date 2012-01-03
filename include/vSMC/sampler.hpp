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
        scheme(resample_scheme), threshold(resample_threshold),
        iter_num(0), particle(N, copy), resample(false), ess(0), accept(0),
        mode(history_mode), history(history_mode),
        integrate_tmp(N)
    {
        rng = gsl_rng_alloc(gsl_rng_mt19937);
        gsl_rng_set(rng, 100);
    }

    ~Sampler ()
    {
        gsl_rng_free(rng);
    }

    void Initialize ()
    {
        while (history.size())
            history.pop_back();

        iter_num = 0;
        accept = init_particle(particle);
        post_move();
    }

    void Iterate ()
    {
        ++iter_num;
        accept = move_particle(iter_num, particle);
        post_move();
    }

    void Iterate (std::size_t n)
    {
        for (std::size_t i = 0; i != n; ++i)
            Iterate();
    }

    double Integrate (void (*intl) (const Particle<T> &, double *res))
    {
        double sw, sum = 0;
        std::size_t n = particle.size();
        intl(particle, integrate_tmp);
        vdMul(n, particle.Weight(sw),
                integrate_tmp, integrate_tmp);
        for (std::size_t i = 0; i != n; ++i)
            sum += integrate_tmp[i];

        return sum / sw;
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
    HistoryMode mode;
    History<T> history;
    gsl_rng *rng;

    typedef vDist::internal::Buffer<double> dBuffer;

    dBuffer integrate_tmp;

    void post_move ()
    {
        ess = particle.ESS();
        if (ess < threshold) {
            particle.Resample(scheme, rng);
            ess = 1;
        }
        if (mode != HISTORY_NONE)
            history.push_back(HistoryElement<T>(
                        particle, resample, ess, accept));
    }
};

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
