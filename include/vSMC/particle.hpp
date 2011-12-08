#ifndef V_SMC_PARTICLE_HPP
#define V_SMC_PARTICLE_HPP

#include <vector>
#include <cstddef>
#include <vDist/utilities/service.hpp>

namespace vSMC {

enum RESAMPLE_SCHEME {MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC};

template <class PartContainer>
class ParticleSet
{
    public :

    Particle (std::size_t N) :
        particle_num(N), particle_set(N), log_weight(N) {};

    inline void Resample ();
    inline double ESS ();

    inline void SetWeight    (std::size_t n, const double *weight);
    inline void MulWeight    (std::size_t n, const double *inc_weight);
    inline void SetLogWeight (std::size_t n, const double *log_weight);
    inline void AddLogWeight (std::size_t n, const double *inc_weight);

    private :

    std::size_t particle_num;
    PartContainer particle_set;
    vDist::internal::Buffer<double> log_weight;

    inline void resample_multinomial ();
    inline void resample_residual ();
    inline void resample_stratified ();
    inline void resample_systematic ();
    inline void resample_do (std::size_t n, std::size_t *rep);
};

} // namespace vSMC

#endif // V_SMC_PARTICLE_HPP
