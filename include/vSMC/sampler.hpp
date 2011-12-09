#ifndef V_SMC_SAMPLER_HPP
#define V_SMC_SAMPLER_HPP

#include <vSMC/particle.hpp>
#include <vSMC/history.hpp>

namespace vSMC {

template <class PartContainer>
class Sampler
{
    public :

    inline void Initialize ();
    inline void Iterate ();
    inline void Iterate (std::size_t n);

    private :

    Particle particle;
    History history;
};

} // namespace vSMC

#endif // V_SMC_SAMPLER_HPP
