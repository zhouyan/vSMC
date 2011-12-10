#ifndef V_SMC_HISTORY_HPP
#define V_SMC_HISTORY_HPP

#include <vector>
#include <cstddef>
#include <vSMC/particle.hpp>

namespace vSMC {

enum HistoryMode {HISTORY_RAM, HISTORY_FILE, HISTORY_NONE};

template <class T>
class HistoryElement
{
    public :

    HistoryElement (Particle<T> &particle,
            bool resample, double ess, std::size_t accept) :
        e_particle(particle), e_resample(resample),
        e_ess(ess), e_accept(accept) {}

    private :

    Particle<T> e_particle;
    bool e_resample;
    double e_ess;
    std::size_t e_accept;
}; // class HistoryElement

template <class T>
class History
{
    public :

    inline void push_back (const HistoryElement<T> &element)
    {
        history.push_back(element);
    }

    inline void pop_back ()
    {
        history.pop_back();
    }

    inline void pop_back (HistoryElement<T> &element)
    {
        element = history.back();
        history.pop_back();
    }

    private :

    std::vector<HistoryElement<T> > history;
}; // class History

} // namespace vSMC

#endif // V_SMC_HISTORY_HPP
