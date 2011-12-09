#ifndef V_SMC_HISTORY_HPP
#define V_SMC_HISTORY_HPP

#include <vector>
#include <cstddef>
#include <vSMC/particle.hpp>

namespace vSMC {

enum HistoryMode {HISTORY_RAM, HISTORY_FILE, HISTORY_NONE};

template <class PartType>
class HistoryElement
{
    public :

    HistoryElement (Particle<PartType> &particle,
            bool resample, double ess, std::size_t accept) :
        e_particle(particle), e_resample(resample),
        e_ess(ess), e_accept(accept) {}

    HistoryElement (const HistoryElement<PartType> &element) :
        e_particle(element.particle), e_resample(element.resample),
        e_ess(element.ess), e_accept(element.accept) {}

    HistoryElement<PartType> & operator= (
            const HistoryElement<PartType> &element)
    {
        if (&element != this) {
            e_particle = element.particle;
            e_resample = element.resample;
            e_ess = element.ess;
            e_accept = element.accept;
        }

        return *this;
    }

    private :

    Particle<PartType> e_particle;
    bool e_resample;
    double e_ess;
    std::size_t e_accept;
}; // class HistoryElement

template <class PartType>
class History
{
    public :

    inline void push_back (const HistoryElement<PartType> &element);

    inline void pop_back () const;

    inline void pop_back (HistoryElement<PartType> &element) const;

    private :

    std::vector<HistoryElement<PartType> > history;
}; // class History

template <class PartType>
void History<PartType>::push_back (const HistoryElement<PartType> &element)
{
    history.push_back(element);
}

template <class PartType>
void History<PartType>::pop_back ()
{
    history.pop_back();
}

template <class PartType>
void History<PartType>::pop_back (HistoryElement<PartType> &element)
{
    element = history.back();
    pop_back();
}

} // namespace vSMC

#endif // V_SMC_HISTORY_HPP
