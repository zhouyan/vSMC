#ifndef V_SMC_HISTORY_HPP
#define V_SMC_HISTORY_HPP

#include <vector>
#include <cstddef>
#include <vSMC/particle.hpp>

namespace vSMC {

enum HistoryMode {HISTORY_RAM, HISTORY_FILE, HISTORY_NONE};

template <class PartContainer>
class HistoryElement
{
    public :

    private :

    Particle<PartContainer> particle;
    bool resample;
    double ESS;
    std::size_t accept;
}; // class HistoryElement

template <class PartContainer>
class History
{
    public :

    inline void push_back (const HistoryElement<PartContainer> &element);

    inline void pop_back () const;

    inline void pop_back (HistoryElement<PartContainer> &element) const;

    private :

    std::vector<HistoryElement<PartContainer> > history;
}; // class History

template <class PartContainer>
void History<PartContainer>::push_back (
        const HistoryElement<PartContainer> &element)
{
    history.push_back(element);
}

template <class PartContainer>
void History<PartContainer>::pop_back ()
{
    history.pop_back();
}

template <class PartContainer>
void History<PartContainer>::pop_back (
        HistoryElement<PartContainer> &element)
{
    element = history.back();
    pop_back();
}

} // namespace vSMC

#endif // V_SMC_HISTORY_HPP
