#ifndef V_SMC_HISTORY_HPP
#define V_SMC_HISTORY_HPP

#include <stdexcept>
#include <vector>
#include <cstddef>
#include <vSMC/particle.hpp>

namespace vSMC {

enum HistoryMode {HISTORY_NONE, HISTORY_RAM, HISTORY_FILE};

template <class T>
class HistoryElement
{
    public :

    explicit HistoryElement (const Particle<T> &particle,
            bool was_resample, double ess, std::size_t accept_count) :
        e_particle(particle), e_resample(was_resample),
        e_ess(ess), e_accept(accept_count) {}

    const Particle<T> &particle () const
    {
        return e_particle;
    }

    double ESS () const
    {
        return e_ess;
    }

    bool WasResample () const
    {
        return e_resample;
    }

    std::size_t AcceptCount () const
    {
        return e_accept;
    }

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

    explicit History (HistoryMode history_mode) : mode(history_mode) {};

    void push_back (const HistoryElement<T> &element)
    {
        if (mode == HISTORY_RAM)
            history.push_back(element);
    }

    void pop_back ()
    {
        if (mode == HISTORY_RAM)
            history.pop_back();
    }

    void pop_back (HistoryElement<T> &element)
    {
        if (mode == HISTORY_RAM) {
            element = history.back();
            history.pop_back();
        }
    }

    std::size_t size () const
    {
        switch (mode) {
            case HISTORY_RAM :
                return history.size();
            case HISTORY_FILE :
                return 0;
            case HISTORY_NONE :
                return 0;
            default :
                throw std::runtime_error("Unknown HistoryMode");
        }
    }

    private :

    HistoryMode mode;
    std::vector<HistoryElement<T> > history;
}; // class History

} // namespace vSMC

#endif // V_SMC_HISTORY_HPP
