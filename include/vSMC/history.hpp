#ifndef V_SMC_HISTORY_HPP
#define V_SMC_HISTORY_HPP

#include <stdexcept>
#include <vector>
#include <cstddef>
#include <vSMC/particle.hpp>

namespace vSMC {

enum HistoryMode {HISTORY_NONE, HISTORY_RAM, HISTORY_FILE};

template <class T>
class History
{
    public :

    explicit History (HistoryMode mode) : mode_(mode) {};

    HistoryMode mode () const
    {
        return mode_;
    }

    void push_back (const Particle<T> &particle)
    {
        if (mode_ == HISTORY_RAM)
            history.push_back(particle);
    }

    void pop_back ()
    {
        if (mode_ == HISTORY_RAM)
            history.pop_back();
    }

    void pop_back (Particle<T> &particle)
    {
        if (mode_ == HISTORY_RAM) {
            particle = history.back();
            history.pop_back();
        }
    }

    void clear ()
    {
        if (mode_ == HISTORY_RAM)
            history.clear();
    }

    std::size_t size () const
    {
        switch (mode_) {
            case HISTORY_RAM :
                return history.size();
            case HISTORY_FILE :
                return 0;
            case HISTORY_NONE :
                return 0;
            default :
                throw std::runtime_error(
                        "ERROR: vSMC::Particle::resample: "
                        "Unknown History Mode");
        }
    }

    private :

    HistoryMode mode_;
    std::vector<Particle<T> > history;
}; // class History

} // namespace vSMC

#endif // V_SMC_HISTORY_HPP
