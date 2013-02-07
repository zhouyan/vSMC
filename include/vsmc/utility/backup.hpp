#ifndef VSMC_UTILITY_BACKUP_HPP
#define VSMC_UTILITY_BACKUP_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>

/// \brief Backup and restore objects
/// \ingroup Uitlity
namespace vsmc {

template <typename T>
class Backup
{
    public :

    Backup () : backup_(VSMC_NULLPTR), is_saved_(false) {}

    Backup (const T *src) : backup_(new T(*src)), is_saved_(true) {}

    Backup (const Backup<T> &other) :
        backup_(VSMC_NULLPTR), is_saved_(other.is_saved_)
    {
        if (is_saved_)
            backup_ = new T(*(other.backup_));
    }

    Backup<T> &operator= (const Backup<T> &other)
    {
        if (&other != this) {
            delete backup_;
            backup_ = VSMC_NULLPTR;
            is_saved_ = other.is_saved_;
            if (is_saved_)
                backup_ = new T(*(other.backup_));
        }

        return *this;
    }

    ~Backup () {delete backup_;}

    bool is_saved () const {return is_saved_;}

    void save (const T *src)
    {
        VSMC_RUNTIME_ASSERT_DERIVED_BASE(Backup);

        if (!backup_)
            backup_ = new T(*src);
        else
            *backup_ = *src;

        is_saved_ = true;
    }

    bool restore (T *dst) const
    {
        if (!is_saved_)
            return false;

        *dst = *backup_;

        return true;
    }

    private :

    T *backup_;
    bool is_saved_;
};

}; // 

#endif // VSMC_UTILITY_BACKUP_HPP
