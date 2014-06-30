//============================================================================
// vsmc/utility/backup.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_BACKUP_HPP
#define VSMC_UTILITY_BACKUP_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Backup and restore objects manually
/// \ingroup Backup
template <typename T>
class Backup
{
    public :

    Backup () : backup_(VSMC_NULLPTR), is_saved_(false) {}

    Backup (const T *src) : backup_(new T(*src)), is_saved_(true) {}

    Backup (const T &src) : backup_(new T(src)), is_saved_(true) {}

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
        if (!backup_)
            backup_ = new T(*src);
        else
            *backup_ = *src;

        is_saved_ = true;
    }

    void save (const T &src) {save(&src);}

    bool restore (T *dst) const
    {
        if (!is_saved_)
            return false;

        *dst = *backup_;

        return true;
    }

    bool restore (T &dst) const {return restore(&dst);}

    private :

    T *backup_;
    bool is_saved_;
}; // class Backup

/// \brief Backup and restore objects in scope
/// \ingroup Backup
template <typename T>
class ScopedBackup
{
    public :

    ScopedBackup (T &src) : src_(src), backup_(src) {}
    ScopedBackup (T *src) : src_(*src), backup_(src) {}
    ~ScopedBackup () {src_ = backup_;}

    private :

    T &src_;
    T backup_;
}; // class BackupStatic

} // namespace vsmc

#endif // VSMC_UTILITY_BACKUP_HPP
