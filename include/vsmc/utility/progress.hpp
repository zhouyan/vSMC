//============================================================================
// include/vsmc/utility/progress.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_PROGRESS_HPP
#define VSMC_UTILITY_PROGRESS_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <iostream>
#include <string>

#if VSMC_HAS_CXX11LIB_CHRONO && VSMC_HAS_CXX11LIB_THREAD
#include <cmath>
#include <chrono>
#include <thread>
#endif

namespace vsmc {

#if VSMC_HAS_CXX11LIB_CHRONO && VSMC_HAS_CXX11LIB_THREAD
namespace internal {

struct ProgressThisThread
{
    static void sleep (double s)
    {
        double ms = std::fmax(1.0, std::floor(s * 1000));
        std::this_thread::sleep_for(std::chrono::milliseconds(
                    static_cast<std::chrono::milliseconds::rep>(ms)));
    }
}; // class ProgressThisThread

} // namespace vsmc::internal
#endif

/// \brief Display a progress bar while algorithm proceed
/// \ingroup Progress
///
/// \tparam ThreadType Any type that (partially) follows C++11 `std::thread`
/// interface. In particular, the following calls shal be supported,
/// ~~~{.cpp}
/// void (task *) (void *); // A function pointer
/// void *context;          // A void pointer
/// ThreadType *thread_ptr_ = new ThreadType(task, context);
/// thread_ptr_->joinable();
/// thread_ptr_->join();
/// delete thread_ptr_;
/// ~~~
/// \tparam ThisThread This shall be a class that provides a `sleep` static
/// member function that allows the calling thread to sleep for a specified
/// time in seconds. A simple implementation using C++11 `sleep_for` is as the
/// following,
/// ~~~{.cpp}
/// struct ThisThread
/// {
///     static void sleep (double s)
///     {
///         // Make the interval acurate to milliseconds
///         double ms = std::max(1.0, std::floor(s * 1000));
///         std::this_thread::sleep_for(std::chrono::milliseconds(
///                     static_cast<std::chrono::milliseconds::rep>(ms)));
///     }
/// };
/// ~~~
/// An implementation using Boost is almost the same except for the namespace
/// changing from `std` to `boost`.
#if VSMC_HAS_CXX11LIB_CHRONO && VSMC_HAS_CXX11LIB_THREAD
template <typename ThreadType = std::thread,
         typename ThisThread = internal::ProgressThisThread>
#else
template <typename ThreadType, typename ThisThread>
#endif
class Progress
{
    public :

    typedef ThreadType thread_type;

    /// \brief Construct a Progress with an output stream
    Progress (std::ostream &os = std::cout) :
        thread_ptr_(VSMC_NULLPTR), interval_(0), iter_(0), total_(0),
        length_(0), show_iter_(false), print_first_(true), in_progress_(false),
        num_equal_(0), percent_(0), seconds_(0), last_iter_(0),
        cstr_bar_(), cstr_percent_(), cstr_time_(), cstr_iter_(), os_(os) {}

    /// \brief Start to print the progress
    ///
    /// \param total Total amount of work represented by an integer, for
    /// example file size or SMC algorithm total number of iterations
    /// \param msg A (short) discreptive message
    /// \param length The length of the progress bar between brackets. If it is
    /// zero, then no bar is displayed at all
    /// \param show_iter Shall the iteration count be displayed.
    /// \param interval The sleep interval in seconds
    void start (std::size_t total, const std::string &msg = std::string(),
            std::size_t length = 0, bool show_iter = false,
            double interval = 0.1)
    {
        total_ = total;
        msg_ = msg;
        length_ = length;
        show_iter_ = show_iter;
        interval_ = interval;

        iter_ = 0;
        print_first_ = true;
        in_progress_ = true;

        if (length_ == 0) {
            cstr_bar_[0] = ' ';
            cstr_bar_[1] = '\0';
        }

        watch_.reset();
        watch_.start();
        fork();
    }

    /// \brief Stop to print the progress
    ///
    /// \param finished If true, then it is assumed that all work has been
    /// finished, and at the end the progress will be shown as `100%` and
    /// `total/total`, where total is the first parameter of `start`.
    /// Otherwise, whatever progress has been made will be shown.
    void stop (bool finished = false)
    {
        in_progress_ = false;
        join();
        if (finished && iter_ < total_)
            iter_ = total_;
        print_stop_(static_cast<void *>(this));
        watch_.stop();
    }

    /// \brief Increment the iteration count
    void increment (std::size_t step = 1) {iter_ += step;}

    private :

    StopWatch watch_;
    thread_type *thread_ptr_;

    double interval_;
    std::size_t iter_;
    std::size_t total_;
    std::size_t length_;
    bool show_iter_;
    bool print_first_;
    bool in_progress_;

    std::size_t num_equal_;
    std::size_t percent_;
    std::size_t seconds_;
    std::size_t last_iter_;

    std::string msg_;
    char cstr_bar_[128];
    char cstr_percent_[32];
    char cstr_time_[32];
    char cstr_iter_[64];

    std::ostream &os_;

    void fork ()
    {
        join();
        thread_ptr_ = new thread_type(print_start_, static_cast<void *>(this));
    }

    void join ()
    {
        if (thread_ptr_ != VSMC_NULLPTR) {
            if (thread_ptr_->joinable())
                thread_ptr_->join();
            delete thread_ptr_;
            thread_ptr_ = VSMC_NULLPTR;
        }
    }

    static void print_start_ (void *context)
    {
        Progress *ptr = static_cast<Progress *>(context);
        while (ptr->in_progress_) {
            print_progress(context);
            ptr->os_ << '\r' << std::flush;
            ThisThread::sleep(ptr->interval_);
        }
    }

    static void print_stop_ (void *context)
    {
        Progress *ptr = static_cast<Progress *>(context);
        print_progress(context);
        ptr->os_ << '\n' << std::flush;
    }

    static void print_progress (void *context)
    {
        Progress *ptr = static_cast<Progress *>(context);

        ptr->watch_.stop();
        ptr->watch_.start();
        const std::size_t seconds =
            static_cast<std::size_t>(ptr->watch_.seconds());

        const std::size_t display_iter = ptr->iter_ <= ptr->total_ ?
            ptr->iter_ : ptr->total_;
        std::size_t num_equal = (ptr->total_ | ptr->length_) == 0 ?
            ptr->length_ : static_cast<std::size_t>(
                    static_cast<double>(ptr->length_) *
                    static_cast<double>(display_iter) /
                    static_cast<double>(ptr->total_));
        num_equal = num_equal <= ptr->length_ ? num_equal : ptr->length_;
        std::size_t percent = ptr->total_ == 0 ? 100 :
            static_cast<std::size_t>(100.0 *
                    static_cast<double>(display_iter) /
                    static_cast<double>(ptr->total_));
        percent = percent <= 100 ? percent : 100;

        if (ptr->print_first_) {
            ptr->print_first_ = false;
            ptr->num_equal_ = num_equal + 1;
            ptr->percent_ = percent + 1;
            ptr->seconds_ = seconds + 1;
            ptr->last_iter_ = ptr->iter_ + 1;
        }

        if (ptr->length_ != 0 && ptr->num_equal_ != num_equal) {
            ptr->num_equal_ = num_equal;
            std::size_t num_space = ptr->length_ - num_equal;
            std::size_t num_dash = 0;
            if (num_space > 0) {
                num_dash = 1;
                --num_space;
            }

            char *cstr = ptr->cstr_bar_;
            std::size_t offset = 0;
            cstr[offset++] = ' ';
            cstr[offset++] = '[';
            for (std::size_t i = 0; i != num_equal; ++i)
                cstr[offset++] = '=';
            for (std::size_t i = 0; i != num_dash; ++i)
                cstr[offset++] = '-';
            for (std::size_t i = 0; i != num_space; ++i)
                cstr[offset++] = ' ';
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (ptr->percent_ != percent) {
            ptr->percent_ = percent;
            const std::size_t num_space = 3 - uint_digit(percent);

            char *cstr = ptr->cstr_percent_;
            std::size_t offset = 0;
            cstr[offset++] = '[';
            for (std::size_t i = 0; i != num_space; ++i)
                cstr[offset++] = ' ';
            uint_to_char(percent, cstr, offset);
            cstr[offset++] = '%';
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (ptr->seconds_ != seconds) {
            ptr->seconds_ = seconds;
            const std::size_t display_second = seconds % 60;
            const std::size_t display_minute = (seconds / 60) % 60;
            const std::size_t display_hour   = seconds / 3600;

            char *cstr = ptr->cstr_time_;
            std::size_t offset = 0;
            cstr[offset++] = '[';
            if (display_hour > 0) {
                uint_to_char(display_hour, cstr, offset);
                cstr[offset++] = ':';
            }
            cstr[offset++] = '0' + static_cast<char>(display_minute / 10);
            cstr[offset++] = '0' + static_cast<char>(display_minute % 10);
            cstr[offset++] = ':';
            cstr[offset++] = '0' + static_cast<char>(display_second / 10);
            cstr[offset++] = '0' + static_cast<char>(display_second % 10);
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (ptr->show_iter_ && ptr->last_iter_ != ptr->iter_) {
            ptr->last_iter_ = ptr->iter_;
            const std::size_t dtotal = uint_digit(ptr->total_);
            const std::size_t diter = uint_digit(ptr->iter_);
            const std::size_t num_space = dtotal > diter ? dtotal - diter : 0;

            char *cstr = ptr->cstr_iter_;
            std::size_t offset = 0;
            cstr[offset++] = '[';
            for (std::size_t i = 0; i < num_space; ++i)
                cstr[offset++] = ' ';
            uint_to_char(ptr->iter_, cstr, offset);
            cstr[offset++] = '/';
            uint_to_char(ptr->total_, cstr, offset);
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (ptr->length_ != 0) ptr->os_ << ptr->cstr_bar_;
        ptr->os_ << ptr->cstr_percent_;
        ptr->os_ << ptr->cstr_time_;
        if (ptr->show_iter_) ptr->os_ << ptr->cstr_iter_;
        if (ptr->msg_.size() != 0) ptr->os_ << '[' << ptr->msg_ << ']';
    }

    template <typename UIntType>
    static void uint_to_char (UIntType num, char *cstr, std::size_t &offset)
    {
        if (num == 0) {
            cstr[offset++] = '0';
            return;
        }

        char utmp[32];
        std::size_t unum = 0;
        while (num) {
            utmp[unum++] = '0' + static_cast<char>(num % 10);
            num /= 10;
        }
        for (std::size_t i = unum; i != 0; --i)
            cstr[offset++] = utmp[i - 1];
    }

    template <typename UIntType>
    static std::size_t uint_digit (UIntType num)
    {
        if (num == 0)
            return 1;

        std::size_t digit = 0;
        while (num != 0) {
            ++digit;
            num /= 10;
        }

        return digit;
    }
}; // class Progress

} // namespace vsmc

#endif // VSMC_UTILITY_PROGRESS_HPP
