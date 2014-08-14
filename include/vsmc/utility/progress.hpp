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

namespace vsmc {

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
template <typename ThreadType, typename ThisThread>
class Progress
{
    public :

    typedef ThreadType thread_type;

    /// \brief Construct a Progress with an output stream
    Progress (std::ostream &os = std::cout) :
        thread_ptr_(VSMC_NULLPTR), iter_(0), total_(0), interval_(0),
        print_first_(true), in_progress_(false),
        num_equal_(0), percent_(0), seconds_(0), last_iter_(0),
        display_progress_(), display_percent_(), display_time_(),
        display_iter_(), os_(os) {}

    /// \brief Start to print the progress
    ///
    /// \param total Total amount of work represented by an integer, for
    /// example file size or SMC algorithm total number of iterations
    /// \param interval The sleep interval in seconds
    void start (unsigned total, double interval = 0.1)
    {
        iter_ = 0;
        total_ = total;
        interval_ = interval;
        print_first_ = true;
        in_progress_ = true;

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
    void increment (unsigned step = 1) {iter_ += step;}

    private :

    StopWatch watch_;
    thread_type *thread_ptr_;

    unsigned iter_;
    unsigned total_;
    double interval_;
    bool print_first_;
    bool in_progress_;

    unsigned num_equal_;
    unsigned percent_;
    unsigned seconds_;
    unsigned last_iter_;

    char display_progress_[128];
    char display_percent_[32];
    char display_time_[32];
    char display_iter_[64];

    std::ostream &os_;

    static VSMC_CONSTEXPR const unsigned num_equal_max_ = 60;
    static VSMC_CONSTEXPR const unsigned num_dash_max_ = 1;
    static VSMC_CONSTEXPR const unsigned percent_max_ = 100;

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
        const unsigned seconds = static_cast<unsigned>(ptr->watch_.seconds());
        const unsigned iter = ptr->iter_;
        const unsigned total = ptr->total_;

        const unsigned display_iter = iter <= total ? iter : total;
        unsigned num_equal = total == 0 ? num_equal_max_ :
            static_cast<unsigned>(
                    static_cast<double>(num_equal_max_) *
                    static_cast<double>(display_iter) /
                    static_cast<double>(total));
        num_equal = num_equal <= num_equal_max_ ? num_equal : num_equal_max_;
        unsigned percent = total == 0 ? percent_max_ :
            static_cast<unsigned>(
                    static_cast<double>(percent_max_) *
                    static_cast<double>(display_iter) /
                    static_cast<double>(total));
        percent = percent <= percent_max_ ? percent : percent_max_;

        if (ptr->print_first_) {
            ptr->print_first_ = false;
            ptr->num_equal_ = num_equal + 1;
            ptr->percent_ = percent + 1;
            ptr->seconds_ = seconds + 1;
            ptr->last_iter_ = iter + 1;
        }

        if (ptr->num_equal_ != num_equal) {
            ptr->num_equal_ = num_equal;
            unsigned num_space = num_equal_max_ - num_equal;
            unsigned num_dash = 0;
            while (num_space > num_dash) {
                if (num_dash == num_dash_max_)
                    break;
                --num_space;
                ++num_dash;
            }

            char *cstr = ptr->display_progress_;
            std::size_t offset = 0;
            cstr[offset++] = ' ';
            cstr[offset++] = '[';
            for (unsigned i = 0; i != num_equal; ++i)
                cstr[offset++] = '=';
            for (unsigned i = 0; i != num_dash; ++i)
                cstr[offset++] = '-';
            for (unsigned i = 0; i != num_space; ++i)
                cstr[offset++] = ' ';
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (ptr->percent_ != percent) {
            ptr->percent_ = percent;
            const unsigned num_space = 3 - uint_digit(percent);

            char *cstr = ptr->display_percent_;
            std::size_t offset = 0;
            cstr[offset++] = '[';
            for (unsigned i = 0; i != num_space; ++i)
                cstr[offset++] = ' ';
            uint_to_char(percent, cstr, offset);
            cstr[offset++] = '%';
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (ptr->seconds_ != seconds) {
            ptr->seconds_ = seconds;
            const unsigned display_second = seconds % 60;
            const unsigned display_minute = (seconds / 60) % 60;
            const unsigned display_hour   = seconds / 3600;

            char *cstr = ptr->display_time_;
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

        if (ptr->last_iter_ != iter) {
            ptr->last_iter_ = iter;
            const unsigned dtotal = uint_digit(total);
            const unsigned diter = uint_digit(iter);
            const unsigned num_space = dtotal > diter ? dtotal - diter : 0;
            char *cstr = ptr->display_iter_;

            std::size_t offset = 0;
            cstr[offset++] = '[';
            for (unsigned i = 0; i < num_space; ++i)
                cstr[offset++] = ' ';
            uint_to_char(iter, cstr, offset);
            cstr[offset++] = '/';
            uint_to_char(total, cstr, offset);
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        ptr->os_ << ptr->display_progress_;
        ptr->os_ << ptr->display_percent_;
        ptr->os_ << ptr->display_time_;
        ptr->os_ << ptr->display_iter_;
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
    static unsigned uint_digit (UIntType num)
    {
        if (num == 0)
            return 1;

        unsigned digit = 0;
        while (num != 0) {
            ++digit;
            num /= 10;
        }

        return digit;
    }
}; // class Progress

} // namespace vsmc

#endif // VSMC_UTILITY_PROGRESS_HPP
