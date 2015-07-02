//============================================================================
// vSMC/include/vsmc/utility/progress.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_UTILITY_PROGRESS_HPP
#define VSMC_UTILITY_PROGRESS_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/stop_watch.hpp>

namespace vsmc
{

/// \brief Display a progress bar while algorithm proceed
/// \ingroup Progress
class Progress
{
    public:
    /// \brief Construct a Progress with an output stream
    Progress(std::ostream &os = std::cout)
        : thread_ptr_(nullptr)
        , interval_ms_(0)
        , iter_(0)
        , total_(0)
        , length_(0)
        , show_iter_(true)
        , print_first_(true)
        , in_progress_(false)
        , num_equal_(0)
        , percent_(0)
        , seconds_(0)
        , last_iter_(0)
        , cstr_bar_()
        , cstr_percent_()
        , cstr_time_()
        , cstr_iter_()
        , os_(os)
    {
    }

    ~Progress() { join(); }

    /// \brief Start to print the progress
    ///
    /// \param total Total amount of work represented by an integer, for
    /// example file size or SMC algorithm total number of iterations
    /// \param msg A (short) discreptive message
    /// \param length The length of the progress bar between brackets. If it
    /// is zero, then no bar is displayed at all
    /// \param show_iter Shall the iteration count be displayed.
    /// \param interval_s The sleep interval in seconds
    void start(std::size_t total, const std::string &msg = std::string(),
        std::size_t length = 0, bool show_iter = true, double interval_s = 0.1)
    {
        total_ = total;
        msg_ = msg;
        length_ = length;
        show_iter_ = show_iter;
        interval_ms_ = std::max(1.0, interval_s * 1000);

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
    void stop(bool finished = true)
    {
        join();
        if (finished && iter_ < total_)
            iter_ = total_;
        print_stop_(this);
        watch_.stop();
    }

    /// \brief Increment the iteration count
    ///
    /// \details
    /// This member function is thread-safe, and can be called from multiple
    /// threads.
    void increment(std::size_t step = 1) { iter_ += step; }

    /// \brief Set a new message for display
    void message(const std::string &msg) { msg_ = msg; }

    private:
    static constexpr std::size_t max_val_ =
        std::numeric_limits<std::size_t>::max();

    StopWatch watch_;
    std::thread *thread_ptr_;

    double interval_ms_;
    std::atomic<std::size_t> iter_;
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

    void fork()
    {
        join();
        iter_ = 0;
        print_first_ = true;
        in_progress_ = true;
        thread_ptr_ = new std::thread(print_start_, this);
    }

    void join()
    {
        in_progress_ = false;
        if (thread_ptr_ != nullptr) {
            if (thread_ptr_->joinable())
                thread_ptr_->join();
            delete thread_ptr_;
            thread_ptr_ = nullptr;
        }
    }

    static void print_start_(void *context)
    {
        Progress *ptr = static_cast<Progress *>(context);
        while (ptr->in_progress_) {
            print_progress(context);
            ptr->os_ << '\r' << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<std::chrono::milliseconds::rep>(
                    ptr->interval_ms_)));
        }
    }

    static void print_stop_(void *context)
    {
        Progress *ptr = static_cast<Progress *>(context);
        print_progress(context);
        ptr->os_ << '\n' << std::flush;
    }

    static void print_progress(void *context)
    {
        Progress *ptr = static_cast<Progress *>(context);

        ptr->watch_.stop();
        ptr->watch_.start();
        const std::size_t seconds =
            static_cast<std::size_t>(ptr->watch_.seconds());

        std::size_t iter = ptr->iter_;
        std::size_t display_iter = std::min(iter, ptr->total_);
        std::size_t num_equal = (ptr->length_ == 0 || ptr->total_ == 0) ?
            0 :
            ptr->length_ * display_iter / ptr->total_;
        std::size_t percent =
            ptr->total_ == 0 ? 100 : 100 * display_iter / ptr->total_;

        if (ptr->print_first_) {
            ptr->print_first_ = false;
            ptr->num_equal_ = max_val_;
            ptr->percent_ = max_val_;
            ptr->seconds_ = max_val_;
            ptr->last_iter_ = max_val_;
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
            const std::size_t display_hour = seconds / 3600;

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

        if (ptr->show_iter_ && ptr->last_iter_ != iter) {
            ptr->last_iter_ = iter;
            const std::size_t dtotal = uint_digit(ptr->total_);
            const std::size_t diter = uint_digit(iter);
            const std::size_t num_space = dtotal > diter ? dtotal - diter : 0;

            char *cstr = ptr->cstr_iter_;
            std::size_t offset = 0;
            cstr[offset++] = '[';
            for (std::size_t i = 0; i < num_space; ++i)
                cstr[offset++] = ' ';
            uint_to_char(iter, cstr, offset);
            cstr[offset++] = '/';
            uint_to_char(ptr->total_, cstr, offset);
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        ptr->os_ << ' ';
        if (ptr->length_ != 0)
            ptr->os_ << ptr->cstr_bar_;
        ptr->os_ << ptr->cstr_percent_;
        ptr->os_ << ptr->cstr_time_;
        if (ptr->show_iter_)
            ptr->os_ << ptr->cstr_iter_;
        if (ptr->msg_.size() != 0)
            ptr->os_ << '[' << ptr->msg_ << ']';
    }

    template <typename UIntType>
    static void uint_to_char(UIntType num, char *cstr, std::size_t &offset)
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
    static std::size_t uint_digit(UIntType num)
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
