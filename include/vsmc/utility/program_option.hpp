//============================================================================
// vSMC/include/vsmc/utility/program_option.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_UTILITY_PROGRAM_OPTION_HPP
#define VSMC_UTILITY_PROGRAM_OPTION_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, func)         \
    VSMC_RUNTIME_ASSERT((ptr != nullptr),                                     \
        "**ProgramOption::" #func                                             \
        "** ATTEMPT TO SET OPTION WITH A NULL POINTER")

namespace vsmc
{

namespace internal
{

inline void program_option_warning(const std::string &name,
    const std::string &msg, bool silent, std::ostream &os)
{
    if (silent)
        return;

    os << "vSMC Program Option Warning\n";
    os << "Option: --" << name << '\n';
    os << "Message : " << msg << std::endl;
}

} // namespace vsmc::internal

/// \brief Option base class
/// \ingroup Option
class ProgramOption
{
    public:
    ProgramOption() = default;
    ProgramOption(const ProgramOption &) = default;
    ProgramOption &operator=(const ProgramOption &) = default;
    virtual ~ProgramOption() {}

    virtual bool is_bool() const = 0;
    virtual bool is_vector() const = 0;
    virtual bool set(
        const std::string &, const std::string &, bool, std::ostream &) = 0;
    virtual bool set_default() = 0;
    virtual std::string description() const = 0;
    virtual std::string default_str() const = 0;

    protected:
    bool set_value(const std::string &name, const std::string &sval,
        bool *dest, bool silent, std::ostream &os)
    {
        const char *const sptr = sval.c_str();
        const std::size_t size = sval.size();

        bool is_numb = true;
        bool is_zero = true;
        for (std::size_t i = 0; i != size; ++i) {
            char c = sptr[i];
            is_numb = is_numb && c >= '0' && c <= '9';
            is_zero = is_zero && (c == '0');
        }
        if (is_zero) {
            *dest = false;
            return true;
        } else if (is_numb) {
            *dest = true;
            return true;
        }

        bool is_true = false;
        is_true = is_true || std::strcmp(sptr, "y") == 0;
        is_true = is_true || std::strcmp(sptr, "Y") == 0;
        is_true = is_true || std::strcmp(sptr, "yes") == 0;
        is_true = is_true || std::strcmp(sptr, "Yes") == 0;
        is_true = is_true || std::strcmp(sptr, "YES") == 0;
        is_true = is_true || std::strcmp(sptr, "t") == 0;
        is_true = is_true || std::strcmp(sptr, "T") == 0;
        is_true = is_true || std::strcmp(sptr, "true") == 0;
        is_true = is_true || std::strcmp(sptr, "True") == 0;
        is_true = is_true || std::strcmp(sptr, "TRUE") == 0;
        if (is_true) {
            *dest = true;
            return true;
        }

        bool is_false = false;
        is_false = is_false || std::strcmp(sptr, "n") == 0;
        is_false = is_false || std::strcmp(sptr, "N") == 0;
        is_false = is_false || std::strcmp(sptr, "no") == 0;
        is_false = is_false || std::strcmp(sptr, "No") == 0;
        is_false = is_false || std::strcmp(sptr, "NO") == 0;
        is_false = is_false || std::strcmp(sptr, "f") == 0;
        is_false = is_false || std::strcmp(sptr, "F") == 0;
        is_false = is_false || std::strcmp(sptr, "false") == 0;
        is_false = is_false || std::strcmp(sptr, "False") == 0;
        is_false = is_false || std::strcmp(sptr, "FALSE") == 0;
        if (is_false) {
            *dest = false;
            return true;
        }

        internal::program_option_warning(
            name, "Failed to set value: " + sval, silent, os);
        return false;
    }

    template <typename T>
    bool set_value(const std::string &name, const std::string &sval, T *dest,
        bool silent, std::ostream &os)
    {
        std::stringstream ss;
        ss.str(sval);
        T tval;
        ss >> tval;
        if (ss.fail()) {
            internal::program_option_warning(
                name, "Failed to set value: " + sval, silent, os);
            ss.clear();
            return false;
        }
        *dest = std::move(tval);

        return true;
    }
}; // class ProgramOption

/// \brief Option `--help`
/// \ingroup Option
class ProgramOptionHelp : public ProgramOption
{
    public:
    ProgramOptionHelp() : help_(false) {}

    bool is_bool() const { return true; }

    bool is_vector() const { return false; }

    bool set(const std::string &name, const std::string &sval, bool silent,
        std::ostream &os)
    {
        return set_value(name, sval, &help_, silent, os);
    }

    bool set_default() { return false; }

    std::string description() const
    {
        return std::string("Print this help information");
    }

    std::string default_str() const { return std::string("(false)"); }

    bool help() const { return help_; }

    private:
    bool help_;
}; // ProgramOptionHelp

/// \brief Option with a default value
/// \ingroup Option
template <typename T>
class ProgramOptionDefault : public ProgramOption
{
    public:
    ProgramOptionDefault(const std::string &desc)
        : desc_(desc), default_(T()), has_default_(false)
    {
    }

    template <typename V>
    ProgramOptionDefault(const std::string &desc, V val)
        : desc_(desc), default_(static_cast<T>(val)), has_default_(true)
    {
    }

    bool is_bool() const { return std::is_same<T, bool>::value; }

    std::string description() const { return desc_; }

    std::string default_str() const
    {
        return has_default_ ? default_val2str(default_) : std::string();
    }

    protected:
    bool set_value_default(T *dest)
    {
        if (has_default_)
            *dest = default_;

        return has_default_;
    }

    private:
    std::string desc_;
    T default_;
    bool has_default_;

    template <typename U>
    std::string default_val2str(const U &val) const
    {
        std::stringstream ss;
        ss << '(' << val << ')';

        return ss.str();
    }

    std::string default_val2str(bool val) const
    {
        return val ? std::string("(true)") : std::string("(false)");
    }
}; // ProgramOptionDefault

/// \brief Option with a single value
/// \ingroup Option
template <typename T>
class ProgramOptionScalar : public ProgramOptionDefault<T>
{
    public:
    ProgramOptionScalar(const std::string &desc, T *ptr)
        : ProgramOptionDefault<T>(desc), ptr_(ptr)
    {
    }

    template <typename V>
    ProgramOptionScalar(const std::string &desc, T *ptr, V val)
        : ProgramOptionDefault<T>(desc, val), ptr_(ptr)
    {
    }

    bool is_vector() const { return false; }

    bool set(const std::string &name, const std::string &sval, bool silent,
        std::ostream &os)
    {
        return this->set_value(name, sval, ptr_, silent, os);
    }

    bool set_default() { return this->set_value_default(ptr_); }

    private:
    T *const ptr_;
}; // class ProgramOptionScalar

/// \brief Option with multiple values
/// \ingroup Option
template <typename T>
class ProgramOptionVector : public ProgramOptionDefault<T>
{
    public:
    ProgramOptionVector(const std::string &desc, std::vector<T> *ptr)
        : ProgramOptionDefault<T>(desc), ptr_(ptr)
    {
    }

    template <typename V>
    ProgramOptionVector(const std::string &desc, std::vector<T> *ptr, V val)
        : ProgramOptionDefault<T>(desc, val), ptr_(ptr)
    {
    }

    bool is_vector() const { return true; }

    bool set(const std::string &name, const std::string &sval, bool silent,
        std::ostream &os)
    {
        T val;
        bool success = this->set_value(name, sval, &val, silent, os);
        if (success)
            ptr_->push_back(val);

        return success;
    }

    bool set_default()
    {
        T val;
        bool success = this->set_value_default(&val);
        if (success)
            ptr_->push_back(val);

        return success;
    }

    private:
    std::vector<T> *const ptr_;
}; // class ProgramOptionVector

/// \brief Program options
/// \ingroup Option
class ProgramOptionMap
{
    public:
    explicit ProgramOptionMap(bool silent = false)
        : silent_(silent), help_ptr_(std::make_shared<ProgramOptionHelp>())
    {
        add_option("help", help_ptr_);
    }

    /// \brief Add an option with a single value
    ///
    /// \param name Name of the option, on command name it shall be specified
    /// by `--name`
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    template <typename T>
    ProgramOptionMap &add(
        const std::string &name, const std::string &desc, T *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);

        return add_option(
            name, std::make_shared<ProgramOptionScalar<T>>(desc, ptr));
    }

    /// \brief Add an option with a single value, with a default value
    template <typename T, typename V>
    ProgramOptionMap &add(
        const std::string &name, const std::string &desc, T *ptr, V val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);

        return add_option(
            name, std::make_shared<ProgramOptionScalar<T>>(desc, ptr, val));
    }

    /// \brief Add an option with multiple value
    template <typename T>
    ProgramOptionMap &add(
        const std::string &name, const std::string &desc, std::vector<T> *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);

        return add_option(
            name, std::make_shared<ProgramOptionVector<T>>(desc, ptr));
    }

    /// \brief Add an option with multiple value, with a default value
    template <typename T, typename V>
    ProgramOptionMap &add(const std::string &name, const std::string &desc,
        std::vector<T> *ptr, V val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);

        return add_option(
            name, std::make_shared<ProgramOptionVector<T>>(desc, ptr, val));
    }

    ProgramOptionMap &remove(const std::string &name)
    {
        auto iter = option_find(name);
        if (iter != option_vec_.end())
            option_vec_.erase(iter);

        return *this;
    }

    /// \brief Process the options
    ///
    /// \details
    /// If the option `--help` is given at the commad line, help information
    /// are printed.
    ///
    /// \param argc The first argument of the `main` function
    /// \param argv The second argument of the `main` function
    /// \param os The output stream used to print help information and the
    /// warning messages if any error occurs when processing the options.
    void process(int argc, const char **argv, std::ostream &os = std::cout)
    {
        std::string arg;
        Vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i) {
            arg = process_arg(argv[i]);
            if (!arg.empty())
                arg_vector.push_back(arg);
        }
        process_arg_vector(arg_vector, os);
    }

    /// \brief Process the options
    void process(int argc, char **argv, std::ostream &os = std::cout)
    {
        std::string arg;
        Vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i) {
            arg = process_arg(argv[i]);
            if (!arg.empty())
                arg_vector.push_back(arg);
        }
        process_arg_vector(arg_vector, os);
    }

    /// \brief Print help information for each option
    void print_help(std::ostream &os = std::cout) const
    {
        std::size_t len[2] = {0, 0};
        Vector<std::string> str[3];
        for (const auto &option : option_vec_) {
            str[0].push_back("--" + std::get<0>(option));
            str[1].push_back(std::get<1>(option)->description());
            str[2].push_back(std::get<1>(option)->default_str());
            len[0] = std::max(len[0], str[0].back().size());
            len[1] = std::max(len[1], str[1].back().size());
        }
        len[0] += 4;
        len[1] += 4;
        for (std::size_t i = 0; i != str[0].size(); ++i) {
            os << str[0][i] << std::string(len[0] - str[0][i].size(), ' ');
            os << str[1][i] << std::string(len[1] - str[1][i].size(), ' ');
            os << str[2][i] << std::endl;
        }
    }

    /// \brief Count the number of successful processing of an option
    std::size_t count(const std::string &name) const
    {
        auto iter = option_find(name);
        if (iter != option_vec_.end())
            return std::get<2>(*iter);
        return 0;
    }

    /// \brief Get the underlying option object
    std::shared_ptr<ProgramOption> option(const std::string &name)
    {
        auto iter = option_find(name);
        if (iter != option_vec_.end())
            return std::get<1>(*iter);
        return std::shared_ptr<ProgramOption>(
            static_cast<ProgramOption *>(nullptr));
    }

    /// \brief Get the underlying option object
    std::shared_ptr<const ProgramOption> option(const std::string &name) const
    {
        auto iter = option_find(name);
        if (iter != option_vec_.end())
            return std::get<1>(*iter);
        return std::shared_ptr<const ProgramOption>(
            static_cast<const ProgramOption *>(nullptr));
    }

    /// \brief Set the silent flag, if true, no warning messages will be
    /// printed for unknown options etc.,
    void silent(bool flag) { silent_ = flag; }

    private:
    using option_vec_type = Vector<
        std::tuple<std::string, std::shared_ptr<ProgramOption>, std::size_t>>;

    bool silent_;
    std::shared_ptr<ProgramOptionHelp> help_ptr_;
    option_vec_type option_vec_;

    option_vec_type::iterator option_find(const std::string &name)
    {
        auto iter = option_vec_.begin();
        for (; iter != option_vec_.end(); ++iter)
            if (std::get<0>(*iter) == name)
                break;

        return iter;
    }

    option_vec_type::const_iterator option_find(const std::string &name) const
    {
        auto iter = option_vec_.begin();
        for (; iter != option_vec_.end(); ++iter)
            if (std::get<0>(*iter) == name)
                break;

        return iter;
    }

    ProgramOptionMap &add_option(
        const std::string &name, std::shared_ptr<ProgramOption> optr)
    {
        auto option = std::make_tuple(name, optr, 0);
        auto iter = option_find(name);
        if (iter != option_vec_.end())
            *iter = option;
        else
            option_vec_.push_back(option);

        return *this;
    }

    void process_arg_vector(
        std::vector<std::string> &arg_vector, std::ostream &os)
    {
        Vector<std::pair<std::string, Vector<std::string>>> name_vals;
        auto arg_iter = arg_vector.begin();
        while (arg_iter != arg_vector.end() && !is_option(*arg_iter))
            ++arg_iter;
        while (arg_iter != arg_vector.end()) {
            std::string name(arg_iter->begin() + 2, arg_iter->end());
            ++arg_iter;
            Vector<std::string> svals;
            while (arg_iter != arg_vector.end() && !is_option(*arg_iter)) {
                svals.push_back(*arg_iter);
                ++arg_iter;
            }
            name_vals.push_back(std::make_pair(name, svals));
        }

        const std::string sval_true("1");
        for (auto &nv : name_vals) {
            auto iter = option_find(nv.first);
            if (iter == option_vec_.end()) {
                internal::program_option_warning(
                    std::get<0>(*iter), "Unknown option", silent_, os);
                continue;
            }

            bool proc = false;
            if (nv.second.size() == 0 && std::get<1>(*iter)->is_bool()) {
                proc = process_option(iter, sval_true, os);
            } else if (nv.second.size() == 0) {
                internal::program_option_warning(
                    std::get<0>(*iter), "No value found", silent_, os);
            } else if (std::get<1>(*iter)->is_vector()) {
                for (const auto &sval : nv.second)
                    proc = process_option(iter, sval, os);
            } else {
                for (const auto &sval : nv.second)
                    proc = process_option(iter, sval, os) || proc;
            }
        }

        for (auto &option : option_vec_)
            if (std::get<2>(option) == 0)
                if (std::get<1>(option)->set_default())
                    std::get<2>(option) = 1;

        if (help_ptr_->help())
            print_help(os);
    }

    std::string process_arg(const char *arg) const
    {
        std::size_t s = std::strlen(arg);
        std::size_t e = s;
        while (e != 0 && (arg[e - 1] == ' ' || arg[e - 1] == ','))
            --e;

        return std::string(arg, arg + e);
    }

    bool process_option(option_vec_type::iterator iter,
        const std::string &sval, std::ostream &os)
    {
        if (sval.empty()) {
            internal::program_option_warning(
                std::get<0>(*iter), "No value found", silent_, os);
            return false;
        }
        return std::get<1>(*iter)->set(std::get<0>(*iter), sval, silent_, os);
    }

    bool is_option(const std::string &str) const
    {
        if (str.size() < 3)
            return false;

        if (str[0] != '-')
            return false;

        if (str[1] != '-')
            return false;

        return true;
    }
}; // class ProgramOptionMap

} // namespace vsmc

#endif // VSMC_UTILITY_PROGRAM_OPTION_HPP
