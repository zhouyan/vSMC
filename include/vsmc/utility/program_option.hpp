//============================================================================
// include/vsmc/utility/program_option.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

/// \page po Program option
///
/// ## Introduction
///
/// Basic example
/// ~~~{.cpp}
/// #include <vsmc/utility/program_option.hpp>
///
/// int main (int argc, char **argv)
/// {
///     double Value;
///     ProgramOptionMap Map;
///     Map.add("option_name", "option description", &Value);
///     Map.process(argc, argv)
///     if (Map.count("help")) // Option --help are specified
///         return 0;
///
///     std::cout << Value << std::endl;
/// }
/// ~~~
/// And if on the command line,
/// ~~~{.sh}
/// ./prog --option_name 0.5
/// ~~~
/// This program will produce the output
/// ~~~{.txt}
/// 0.5
/// ~~~
/// In the source, options are added using `ProgramOptionMap::add`. The first
/// argument is the option name, say `option_name`. The second is a description
/// of the option. And the third is an pointer points to the destination of the
/// value. On the command line the option is specified with `--option_name`
/// followed by values.
///
/// ## Add options
///
/// The general form of the member function `ProgramOptionMap::add` is
/// ~~~{.cpp}
/// template <typename T, typename Dest, typename V>
/// ProgramOptionMap &add (const std::string &name, const std::string &desc, Dest *dest, V val);
/// ~~~
/// where `T` is the value type of the option. If the fourth (optional)
/// argument is present, it is taken to be the default value of the option. The
/// second template argument, `Dest` is the type of the destination pointer.
///
/// If `T` is the same as `Dest`, then the option is a scalar option that can
/// store a single value. Otherwise, `Dest` is taken to be a container type,
/// e.g., `std::vector<T>`, and the option can store multiple values. The
/// behavior differs when multiple values are specified in the command line.
/// Therefore, if `Dest` is a container type and is intended to be used as
/// such, the first template argument needs to be specified explicity. The only
/// exception is when `Dest` is `std::vector<T>`, in which case no template
/// argument needs to be specified. The value type `T` needs to support input
/// stream `operator>>`. The container type needs to support `push_back(T &)`,
/// in other words, it needs to be a sequential container.
///
/// To summary, the template argument deduction is shown in the following
/// example,
/// ~~~{.cpp}
/// double sOpt;
/// std::vector<double> vOpt;
/// std::list<double> lOpt;
/// vsmc::ProgramOptionMap Map;
///
/// Map.add("sName", "desc", &sOpt);        // OK, single value option. T: double, Dest: double
/// Map.add("vName", "desc", &vOpt);        // OK, multi-value option.  T: double, Dest: std::vector<double>
/// Map.add<double>("name", "desc", &lOpt); // OK, multi-value option.  T: double, Dest: std::list<double>
/// Map.add("lName", "desc", &lOpt);        // Error! T: std::list<double>, Dest: std::list<double>, no operator>>
/// ~~~
///
/// ## Process option
///
/// Options can be processed by calling `Map.process(argc, argv)`, where `argc`
/// and `argv` are the typical arguments of the `main` function.
///
/// After processing, one can use `ProgramOptionMap::count` to obtain the
/// number of a certain option being specified on the command line. For
/// example,
/// ~~~{.cpp}
/// Map.count("name");
/// ~~~
///
/// ## Specify options on the command line
///
/// On the command line, each option can be specified multiple times. After
/// each option name, zero or more values can be specified. Multiple values
/// shall be seperated by white spaces. Optionally comma can be added after
/// each value. For example,
/// ~~~{.sh}
/// ./prog --sName val               # OK
/// ./prog --sName val1 --sName val2 # OK
/// ./prog --sName val1 val2         # OK
/// ./prog --sName val1, val2        # OK
/// ./prog --sName val1,val2         # Error, white space required after comma
/// ~~~
///
/// Note that any values specified before the first option are ignored. Any
/// unknown options are also ignored. For example,
/// ~~~{.sh}
/// ./prog val1 val2 --sName val --nName val3 # "val1 val2" and "--nName val3" ignored
/// ~~~
///
/// ### Specify an option multiple times,
///
/// If the option is a single value option, it is as if only the last one is
/// used. If the option is a multi-value option, then each of them are
/// processed. Continue the example,
/// ~~~{.sh}
/// ./prog --sName val               # sOpt => val
/// ./prog --sName val1 --sName val2 # sOpt => val2
/// ./prog --vName val1 --vName val2 # vOpt => {val1, val2}
/// ~~~
///
/// ### Specify multiple values
///
/// After each option, multiple value can be specified. For each value, first
/// trailing white spaces and comma are stripped. Then, if it is a single value
/// option, values are joint into a single string, separated by a single white
/// space, and processed as a single value. If it is a multi-value option, then
/// each value are processed separately, as if they are specified multiple
/// times. For example,
/// ~~~{.sh}
/// ./prog --sName val1, val2 # Equivalent to ./prog --sName "val1 val2"
/// ./prog --vName val1, val2 # Equivalent to ./prog --vName val1 --vName val2
/// ~~~
/// A more useful example is as the following,
/// ~~~{.cpp}
/// vsmc::cxx11::uniform_real_distribution<double> runif;
/// Map.add("runif", "desc", &runif);
/// ~~~
/// ~~~{.sh}
/// ./prog --runif -1, 1        # OK. Uniform distribution on (-1, 1]
/// ./prog --runif -1 --runif 1 # Error! uniform_real_distribution operator>> requires two values
/// ~~~
///
/// If for a multi-value option, each value needs to contain multiple sub
/// values, proper shell quoting is required. For example
/// ~~~{.cpp}
/// std::vector<vsmc::cxx11::uniform_real_distribution<double> > vrunif;
/// Map.add("vrunif", "desc", &vrunif);
/// ~~~
/// ~~~{.sh}
/// ./prog --vrunif -1, 1           # Error! Equivalent to ./prog --vrunif -1 --vrunif 1
/// ./prog --vrunif "-1 1"          # OK.
/// ./prog --vrunif "-1 1", "-2 2"  # OK. vrunif now has two uniform distributions.
/// ~~~
///
/// ## Boolean options
///
/// If the value type of an option is `bool`, then it is treated specially. If
/// no values are specified, it is treated as `true`. For example,
/// ~~~{.cpp}
/// bool Flag;
/// Map.add("flag", "desc", &Flag);
/// ~~~
/// ~~~{.sh}
/// ./prog --flag # Flag => true
/// ~~~
/// If values are specified, then the following are treated as true: non-zero
/// decimal integer values, e.g., `1`, `23`. `y`, `Y`, `yes`, `Yes`, `YES`,
/// `t`, `T`, `True`, `TRUE`. The following are treated as false: zero, e.g.,
/// `0`, `00`, `n`, `N`, `no`, `No`, `NO`, `f`, `F`, `false`, `False`, `FALSE`.
/// In other words, for values formed entirely of digits, it is treated as an
/// integer and tested as in C/C++; if it is a string, then the common `yes`
/// and `no` and their short forms, lower/upper case variants are treated as
/// `true` and `false`, respectively. And common boolean literal in programming
/// languages (not only `true` and `false` as in C++) are accepted.
///
/// ## Special option `--help`
///
/// Each option has a description string. If on the command line, the `--help`
/// option is specified, then help informations will be printed. User does not
/// need to add this option. However, the user can add this option manually to
/// suppress the default printing of help informations. The default `--help` is
/// a boolean option.

#ifndef VSMC_UTILITY_PROGRAM_OPTION_HPP
#define VSMC_UTILITY_PROGRAM_OPTION_HPP

#include <vsmc/internal/common.hpp>
#include <cstring>
#include <list>
#include <map>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#define VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, func) \
    VSMC_RUNTIME_ASSERT((ptr != VSMC_NULLPTR),                               \
            ("**ProgramOptionMap::"#func                                     \
             "** ATTEMPT TO SET OPTION WITH A NULL POINTER"))

namespace vsmc {

/// \brief Program option warning messages
/// \ingroup Option
inline void program_option_warning (const std::string &oname,
        const std::string &msg, bool silent, std::ostream &os)
{
    if (silent)
        return;

    os << "vSMC Program Option Warning\n";
    os << "Option: " << oname << '\n';
    os << "Message : " << msg << std::endl;
}

/// \brief Program option base class
/// \ingroup Option
class ProgramOption
{
    public :

    ProgramOption () {}
    ProgramOption (const ProgramOption &) {}
    ProgramOption &operator= (const ProgramOption &) {return *this;}
    virtual ~ProgramOption () {}

    virtual bool is_bool () const = 0;
    virtual bool is_vector () const = 0;
    virtual bool set (const std::string &, const std::string &, bool,
            std::ostream &) = 0;
    virtual bool set_default () = 0;
    virtual std::string description () const = 0;
    virtual std::string default_str () const = 0;
    virtual ProgramOption *clone () const = 0;

    protected :

    bool set_value (const std::string &oname, const std::string &sval,
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
        is_true = is_true || std::strcmp(sptr, "y")    == 0;
        is_true = is_true || std::strcmp(sptr, "Y")    == 0;
        is_true = is_true || std::strcmp(sptr, "yes")  == 0;
        is_true = is_true || std::strcmp(sptr, "Yes")  == 0;
        is_true = is_true || std::strcmp(sptr, "YES")  == 0;
        is_true = is_true || std::strcmp(sptr, "t")    == 0;
        is_true = is_true || std::strcmp(sptr, "T")    == 0;
        is_true = is_true || std::strcmp(sptr, "true") == 0;
        is_true = is_true || std::strcmp(sptr, "True") == 0;
        is_true = is_true || std::strcmp(sptr, "TRUE") == 0;
        if (is_true) {
            *dest = true;
            return true;
        }

        bool is_false = false;
        is_false = is_false || std::strcmp(sptr, "n")     == 0;
        is_false = is_false || std::strcmp(sptr, "N")     == 0;
        is_false = is_false || std::strcmp(sptr, "no")    == 0;
        is_false = is_false || std::strcmp(sptr, "No")    == 0;
        is_false = is_false || std::strcmp(sptr, "NO")    == 0;
        is_false = is_false || std::strcmp(sptr, "f")     == 0;
        is_false = is_false || std::strcmp(sptr, "F")     == 0;
        is_false = is_false || std::strcmp(sptr, "false") == 0;
        is_false = is_false || std::strcmp(sptr, "False") == 0;
        is_false = is_false || std::strcmp(sptr, "FALSE") == 0;
        if (is_false) {
            *dest = false;
            return true;
        }

        program_option_warning(oname, "Failed to set value: " + sval,
                silent, os);
        return false;
    }

    template <typename T>
    bool set_value (const std::string &oname, const std::string &sval,
            T *dest, bool silent, std::ostream &os)
    {
        std::stringstream ss;
        ss.str(sval);
        T tval;
        ss >> tval;
        if (ss.fail()) {
            program_option_warning(oname, "Failed to set value: " + sval,
                    silent, os);
            ss.clear();
            return false;
        }
#if VSMC_HAS_CXX11_RVALUE_REFERENCES
        *dest = cxx11::move(tval);
#else
        *dest = tval;
#endif

        return true;
    }
}; // class ProgramOption

/// \brief Option `--help`
/// \ingroup Option
class ProgramOptionHelp : public ProgramOption
{
    public :

    ProgramOptionHelp () : help_(false) {}

    bool is_bool () const {return true;}

    bool is_vector () const {return false;}

    bool set (const std::string &oname, const std::string &sval, bool silent,
            std::ostream &os)
    {return set_value(oname, sval, &help_, silent, os);}

    bool set_default () {return false;}

    std::string description () const
    {return std::string("Print this help information");}

    std::string default_str () const
    {return std::string("false");}

    ProgramOption *clone () const {return new ProgramOptionHelp;}

    bool help () const {return help_;}

    private :

    bool help_;
}; // ProgramOptionHelp

/// \brief Option with a default value
/// \ingroup Option
template <typename T>
class ProgramOptionDefault : public ProgramOption
{
    public :

    ProgramOptionDefault (const std::string &desc) :
        desc_(desc), default_(T()), has_default_(false) {}

    template <typename V>
    ProgramOptionDefault (const std::string &desc, V val) :
        desc_(desc), default_(static_cast<T>(val)), has_default_(true) {}

    bool is_bool () const {return cxx11::is_same<T, bool>::value;}

    std::string description () const {return desc_;}

    std::string default_str () const
    {return has_default_ ? default_val2str(default_) : std::string();}

    protected :

    bool set_value_default (T *dest)
    {
        if (has_default_)
            *dest = default_;

        return has_default_;
    }

    private :

    std::string desc_;
    T default_;
    bool has_default_;

    template <typename U>
    std::string default_val2str (const U &val) const
    {
        std::stringstream ss;
        ss << val;

        return ss.str();
    }

    std::string default_val2str (bool val) const
    {return val ? std::string("true") : std::string("false");}
}; // ProgramOptionDefault

/// \brief Option with a single value
/// \ingroup Option
template <typename T>
class ProgramOptionScalar : public ProgramOptionDefault<T>
{
    public :

    ProgramOptionScalar (const std::string &desc, T *ptr) :
        ProgramOptionDefault<T>(desc), ptr_(ptr) {}

    template <typename V>
    ProgramOptionScalar (const std::string &desc, T *ptr, V val) :
        ProgramOptionDefault<T>(desc, val), ptr_(ptr) {}

    bool is_vector () const {return false;}

    bool set (const std::string &oname, const std::string &sval, bool silent,
            std::ostream &os)
    {return this->set_value(oname, sval, ptr_, silent, os);}

    bool set_default () {return this->set_value_default(ptr_);}

    ProgramOption *clone () const
    {return new ProgramOptionScalar<T>(*this);}

    private :

    T *const ptr_;
}; // class ProgramOptionScalar

/// \brief Option with multiple values
/// \ingroup Option
template <typename T>
class ProgramOptionVector : public ProgramOptionDefault<T>
{
    public :

    ProgramOptionVector (const std::string &desc, std::vector<T> *ptr) :
        ProgramOptionDefault<T>(desc), val_(T()), ptr_(ptr) {}

    template <typename V>
    ProgramOptionVector (const std::string &desc, std::vector<T> *ptr, V val) :
        ProgramOptionDefault<T>(desc, val), val_(T()), ptr_(ptr) {}

    bool is_vector () const {return true;}

    bool set (const std::string &oname, const std::string &sval, bool silent,
            std::ostream &os)
    {
        bool success = this->set_value(oname, sval, &val_, silent, os);

        if (success)
            ptr_->push_back(val_);

        return success;
    }

    bool set_default ()
    {
        bool success = this->set_value_default(&val_);

        if (success)
            ptr_->push_back(val_);

        return success;
    }

    ProgramOption *clone () const {return new ProgramOptionVector<T>(*this);}

    private :

    T val_;
    std::vector<T> *const ptr_;
}; // class ProgramOptionVector

/// \brief A map of ProgramOption
/// \ingroup Option
class ProgramOptionMap
{
    typedef std::map<std::string, std::pair<ProgramOption *, std::size_t> >
        option_map_type;
    typedef std::list<std::pair<std::string, const ProgramOption *> >
        option_list_type;

    public :

    explicit ProgramOptionMap (bool silent = false, bool auto_help = true) :
        silent_(silent), auto_help_(auto_help),
        help_ptr_(new ProgramOptionHelp)
    {
        option_map_["--help"] = std::make_pair(help_ptr_, 0);
        option_list_.push_back(std::make_pair("--help", help_ptr_));
    }

    ProgramOptionMap (const ProgramOptionMap &other) :
        silent_(other.silent_), auto_help_(other.auto_help_),
        option_map_(other.option_map_), option_list_(other.option_list_)
    {
        for (option_map_type::iterator iter = option_map_.begin();
                iter != option_map_.end(); ++iter) {
            if (iter->second.first)
                iter->second.first = iter->second.first->clone();
        }
    }

    ProgramOptionMap &operator= (const ProgramOptionMap &other)
    {
        if (this != &other) {
            silent_ = other.silent_;
            auto_help_ = other.auto_help_;
            for (option_map_type::iterator iter = option_map_.begin();
                    iter != option_map_.end(); ++iter) {
                if (iter->second.first)
                    delete iter->second.first;
            }

            option_map_ = other.option_map_;
            option_list_ = other.option_list_;

            for (option_map_type::iterator iter = option_map_.begin();
                    iter != option_map_.end(); ++iter) {
                if (iter->second.first)
                    iter->second.first = iter->second.first->clone();
            }
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    ProgramOptionMap (ProgramOptionMap &&other) :
        silent_(other.silent_), auto_help_(other.auto_help_),
        help_ptr_(other.help_ptr_),
        option_map_(cxx11::move(other.option_map_)),
        option_list_(cxx11::move(other.option_list_))
    {
        other.help_ptr_ = VSMC_NULLPTR;
        other.option_map_.clear();
        other.option_list_.clear();
    }

    ProgramOptionMap &operator= (ProgramOptionMap &&other)
    {
        if (this != &other) {
            silent_ = other.silent_;
            help_ptr_ = other.help_ptr_;
            option_map_ = cxx11::move(other.option_map_);
            option_list_ = cxx11::move(other.option_list_);
            other.help_ptr_ = VSMC_NULLPTR;
            other.option_map_.clear();
            other.option_list_.clear();
        }

        return *this;
    }
#endif

    ~ProgramOptionMap ()
    {
        for (option_map_type::iterator iter = option_map_.begin();
                iter != option_map_.end(); ++iter) {
            if (iter->second.first != VSMC_NULLPTR)
                delete iter->second.first;
        }
    }

    /// \brief Add an option with a single value
    ///
    /// \param name Name of the option, on command name it shall be specified
    /// by `--name`
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    template <typename T>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            T *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOption *optr = new ProgramOptionScalar<T>(desc, ptr);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with a single value, with a default value
    template <typename T, typename V>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            T *ptr, V val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOption *optr = new ProgramOptionScalar<T>(desc, ptr, val);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with multiple value
    template <typename T>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            std::vector<T> *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOption *optr = new ProgramOptionVector<T>(desc, ptr);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with multiple value, with a default value
    template <typename T, typename V>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            std::vector<T> *ptr, V val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOption *optr = new ProgramOptionVector<T>(desc, ptr, val);
        add_option(oname, optr);

        return *this;
    }

    ProgramOptionMap &remove (const std::string &name)
    {
        const std::string oname("--" + name);
        option_map_type::iterator iter = option_map_.find(oname);
        if (iter != option_map_.end()) {
            if (iter->second.first != VSMC_NULLPTR)
                delete iter->second.first;
            option_map_.erase(iter);
            option_list_type::iterator liter = option_list_find(oname);
            option_list_.erase(liter);
        }

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
    /// \param os The output stream used to print help information if
    /// `auto_help` is set to true, and the warning messages if any error
    /// occurs when processing the options.
    void process (int argc, const char **argv, std::ostream &os = std::cout)
    {
        std::string arg;
        std::vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i) {
            arg = process_arg(argv[i]);
            if (!arg.empty())
                arg_vector.push_back(arg);
        }
        process_arg_vector(arg_vector, os);
    }

    /// \brief Process the options
    void process (int argc, char **argv, std::ostream &os = std::cout)
    {
        std::string arg;
        std::vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i) {
            arg = process_arg(argv[i]);
            if (!arg.empty())
                arg_vector.push_back(arg);
        }
        process_arg_vector(arg_vector, os);
    }

    /// \brief Print help information for each option
    void print_help (std::ostream &os = std::cout) const
    {
        std::size_t len[2] = {0, 0};
        std::vector<std::string> vec[3];
        for (option_list_type::const_iterator liter = option_list_.begin();
                liter != option_list_.end(); ++liter) {
            vec[0].push_back(liter->first);
            vec[1].push_back(liter->second->description());
            vec[2].push_back(liter->second->default_str());
            if (len[0] < vec[0].back().size())
                len[0] = vec[0].back().size();
            if (len[1] < vec[1].back().size())
                len[1] = vec[1].back().size();
        }
        len[0] += 4;
        len[1] += 4;
        for (std::size_t i = 0; i != vec[0].size(); ++i) {
            os << vec[0][i] << std::string(len[0] - vec[0][i].size(), ' ');
            os << vec[1][i] << std::string(len[1] - vec[1][i].size(), ' ');
            os << "(default:" << vec[2][i] << ')' << std::endl;
        }
    }

    /// \brief Count the number of successful processing of an option
    std::size_t count (const std::string &name) const
    {
        option_map_type::const_iterator iter = option_map_.find("--" + name);
        if (iter != option_map_.end())
            return iter->second.second;
        else
            return 0;
    }

    /// \brief Get the underlying option object
    const ProgramOption *option (const std::string &name) const
    {
        option_map_type::const_iterator iter = option_map_.find("--" + name);
        if (iter != option_map_.end())
            return iter->second.first;
        else
            return VSMC_NULLPTR;
    }

    /// \brief Get the underlying option object
    ProgramOption *option (const std::string &name)
    {
        option_map_type::const_iterator iter = option_map_.find("--" + name);
        if (iter != option_map_.end())
            return iter->second.first;
        else
            return VSMC_NULLPTR;
    }

    /// \brief Set the silent flag, if true, no warning messages will be
    /// printed for unknown options etc.,
    void silent (bool flag) {silent_ = flag;}

    /// \brief Set the auto_help flag, if true, help information is printed
    /// automatically when the `--help` option is processed
    void auto_help (bool flag) {auto_help_ = flag;}

    private :

    bool silent_;
    bool auto_help_;
    ProgramOptionHelp *help_ptr_;
    option_map_type option_map_;
    option_list_type option_list_;

    option_list_type::iterator option_list_find (const std::string &oname)
    {
        option_list_type::iterator liter = option_list_.begin();
        for (; liter != option_list_.end(); ++liter) {
            if (liter->first == oname)
                break;
        }

        return liter;
    }

    void add_option (const std::string &oname, ProgramOption *optr)
    {
        std::pair<option_map_type::iterator, bool> insert =
            option_map_.insert(std::make_pair(oname, std::make_pair(optr, 0)));
        if (insert.second) {
            option_list_.push_back(std::make_pair(oname, optr));
        } else {
            if (insert.first->second.first != VSMC_NULLPTR)
                delete insert.first->second.first;
            insert.first->second.first = optr;
            option_list_type::iterator liter = option_list_find(oname);
            liter->second = optr;
        }
    }

    void process_arg_vector (std::vector<std::string> &arg_vector,
            std::ostream &os)
    {
        std::string option_value;
        const std::vector<std::string> option_value_vec;
        std::vector<std::pair<std::string, std::vector<std::string> > >
            option_vector;
        std::vector<std::string>::iterator aiter = arg_vector.begin();
        while (aiter != arg_vector.end() && !is_option(*aiter))
            ++aiter;
        while (aiter != arg_vector.end()) {
            option_vector.push_back(std::make_pair(*aiter, option_value_vec));
            std::vector<std::string> &value = option_vector.back().second;
            ++aiter;
            while (aiter != arg_vector.end() &&!is_option(*aiter)) {
                    value.push_back(*aiter);
                ++aiter;
            }
        }

        const std::string sval_true("1");
        for (std::vector<std::pair<std::string, std::vector<std::string> > >::
                iterator iter = option_vector.begin();
                iter != option_vector.end(); ++iter) {
            option_map_type::iterator miter = option_map_.find(iter->first);
            if (miter == option_map_.end()) {
                program_option_warning(iter->first, "Unknown option",
                        silent_, os);
                continue;
            }

            bool proc = false;
            const std::size_t vsize = iter->second.size();
            if (vsize == 0 && miter->second.first->is_bool()) {
                proc = process_option(miter, sval_true, os);
            } else if (vsize == 0) {
                program_option_warning(miter->first, "Value not found",
                        silent_, os);
            } else if (!miter->second.first->is_vector()) {
                option_value.clear();
                for (std::size_t i = 0; i != vsize - 1; ++i)
                    option_value += iter->second[i] + ' ';
                option_value += iter->second[vsize - 1];
                proc = process_option(miter, option_value, os);
            } else {
                for (std::size_t i = 0; i != vsize; ++i)
                    proc = process_option(miter, iter->second[i], os) || proc;
            }
            if (proc)
                ++miter->second.second;
        }

        for (option_map_type::iterator iter = option_map_.begin();
                iter != option_map_.end(); ++iter) {
            if (iter->second.second == 0)
                if (iter->second.first->set_default())
                    iter->second.second = 1;
        }

        if (auto_help_ && help_ptr_->help())
            print_help(os);
    }

    std::string process_arg (const char *arg) const
    {
        std::size_t s = std::strlen(arg);
        std::size_t e = s;
        while (e != 0 && (arg[e - 1] == ' ' || arg[e - 1] == ','))
            --e;

        return std::string(arg, arg + e);
    }

    bool process_option (option_map_type::iterator iter,
            const std::string &sval, std::ostream &os)
    {
        if (sval.empty()) {
            program_option_warning(iter->first, "No value found", silent_, os);
            return false;
        }

        return iter->second.first->set(iter->first, sval, silent_, os);
    }

    bool is_option (const std::string &str) const
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
