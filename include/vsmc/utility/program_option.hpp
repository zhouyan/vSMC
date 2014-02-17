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
/// ProgramOptionMap &add (const std::string &name, const std::string &desc, Dest *dest, const V &val);
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

#define VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, func) \
    VSMC_RUNTIME_ASSERT((bool(ptr)),                                         \
            ("**vsmc::ProgramOptionMap::"#func                               \
             "** ATTEMPT TO SET OPTION WITH A NULL POINTER"))

namespace vsmc {

/// \brief Program option error messages
/// \ingroup Option
inline void program_option_error (const std::string &oname,
        const std::string &msg)
{
    std::fprintf(stderr, "vSMC Program option error: option %s: %s\n",
            oname.c_str(), msg.c_str());
}

/// \brief Program option base class
/// \ingroup Option
class ProgramOptionBase
{
    public :

    ProgramOptionBase () {}
    ProgramOptionBase (const ProgramOptionBase &) {}
    ProgramOptionBase &operator= (const ProgramOptionBase &) {return *this;}

    virtual bool is_bool () const = 0;
    virtual bool is_vector () const = 0;
    virtual bool set (std::stringstream &, const std::string &,
            const std::string &) = 0;
    virtual bool set_default () = 0;
    virtual void print_help (const std::string &) const = 0;
    virtual ProgramOptionBase *clone () const = 0;
    virtual ~ProgramOptionBase () {}

    protected :

    bool set_value (std::stringstream &, const std::string &oname,
            const std::string &sval, bool *dest)
    {
        const char *const sptr = sval.c_str();
        const std::size_t size = sval.size();

        bool is_numb = true;
        bool is_zero = true;
        for (std::size_t i = 0; i != size; ++i) {
            is_numb = is_numb && std::isdigit(sptr[i]);
            is_zero = is_zero && (sptr[i] == '0');
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

        program_option_error(oname, "Failed to set boolean value: " + sval);
        return false;
    }

    template <typename T>
    bool set_value (std::stringstream &ss, const std::string &oname,
            const std::string &sval, T *dest)
    {
        ss.clear();
        ss.str(sval);
        T tval;
        ss >> tval;
        if (ss.fail()) {
            program_option_error(oname, "Failed to set value: " + sval);
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
}; // class ProgramOptionBase

/// \brief Option --help
/// \ingroup Option
class ProgramOptionHelp : public ProgramOptionBase
{
    public :

    ProgramOptionHelp () : help_(false) {}

    bool is_bool () const {return true;}

    bool is_vector () const {return false;}

    bool set (std::stringstream &ss, const std::string &,
            const std::string &sval)
    {return set_value(ss, "--help", sval, &help_);}

    bool set_default () {return false;}

    void print_help (const std::string &) const
    {
        std::cout << "  " << std::setw(20) << std::left << "--help";
        std::cout << "Print help information" << std::endl;
    }

    ProgramOptionBase *clone () const {return new ProgramOptionHelp;}

    bool help () const {return help_;}

    private :

    bool help_;
}; // ProgramOptionHelp

/// \brief Option with a default value
/// \ingroup Option
template <typename T>
class ProgramOptionDefault : public ProgramOptionBase
{
    public :

    ProgramOptionDefault (const std::string &desc) :
        desc_(desc), default_(T()), has_default_(false) {}

    template <typename V>
    ProgramOptionDefault (const std::string &desc, const V &val) :
        desc_(desc), default_(static_cast<T>(val)), has_default_(true) {}

    bool is_bool () const {return cxx11::is_same<T, bool>::value;}

    void print_help (const std::string &oname) const
    {
        std::cout << "  " << std::setw(20) << std::left << oname << desc_;
        if (has_default_)
            std::cout << " (default: " << default_ << ")";
        std::cout << std::endl;
    }

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
    ProgramOptionScalar (const std::string &desc, T *ptr, const V &val) :
        ProgramOptionDefault<T>(desc, val), ptr_(ptr) {}

    bool is_vector () const {return false;}

    bool set (std::stringstream &ss, const std::string &oname,
            const std::string &sval)
    {return this->set_value(ss, oname, sval, ptr_);}

    bool set_default () {return this->set_value_default(ptr_);}

    ProgramOptionBase *clone () const
    {return new ProgramOptionScalar<T>(*this);}

    private :

    T *const ptr_;
}; // class ProgramOptionScalar

/// \brief Option with multiple values
/// \ingroup Option
template <typename T, typename Cont>
class ProgramOptionVector : public ProgramOptionDefault<T>
{
    public :

    ProgramOptionVector (const std::string &desc, Cont *ptr) :
        ProgramOptionDefault<T>(desc), val_(T()), ptr_(ptr) {}

    template <typename V>
    ProgramOptionVector (const std::string &desc, Cont *ptr,
            const V &val) :
        ProgramOptionDefault<T>(desc, val), val_(T()), ptr_(ptr) {}

    bool is_vector () const {return true;}

    bool set (std::stringstream &ss, const std::string &oname,
            const std::string &sval)
    {
        bool success = this->set_value(ss, oname, sval, &val_);

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

    ProgramOptionBase *clone () const
    {return new ProgramOptionVector<T, Cont>(*this);}

    private :

    T val_;
    Cont *const ptr_;
}; // class ProgramOptionVector

/// \brief A map of ProgramOption
/// \ingroup Option
class ProgramOptionMap
{
    public :

    typedef std::map<std::string, std::pair<ProgramOptionBase *, std::size_t> >
        option_map_type;

    ProgramOptionMap ()
    {
        help_ptr_ = new ProgramOptionHelp;
        option_map_["--help"] = std::make_pair(help_ptr_, 0);
    }

    ProgramOptionMap (const ProgramOptionMap &other) :
        option_map_(other.option_map_)
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
            for (option_map_type::iterator iter = option_map_.begin();
                    iter != option_map_.end(); ++iter) {
                if (iter->second.first)
                    delete iter->second.first;
            }

            option_map_ = other.option_map_;

            for (option_map_type::iterator iter = option_map_.begin();
                    iter != option_map_.end(); ++iter) {
                if (iter->second.first)
                    iter->second.first = iter->second.first->clone();
            }
        }

        return *this;
    }

    ~ProgramOptionMap ()
    {
        for (option_map_type::iterator iter = option_map_.begin();
                iter != option_map_.end(); ++iter) {
            if (iter->second.first)
                delete iter->second.first;
        }
    }

    /// \brief Add an option with a single value
    ///
    /// \param name Name of the option, on command name it shall be specified
    /// by --name
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    template <typename T>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            T *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOptionScalar<T>(desc, ptr);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with a single value, with a default value
    template <typename T, typename V>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            T *ptr, const V &val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOptionScalar<T>(desc, ptr, val);
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
        ProgramOptionBase *optr =
            new ProgramOptionVector<T, std::vector<T> >(desc, ptr);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with multiple value, with a default value
    template <typename T, typename V>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            std::vector<T> *ptr, const V &val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr =
            new ProgramOptionVector<T, std::vector<T> >(desc, ptr, val);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with multiple value with a container other than
    /// `std::vector`
    template <typename T, typename Cont>
    typename cxx11::enable_if<!cxx11::is_same<T, Cont>::value,
             ProgramOptionMap &>::type
    add (const std::string &name, const std::string &desc, Cont *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr =
            new ProgramOptionVector<T, Cont>(desc, ptr);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with multiple value, with a default value, with a
    /// container other than `std::vector`.
    template <typename T, typename Cont, typename V>
    typename cxx11::enable_if<!cxx11::is_same<T, Cont>::value,
             ProgramOptionMap &>::type
    add (const std::string &name, const std::string &desc, Cont *ptr,
            const V &val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOptionVector<T, Cont>(desc, ptr, val);
        add_option(oname, optr);

        return *this;
    }

    ProgramOptionMap &remove (const std::string &name)
    {
        option_map_type::iterator iter = option_map_.find("--" + name);
        if (iter != option_map_.end()) {
            if (iter->second.first)
                delete iter->second.first;
            option_map_.erase(iter);
        }

        return *this;
    }

    /// \brief Process the options
    ///
    /// \details
    /// If the option "--help" is given at the commad line, help information
    /// are printed.
    ///
    /// \param argc The first argument of the `main` function
    /// \param argv The second argument of the `main` function
    void process (int argc, const char **argv)
    {
        std::vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i)
            arg_vector.push_back(argv[i]);
        process_arg(arg_vector);
    }

    /// \brief Process the options
    void process (int argc, char **argv)
    {
        std::vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i)
            arg_vector.push_back(argv[i]);
        process_arg(arg_vector);
    }

    /// \brief Print help information for each option
    void print_help () const
    {
        for (option_map_type::const_iterator iter = option_map_.begin();
                iter != option_map_.end(); ++iter) {
            iter->second.first->print_help(iter->first);
        }
    }

    /// \brief Count the number of occurence of an option on the command line
    /// given its name
    std::size_t count (const std::string &name) const
    {
        option_map_type::const_iterator iter = option_map_.find("--" + name);
        if (iter != option_map_.end())
            return iter->second.second;
        else
            return 0;
    }

    private :

    ProgramOptionHelp *help_ptr_;
    option_map_type option_map_;
    mutable std::stringstream ss_;

    void add_option (const std::string &oname, ProgramOptionBase *optr)
    {
        std::pair<option_map_type::iterator, bool> insert =
            option_map_.insert(std::make_pair(oname, std::make_pair(optr, 0)));
        if (!insert.second) {
            if (insert.first->second.first)
                delete insert.first->second.first;
            insert.first->second.first = optr;
        }
    }

    void process_arg (std::vector<std::string> &arg_vector)
    {
        const std::vector<std::string> empty_value;
        std::vector<std::pair<std::string, std::vector<std::string> > >
            option_vector;
        std::vector<std::string>::iterator aiter = arg_vector.begin();
        while (aiter != arg_vector.end() && !is_option(*aiter))
            ++aiter;
        while (aiter != arg_vector.end()) {
            option_vector.push_back(std::make_pair(*aiter, empty_value));
            std::vector<std::string> &value = option_vector.back().second;
            ++aiter;
            while (aiter != arg_vector.end() &&!is_option(*aiter)) {
                value.push_back(*aiter);
                ++aiter;
            }
        }

        const std::string sval_true("1");
        std::string vsval;
        for (std::vector<std::pair<std::string, std::vector<std::string> > >::
                iterator iter = option_vector.begin();
                iter != option_vector.end(); ++iter) {
            option_map_type::iterator miter = option_map_.find(iter->first);
            if (miter == option_map_.end()) {
                program_option_error(iter->first, "Unknown option ignored");
                continue;
            }

            bool proc = false;
            const std::size_t vsize = iter->second.size();
            if (vsize == 0 && miter->second.first->is_bool()) {
                proc = process_option(miter, sval_true);
            } else if (vsize == 0) {
                program_option_error(miter->first, "No value specified");
            } else if (!miter->second.first->is_vector()) {
                vsval.clear();
                for (std::size_t i = 0; i != vsize; ++i) {
                    process_value(iter->second[i]);
                    vsval += iter->second[i] + ' ';
                }
                proc = process_option(miter, vsval);
            } else {
                for (std::size_t i = 0; i != vsize; ++i) {
                    process_value(iter->second[i]);
                    proc = process_option(miter, iter->second[i]) || proc;
                }
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

        if (help_ptr_->help())
            print_help();
    }

    void process_value (std::string &sval)
    {
        std::size_t e = sval.size();
        std::size_t n = 0;
        while (e != 0 && (sval[e - 1] == ' ' || sval[e - 1] == ',')) {
            ++n;
            --e;
        }
        if (n != 0)
            sval.erase(e, n);
    }

    bool process_option (option_map_type::iterator iter,
            const std::string &sval)
    {
        if (sval.empty()) {
            program_option_error(iter->first, "No value specified");
            return false;
        }

        return iter->second.first->set(ss_, iter->first, sval);
    }

    bool is_option (const std::string &str)
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
