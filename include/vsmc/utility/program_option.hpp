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
    std::fprintf(stderr, "vSMC Program option error: option: %s: %s",
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

    virtual bool set (std::stringstream &, const std::string &,
            const std::string &) = 0;
    virtual bool set_default () = 0;
    virtual void print_help (const std::string &) const = 0;
    virtual ProgramOptionBase *clone () const = 0;
    virtual ~ProgramOptionBase () {}

    protected :

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

/// \brief Opiton with a default value
/// \ingroup Option
///
/// \details
/// Basic example
/// ~~~{.cpp}
/// #include <vsmc/utility/program_option.hpp>
///
/// int main (int argc, char **argv)
/// {
///     double Value;
///     ProgramOptionMap Map;
///     Map.add("option_name", "option description", &Value);
///     if (Map.process(argc, argv)) // Help info was printed
///         return 0;
///
///     std::cout << Value << std::endl;
/// }
/// ~~~
/// And if on the command line,
/// ~~~{.sh}
/// ./prog --option_name 0.5
/// ~~~
/// will produce the output
/// ~~~{.txt}
/// 0.5
/// ~~~
template <typename T>
class ProgramOptionDefault : public ProgramOptionBase
{
    public :

    ProgramOptionDefault (const std::string &desc) :
        desc_(desc), default_(T()), has_default_(false) {}

    template <typename V>
    ProgramOptionDefault (const std::string &desc, const V &val) :
        desc_(desc), default_(static_cast<T>(val)), has_default_(true) {}

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
template <typename T>
class ProgramOptionVector : public ProgramOptionDefault<T>
{
    public :

    ProgramOptionVector (const std::string &desc, std::vector<T> *ptr) :
        ProgramOptionDefault<T>(desc), val_(T()), ptr_(ptr) {}

    template <typename V>
    ProgramOptionVector (const std::string &desc, std::vector<T> *ptr,
            const V &val) :
        ProgramOptionDefault<T>(desc, val), val_(T()), ptr_(ptr) {}

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
    {return new ProgramOptionVector<T>(*this);}

    private :

    T val_;
    std::vector<T> *const ptr_;
}; // class ProgramOptionVector

/// \brief A map of ProgramOption
/// \ingroup Option
class ProgramOptionMap
{
    public :

    typedef std::map<std::string, std::pair<ProgramOptionBase *, std::size_t> >
        option_map_type;

    ProgramOptionMap () {}

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

    /// \brief Add an opiton with multiple value
    template <typename T>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            std::vector<T> *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOptionVector<T>(desc, ptr);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an opiton with multiple value, with a default value
    template <typename T, typename V>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            std::vector<T> *ptr, const V &val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOptionVector<T>(desc, ptr, val);
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
    ///
    /// \return `true` If the option `--help` has been specified on the command
    /// line. Otherwise `false`.
    bool process (int argc, const char **argv)
    {
        std::vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i)
            arg_vector.push_back(argv[i]);

        return process_option(arg_vector);
    }

    /// \brief Process the options
    bool process (int argc, char **argv)
    {
        std::vector<std::string> arg_vector;
        arg_vector.reserve(static_cast<std::size_t>(argc));
        for (int i = 0; i != argc; ++i)
            arg_vector.push_back(argv[i]);

        return process_option(arg_vector);
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

    option_map_type option_map_;
    mutable std::stringstream ss_;

    void add_option (const std::string &oname, ProgramOptionBase *optr)
    {
        std::pair<option_map_type::iterator, bool> set =
            option_map_.insert(std::make_pair(oname,
                        std::make_pair(optr, static_cast<std::size_t>(0))));
        if (!set.second) {
            if (set.first->second.first)
                delete set.first->second.first;
            set.first->second.first = optr;
        }
    }

    bool process_option (std::vector<std::string> &arg_vector)
    {
        bool help = false;
        const std::string oname_help("--help");
        const std::vector<std::string> empty_value;
        std::vector<std::pair<std::string, std::vector<std::string> > >
            option_vector;

        std::vector<std::string>::iterator aiter = arg_vector.begin();
        while (!is_option(*aiter) && aiter != arg_vector.end())
            ++aiter;
        while (aiter != arg_vector.end()) {
            if (*aiter == oname_help) {
                help = true;
                while (!is_option(*aiter) && aiter != arg_vector.end())
                    ++aiter;
            }
            option_vector.push_back(std::make_pair(*aiter, empty_value));
            std::vector<std::string> &value = option_vector.back().second;
            ++aiter;
            while (!is_option(*aiter) && aiter != arg_vector.end()) {
                value.push_back(*aiter);
                ++aiter;
            }
        }

        if (help) {
            for (option_map_type::iterator iter = option_map_.begin();
                    iter != option_map_.end(); ++iter) {
                iter->second.first->print_help(iter->first);
            }
        }

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
            for (std::size_t i = 0; i != vsize; ++i)
                proc = process_option(miter, iter->second[i]) || proc;
            if (proc)
                ++miter->second.second;
            else
                program_option_error(iter->first, "Failed to set value");
        }

        for (option_map_type::iterator iter = option_map_.begin();
                iter != option_map_.end(); ++iter) {
            if (iter->second.second == 0)
                if (iter->second.first->set_default())
                    iter->second.second = 1;
        }

        return help;
    }

    bool process_option (option_map_type::iterator iter, std::string &sval)
    {
        std::size_t e = sval.size();
        std::size_t n = 0;
        while (e != 0 && (sval[e] == ' ' || sval[e] == ',')) {
            ++n;
            --e;
        }
        if (n != 0)
            sval.erase(++e, n);

        if (sval.empty())
            return false;

        if (!iter->second.first->set(ss_, iter->first, sval))
            return false;

        return true;
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
