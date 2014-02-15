#ifndef VSMC_UTILITY_PROGRAM_OPTION_HPP
#define VSMC_UTILITY_PROGRAM_OPTION_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, func) \
    VSMC_RUNTIME_ASSERT((bool(ptr)),                                         \
            ("**vsmc::ProgramOptionMap::"#func                               \
             "** ATTEMPT TO SET OPTION WITH A NULL POINTER"))

namespace vsmc {

/// \brief Program option base class
/// \ingroup Option
class ProgramOptionBase
{
    public :

    ProgramOptionBase () {}
    ProgramOptionBase (const ProgramOptionBase &) {}
    ProgramOptionBase &operator= (const ProgramOptionBase &) {return *this;}

    virtual bool set (std::stringstream &,
            const std::string &, const std::string &) = 0;
    virtual bool set_default () = 0;
    virtual void print_help (const std::string &) const = 0;
    virtual ProgramOptionBase *clone () const = 0;
    virtual ~ProgramOptionBase () {}
}; // class ProgramOptionBase

/// \brief Class that store an option and its default value
/// \ingroup Option
template <typename T>
class ProgramOption : public ProgramOptionBase
{
    public :

    typedef T value_type;

    ProgramOption (const std::string &desc, T *ptr) :
        desc_(desc), ptr_(ptr), vec_ptr_(VSMC_NULLPTR),
        default_(T()), has_default_(false) {}

    ProgramOption (const std::string &desc, std::vector<T> *ptr) :
        desc_(desc), ptr_(VSMC_NULLPTR), vec_ptr_(ptr),
        default_(T()), has_default_(false) {}

    template <typename V>
    ProgramOption (const std::string &desc, T *ptr, const V &val) :
        desc_(desc), ptr_(ptr), vec_ptr_(VSMC_NULLPTR),
        default_(static_cast<T>(val)), has_default_(true) {}

    template <typename V>
    ProgramOption (const std::string &desc, std::vector<T> *ptr,
            const V &val) :
        desc_(desc), ptr_(VSMC_NULLPTR), vec_ptr_(ptr),
        default_(static_cast<T>(val)), has_default_(true) {}

    bool set (std::stringstream &ss,
            const std::string &oname, const std::string &sval)
    {
        ss.clear();
        ss.str(sval);
        T tval;
        ss >> tval;
        if (ss.fail()) {
            std::fprintf(stderr,
                    "Failed to set value of option '%s': %s\n",
                    oname.c_str(), sval.c_str());
            ss.clear();
            return false;
        }

        if (ptr_) *ptr_ = tval;
        if (vec_ptr_) vec_ptr_->push_back(tval);

        return true;
    }

    bool set_default ()
    {
        if (has_default_) {
            if (ptr_) *ptr_ = default_;
            if (vec_ptr_) vec_ptr_->push_back(default_);
        }

        return has_default_;
    }

    void print_help (const std::string &oname) const
    {
        std::cout << "  " << std::setw(20) << std::left << oname << desc_;
        if (has_default_)
            std::cout << " (default: " << default_ << ")";
        std::cout << std::endl;
    }

    ProgramOptionBase *clone () const
    {
        ProgramOptionBase *ptr = new ProgramOption<T>(*this);

        return ptr;
    }

    private :

    std::string desc_;
    T *const ptr_;
    std::vector<T> *const vec_ptr_;
    T default_;
    bool has_default_;
}; // class ProgramOption

/// \brief A map of ProgramOption
/// \ingroup Option
///
/// \details
/// Basic example:
/// \code
/// int main (int argc, char **argv)
/// {
///     double Value1;
///     double Value2;
///     std::vector<double> Value3;
///     std::vector<double> Value4;
///     ProgramOptionMap Map;
///     Map.add<double>("option1", "option description string 1", &Value1);
///     Map.add<double>("option2", "option description string 2", &Value2);
///     Map.add<double>("option3", "option description string 3", &Value3);
///     Map.add<double>("option4", "option description string 4", &Value4, 401);
///     if (Map.process(argc, argv)) // Help info was printed
///         return 0;
///
///     std::cout << Value1 << std::endl;
///     std::cout << Value2 << std::endl;
///     for (std::size_t i = 0; i != Value3.size(); ++i)
///         std::cout << Value3[i] << std::endl;
///     for (std::size_t i = 0; i != Value4.size(); ++i)
///         std::cout << Value4[i] << std::endl;
/// }
/// \endcode
/// And if on the command line,
/// \code
/// ./prog --option1 101 --option2 201 --option2 202 --option3 301 --option3 202 --option4 402
/// \end
/// will produce the output
/// \code
/// 101
/// 202
/// 301
/// 302
/// 401
/// 402
/// \endcode
/// An option may be specified multiple times on the command line. If the
/// option's value type is a specialization of std::vector, then all values
/// will be stored. Otherwise only the last one will be stored. A default value
/// may be specified for an option. If an option's value type is a
/// specializaiton of std::vector, then values specified on the command line
/// will be appended to the destination vector instead of override the defualt
/// value.
///
/// If add an option with a name that already exists, then it overrides the
/// previous one
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

    /// \brief Add an option
    ///
    /// \param name Name of the option, on command name it shall be specified
    /// by --name
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    template <typename T, typename Dest>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            Dest *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOption<T>(desc, ptr);
        add_option(oname, optr);

        return *this;
    }

    /// \brief Add an option with a default value
    template <typename T, typename Dest, typename V>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            Dest *ptr, const V &val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_NULLPTR(ptr, add);
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOption<T>(desc, ptr, val);
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
    /// If the option "--help" is given at the commad line, then this function
    /// does not process any other options. It merely process the options
    /// --help by printing the help information.
    ///
    /// \param argc The first argument of the `main` function
    /// \param argv The second argument of the `main` function
    ///
    /// \return `true` If the option `--help` has been specified on the command
    /// line. Otherwise `false`.
    bool process (int argc, const char **argv)
    {
        for (int ac = 1; ac != argc; ++ac) {
            if (!std::strcmp(argv[ac], "--help")) {
                for (option_map_type::iterator iter = option_map_.begin();
                        iter != option_map_.end(); ++iter) {
                    iter->second.first->print_help(iter->first);
                }
                return true;
            }
        }

        for (int ac = 1; ac < argc - 1; ++ac) {
            if (process_option(argv[ac], argv[ac + 1]))
                ++ac;
        }

        for (option_map_type::iterator iter = option_map_.begin();
                iter != option_map_.end(); ++iter) {
            if (iter->second.second == 0)
                if (iter->second.first->set_default())
                    iter->second.second = 1;
        }

        return false;
    }

    /// \brief Process the options
    bool process (int argc, char **argv)
    {
        const char **cargv = new const char *[argc];
        for (int i = 0; i != argc; ++i)
            cargv[i] = argv[i];

        return process(argc, cargv);
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

    bool process_option (const std::string &oname, const std::string &sval)
    {
        option_map_type::iterator iter = option_map_.find(oname);

        if (iter == option_map_.end())
            return false;

        if (!iter->second.first->set(ss_, iter->first, sval))
            return false;

        ++iter->second.second;
        return true;
    }
}; // class ProgramOptionMap

} // namespace vsmc

#endif // VSMC_UTILITY_PROGRAM_OPTION_HPP
