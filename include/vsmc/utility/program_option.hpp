#ifndef VSMC_UTILITY_PROGRAM_OPTION_HPP
#define VSMC_UTILITY_PROGRAM_OPTION_HPP

#include <vsmc/internal/common.hpp>

#ifndef NDEBUG
#define VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_DUPLICATE(set, name) \
{                                                                            \
    if (set.count(name) == 1) {                                              \
        std::string msg("**vsmc::ProgramOptionMap::add** Option \"");        \
        msg += name;                                                         \
        msg += "\" ALREADY EXISTS";                                          \
        VSMC_RUNTIME_ASSERT(false, msg.c_str());                             \
    }                                                                        \
}
#else
#define VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_DUPLICATE(set, name)
#endif

namespace vsmc {

/// \brief Program opiton base class
/// \ingroup Option
class ProgramOptionBase
{
    public :

    virtual bool compare_set (const char *, const char *) = 0;
    virtual void print_help () const = 0;
    virtual ~ProgramOptionBase () {}
};

/// \brief Class that store an option and its default value
/// \ingroup Option
template <typename T>
class ProgramOption : public ProgramOptionBase
{
    public :

    /// \brief Value type of this option
    typedef T value_type;

    /// \brief Construct an option that can store a single value
    ///
    /// \param name Name of the option, on command name it shall be specified
    /// by --name
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    ProgramOption (const char *name, const char *desc, T *ptr) :
        name_(std::string("--") + name),
        desc_(desc), ptr_(ptr), vec_ptr_(VSMC_NULLPTR), has_default_(false) {}

    /// \brief Construct an option that can store multiple values
    ///
    /// \param name Name of the option, on command name it shall be specified
    /// by --name. The option can be specified multiple times on the command
    /// line. Each value will be stored within the vector
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    ProgramOption (const char *name, const char *desc, std::vector<T> *ptr) :
        name_(std::string("--") + name),
        desc_(desc), ptr_(VSMC_NULLPTR), vec_ptr_(ptr), has_default_(false) {}

    /// \brief Construct an option that can store a single value with a default
    /// value
    template <typename V>
    ProgramOption (const char *name, const char *desc, T *ptr, const V &val) :
        name_(std::string("--") + name),
        desc_(desc), ptr_(ptr), vec_ptr_(VSMC_NULLPTR),
        default_(static_cast<T>(val)), has_default_(true) {*ptr = default_;}

    /// \brief Construct an option that can store multiple values with a
    /// default value
    template <typename V>
    ProgramOption (const char *name, const char *desc, std::vector<T> *ptr,
            const V &val) :
        name_(std::string("--") + name),
        desc_(desc), ptr_(VSMC_NULLPTR), vec_ptr_(ptr),
        default_(static_cast<T>(val)), has_default_(true)
    {
        vec_ptr_->clear();
        vec_ptr_->push_back(default_);
    }

    /// \brief Compare an option name and set value if it is the same as the
    /// name of this option
    ///
    /// \param name Name of the specified opiton to be checked against this
    /// option
    /// \param sval A string contains the value, will be converted to this
    /// option's type
    bool compare_set (const char *name, const char *sval)
    {
        if (std::strcmp(name_.c_str(), name))
            return false;

        std::stringstream ss;
        ss << sval;
        T tval;
        ss >> tval;
        if (ss.fail()) {
            std::fprintf(stderr, "Invalid value for option %s: %s\n",
                    name_.c_str(), sval);
            return false;
        }

        if (ptr_) *ptr_ = tval;
        if (vec_ptr_) vec_ptr_->push_back(tval);

        return true;
    }

    /// \brief Print help information
    void print_help () const
    {
        std::cout << "  " << std::setw(20) << std::left << name_ << desc_;
        if (has_default_)
            std::cout << " (default: " << default_ << ")";
        std::cout << std::endl;
    }

    private :

    std::string name_;
    std::string desc_;
    T *const ptr_;
    std::vector<T> *const vec_ptr_;
    T default_;
    bool has_default_;
};

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
///     if (Map.process(argc, argv)) {
///         Map.print_help();
///         return 0;
///     }
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
/// If add an option with a name that already exists, then it is a runtime
/// error.
class ProgramOptionMap
{
    public :

    ~ProgramOptionMap ()
    {
        for (std::size_t i = 0; i != options_.size(); ++i)
            delete options_[i];
    }

    /// \brief Add an option
    ///
    /// \param name Name of the option, on command name it shall be specified
    /// by --name
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    template <typename T, typename Dest>
    ProgramOptionMap &add (const char *name, const char *desc, Dest *ptr)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_DUPLICATE(option_names_, name);
        option_names_.insert(name);
        options_.push_back(new ProgramOption<T>(name, desc, ptr));

        return *this;
    }

    /// \brief Add an option with a default value
    template <typename T, typename Dest, typename V>
    ProgramOptionMap &add (const char *name, const char *desc, Dest *ptr,
            const V &val)
    {
        VSMC_RUNTIME_ASSERT_UTILITY_PROGRAM_OPTION_DUPLICATE(option_names_, name);
        option_names_.insert(name);
        options_.push_back(new ProgramOption<T>(name, desc, ptr, val));

        return *this;
    }

    /// \brief Process the options
    ///
    /// \param argc The first argument of the `main` function
    /// \param argv The second argument of the `main` function
    ///
    /// \return `true` If the option `--help` has been specified on the command
    /// line. Otherwise `false`.
    bool process (int argc, const char *const *argv)
    {
        processed_.clear();
        for (int ac = 1; ac != argc; ++ac) {
            if (!std::strcmp(argv[ac], "--help")) {
                for (std::size_t i = 0; i != options_.size(); ++i)
                    options_[i]->print_help();
                return true;
            }
        }

        int ac = 1;
        while (ac < argc) {
            bool set = false;
            for (std::size_t i = 0; i != options_.size(); ++i) {
                set = options_[i]->compare_set(argv[ac], argv[ac + 1]);
                if (set) processed_.push_back(argv[ac]);
            }
            ac += set ? 2 : 1;
        }

        return false;
    }

    /// \brief Process the options
    bool process (int argc, char *const *argv)
    {
        const char **cargv = new const char *[argc];
        for (int i = 0; i != argc; ++i)
            cargv[i] = argv[i];

        return process(argc, cargv);
    }

    /// \brief Count the number of occurence of an option on the command line
    /// given its name
    std::size_t count (const char *name) const
    {
        std::string oname("--");
        oname += name;
        std::vector<std::string>::const_iterator iter = processed_.begin();
        std::size_t c = 0;
        for (; iter != processed_.end(); ++iter)
            if (*iter == oname)
                ++c;

        return c;
    }

    private :

    std::set<std::string> option_names_;
    std::vector<ProgramOptionBase *> options_;
    std::vector<std::string> processed_;
};

} // namespace vsmc

#endif // VSMC_UTILITY_PROGRAM_OPTION_HPP
