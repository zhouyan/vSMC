#ifndef VSMC_UTILITY_PROGRAM_OPTION_HPP
#define VSMC_UTILITY_PROGRAM_OPTION_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief Program opiton base class
/// \ingroup Option
class ProgramOptionBase
{
    public :

    virtual bool set (const std::string &) = 0;
    virtual void print_help () const = 0;
    virtual ProgramOptionBase *clone () const = 0;
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
    /// \param oname Name of the option in the format `--name`
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    ProgramOption (const std::string &oname, const std::string &desc, T *ptr) :
        oname_(oname), desc_(desc), ptr_(ptr), vec_ptr_(VSMC_NULLPTR),
        has_default_(false) {}

    /// \brief Construct an option that can store multiple values
    ///
    /// \param oname Name of the option in the format `--name`. The option can
    /// be specified multiple times on the command line. Each value will be
    /// stored within the vector
    /// \param desc A descritpion stream of the option
    /// \param ptr The destination that store the option value
    ProgramOption (const std::string &oname, const std::string &desc,
            std::vector<T> *ptr) :
        oname_(oname), desc_(desc), ptr_(VSMC_NULLPTR), vec_ptr_(ptr),
        has_default_(false) {}

    /// \brief Construct an option that can store a single value with a default
    /// value
    template <typename V>
    ProgramOption (const std::string &oname, const std::string &desc,
            T *ptr, const V &val) :
        oname_(oname), desc_(desc), ptr_(ptr), vec_ptr_(VSMC_NULLPTR),
        default_(static_cast<T>(val)), has_default_(true) {*ptr = default_;}

    /// \brief Construct an option that can store multiple values with a
    /// default value
    template <typename V>
    ProgramOption (const std::string &oname, const std::string &desc,
            std::vector<T> *ptr, const V &val) :
        oname_(oname), desc_(desc), ptr_(VSMC_NULLPTR), vec_ptr_(ptr),
        default_(static_cast<T>(val)), has_default_(true)
    {vec_ptr_->push_back(default_);}

    bool set (const std::string &sval)
    {
        std::stringstream ss;
        ss << sval;
        T tval;
        ss >> tval;
        if (ss.fail()) {
            std::fprintf(stderr, "Invalid value for option %s: %s\n",
                    oname_.c_str(), sval.c_str());
            return false;
        }

        if (ptr_) *ptr_ = tval;
        if (vec_ptr_) vec_ptr_->push_back(tval);

        return true;
    }

    /// \brief Print help information
    void print_help () const
    {
        std::cout << "  " << std::setw(20) << std::left << oname_ << desc_;
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

    std::string oname_;
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

    ProgramOptionMap () {}

    ProgramOptionMap (const ProgramOptionMap &other) :
        option_ptr_(other.option_ptr_), option_cnt_(other.option_cnt_)
    {
        for (std::map<std::string, ProgramOptionBase *>::iterator
                iter = option_ptr_.begin();
                iter != option_ptr_.end(); ++iter) {
            if (iter->second)
                iter->second = iter->second->clone();
        }
    }

    ProgramOptionMap &operator= (const ProgramOptionMap &other)
    {
        if (this != &other) {
            for (std::map<std::string, ProgramOptionBase *>::iterator
                    iter = option_ptr_.begin();
                    iter != option_ptr_.end(); ++iter) {
                if (iter->second)
                    delete iter->second;
            }

            option_ptr_ = other.option_ptr_;
            option_cnt_ = other.option_cnt_;

            for (std::map<std::string, ProgramOptionBase *>::iterator
                    iter = option_ptr_.begin();
                    iter != option_ptr_.end(); ++iter) {
                if (iter->second)
                    iter->second = iter->second->clone();
            }
        }

        return *this;
    }

    ~ProgramOptionMap ()
    {
        for (std::map<std::string, ProgramOptionBase *>::iterator
                iter = option_ptr_.begin();
                iter != option_ptr_.end(); ++iter) {
            if (iter->second)
                delete iter->second;
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
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOption<T>(oname, desc, ptr);
        if (option_ptr_.count(oname) != 0)
            if (option_ptr_[oname])
                delete option_ptr_[oname];
        option_ptr_[oname] = optr;
        option_cnt_[oname] = 0;

        return *this;
    }

    /// \brief Add an option with a default value
    template <typename T, typename Dest, typename V>
    ProgramOptionMap &add (const std::string &name, const std::string &desc,
            Dest *ptr, const V &val)
    {
        const std::string oname("--" + name);
        ProgramOptionBase *optr = new ProgramOption<T>(oname, desc, ptr, val);
        if (option_ptr_.count(oname) != 0)
            if (option_ptr_[oname])
                delete option_ptr_[oname];
        option_ptr_[oname] = optr;
        option_cnt_[oname] = 0;

        return *this;
    }

    ProgramOptionMap &remove (const std::string &name)
    {
        const std::string oname("--" + name);
        if (option_ptr_.count(oname) != 0) {
            if (option_ptr_[oname])
                delete option_ptr_[oname];
            option_ptr_.erase(oname);
            option_cnt_.erase(oname);
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
                for (std::map<std::string, ProgramOptionBase *>::iterator
                        iter = option_ptr_.begin();
                        iter != option_ptr_.end(); ++iter) {
                    iter->second->print_help();
                }
                return true;
            }
        }

        for (int ac = 1; ac < argc - 1; ++ac) {
            if (process_pair(argv[ac], argv[ac + 1]))
                ++ac;
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
    {return option_cnt_.find("--" + name)->second;}

    private :

    std::map<std::string, ProgramOptionBase *> option_ptr_;
    std::map<std::string, std::size_t> option_cnt_;

    bool process_pair (const std::string &oname, const std::string &sval)
    {
        std::map<std::string, ProgramOptionBase *>::iterator iter =
            option_ptr_.find(oname);

        if (iter == option_ptr_.end())
            return false;

        if (!iter->second->set(sval))
            return false;

        ++option_cnt_[oname];
        return true;
    }
};

} // namespace vsmc

#endif // VSMC_UTILITY_PROGRAM_OPTION_HPP
