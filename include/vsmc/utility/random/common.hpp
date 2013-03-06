#ifndef VSMC_UTILITY_RANDOM_COMMON_HPP
#define VSMC_UTILITY_RANDOM_COMMON_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc { namespace internal {

template <typename CharT, typename Traits>
class SaveIOFlags
{
    public :

    explicit SaveIOFlags (std::basic_ios<CharT, Traits> &stream) :
        stream_(stream), flags_(stream_.flags()), fill_(stream_.fill()) {};

    ~SaveIOFlags ()
    {
        stream_.flags(flags_);
        stream_.fill(fill_);
    }

    private :

    std::basic_ios<CharT, Traits> &stream_;
    typename std::basic_ios<CharT, Traits>::fmtflags flags_;
    CharT fill_;

    SaveIOFlags (const SaveIOFlags<CharT, Traits> &);
    SaveIOFlags<CharT, Traits> &operator= (const SaveIOFlags<CharT, Traits> &);
}; // class SaveFlags

} } // namepsace vsmc::internal

#endif // VSMC_UTILITY_RANDOM_COMMON_HPP
