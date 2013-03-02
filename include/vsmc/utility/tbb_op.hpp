#ifndef VSMC_UTILITY_TBB_OP_HPP
#define VSMC_UTILITY_TBB_OP_HPP

#include <vsmc/internal/common.hpp>

#define VSMC_DEFINE_TBB_OP_FOR_UNARY_OBJECT(name, uni)                        \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data, T *result) : data_(data), result_(result) {}         \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range) const                      \
    {                                                                         \
        const T *const data = data_;                                          \
        T *const result = result_;                                            \
        uni op;                                                               \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result[i] = op(data[i]);                                          \
    }                                                                         \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T *const result_;                                                         \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_FOR_BINARY_OBJECT(name, bin)                       \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data, T *result, const T &val) :                           \
        data_(data), result_(result), val_(val) {}                            \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range) const                      \
    {                                                                         \
        const T *const data = data_;                                          \
        T *const result = result_;                                            \
        bin op;                                                               \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result[i] = op(data[i], val_);                                    \
    }                                                                         \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T *const result_;                                                         \
    const T val_;                                                             \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(name, uni)                      \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data, T *result) : data_(data), result_(result) {}         \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range) const                      \
    {                                                                         \
        const T *const data = data_;                                          \
        T *const result = result_;                                            \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result[i] = uni(data[i]);                                         \
    }                                                                         \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T *const result_;                                                         \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_FOR_BINARY_FUNCTION(name, bin)                     \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data, T *result, const T &val) :                           \
        data_(data), result_(result), val_(val) {}                            \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range) const                      \
    {                                                                         \
        const T *const data = data_;                                          \
        T *const result = result_;                                            \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result[i] = bin(data[i], val_);                                   \
    }                                                                         \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T *const result_;                                                         \
    const T val_;                                                             \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_FOR_UNARY_OPERATOR(name, uni)                      \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data, T *result) : data_(data), result_(result) {}         \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range) const                      \
    {                                                                         \
        const T *const data = data_;                                          \
        T *const result = result_;                                            \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result[i] = uni data[i];                                          \
    }                                                                         \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T *const result_;                                                         \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(name, bin)                     \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data, T *result, const T &val) :                           \
        data_(data), result_(result), val_(val) {}                            \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range) const                      \
    {                                                                         \
        const T *const data = data_;                                          \
        T *const result = result_;                                            \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result[i] = data[i] bin val_;                                     \
    }                                                                         \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T *const result_;                                                         \
    const T val_;                                                             \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OBJECT(name, bin, identity)          \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data) : data_(data), result_(identity) {}                  \
                                                                              \
    template <typename Split>                                                 \
    name (const name<T> &other, Split) :                                      \
        data_(other.data_), result_(identity) {}                              \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range)                            \
    {                                                                         \
        const T *const data = data_;                                          \
        T result = result_;                                                   \
        bin op;                                                               \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result = op(result, data[i]);                                     \
        result_ = result;                                                     \
    }                                                                         \
                                                                              \
    void join (const name<T> &other)                                          \
    {                                                                         \
        bin op;                                                               \
        result_ = op(result_, other.result_);                                 \
    }                                                                         \
                                                                              \
    T result () const {return result_;}                                       \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T result_;                                                                \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_REDUCE_BINARY_FUNCTION(name, bin, identity)        \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data) : data_(data), result_(identity) {}                  \
                                                                              \
    template <typename Split>                                                 \
    name (const name<T> &other, Split) :                                      \
        data_(other.data_), result_(identity) {}                              \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range)                            \
    {                                                                         \
        const T *const data = data_;                                          \
        T result = result_;                                                   \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result = bin(result, data[i]);                                    \
        result_ = result;                                                     \
    }                                                                         \
                                                                              \
    void join (const name<T> &other) {result_ = bin(result_, other.result_);} \
                                                                              \
    T result () const {return result_;}                                       \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T result_;                                                                \
};                                                                            \
} }

#define VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OPERATOR(name, bin, identity)        \
namespace vsmc { namespace tbb_op {                                           \
template <typename T>                                                         \
class name                                                                    \
{                                                                             \
    public :                                                                  \
                                                                              \
    name (const T *data) : data_(data), result_(identity) {}                  \
                                                                              \
    template <typename Split>                                                 \
    name (const name<T> &other, Split) :                                      \
        data_(other.data_), result_(identity) {}                              \
                                                                              \
    template <typename SizeType, template <typename> class Range>             \
    void operator() (const Range<SizeType> &range)                            \
    {                                                                         \
        const T *const data = data_;                                          \
        T result = result_;                                                   \
        for (SizeType i = range.begin(); i != range.end(); ++i)               \
            result = result bin data[i];                                      \
        result_ = result;                                                     \
    }                                                                         \
                                                                              \
    void join (const name<T> &other) {result_ = result_ bin other.result_;}   \
                                                                              \
    T result () const {return result_;}                                       \
                                                                              \
    private :                                                                 \
                                                                              \
    const T *const data_;                                                     \
    T result_;                                                                \
};                                                                            \
} }

namespace vsmc { namespace tbb_op {

template <typename T> inline const T &max_fn (const T &a, const T &b)
{return std::max VSMC_MACRO_NO_EXPANSION (a, b);}

template <typename T> inline const T &min_fn (const T &a, const T &b)
{return std::min VSMC_MACRO_NO_EXPANSION (a, b);}

template <typename T> struct positive_infinity_trait
{
    static VSMC_CONSTEXPR T value ()
    {return std::numeric_limits<T>::infinity();}
};

template <typename T> struct negative_infinity_trait
{
    static VSMC_CONSTEXPR T value ()
    {return -std::numeric_limits<T>::infinity();}
};

template <typename T> struct zero_trait
{static VSMC_CONSTEXPR T value () {return static_cast<T>(0);}};

template <typename T> struct one_trait
{static VSMC_CONSTEXPR T value () {return static_cast<T>(1);}};

template <typename T>
class square_sum
{
    public :

    square_sum (const T *data) :
        data_(data), result_(zero_trait<T>::value()) {}

    template <typename Split>
    square_sum (const square_sum<T> &other, Split) :
        data_(other.data_), result_(zero_trait<T>::value()) {}

    template <typename SizeType, template <typename> class Range>
    void operator() (const Range<SizeType> &range)
    {
        const T *const data = data_;
        T result = result_;
        for (SizeType i = range.begin(); i != range.end(); ++i)
            result = result + data[i] * data[i];
        result_ = result;
    }

    void join (const square_sum<T> &other) {result_ = result_ + other.result_;}

    T result () const {return result_;}

    private :

    const T *const data_;
    T result_;
}; // class square_sum

} } // namespace vsmc::tbb_op

VSMC_DEFINE_TBB_OP_FOR_UNARY_OPERATOR(negate, -)

VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(plus,       +)
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(minus,      -)
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(multiplies, *)
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(divides,    /)
VSMC_DEFINE_TBB_OP_FOR_BINARY_OPERATOR(modulus,    %)

VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(abs,   std::abs)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(exp,   std::exp)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(log,   std::log)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(log10, std::log10)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(sqrt,  std::sqrt)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(sin,   std::sin)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(cos,   std::cos)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(tan,   std::tan)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(asin,  std::asin)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(acos,  std::acos)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(atan,  std::atan)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(sinh,  std::sinh)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(cosh,  std::cosh)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(tanh,  std::tanh)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(ceil,  std::ceil)
VSMC_DEFINE_TBB_OP_FOR_UNARY_FUNCTION(floor, std::floor)

VSMC_DEFINE_TBB_OP_REDUCE_BINARY_FUNCTION(maximum, max_fn,
        negative_infinity_trait<T>::value())
VSMC_DEFINE_TBB_OP_REDUCE_BINARY_FUNCTION(minimum, min_fn,
        positive_infinity_trait<T>::value())

VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OPERATOR(summation, +, zero_trait<T>::value())
VSMC_DEFINE_TBB_OP_REDUCE_BINARY_OPERATOR(product,   *, one_trait<T>::value())

#endif // VSMC_UTILITY_TBB_OP_HPP
