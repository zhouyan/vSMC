//============================================================================
// vSMC/include/vsmc/utility/hdf5io.hpp
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

#ifndef VSMC_UTILITY_HDF5IO_HPP
#define VSMC_UTILITY_HDF5IO_HPP

#include <vsmc/internal/common.hpp>
#include <hdf5.h>

#define VSMC_DEFINE_HDF5TYPE(CPPName, CName)                                  \
    class HDF5##CPPName : public HDF5ID<HDF5##CPPName>                        \
    {                                                                         \
        public:                                                               \
        HDF5##CPPName(::hid_t id) : HDF5ID<HDF5##CPPName>(id) {}              \
                                                                              \
        static void close(::hid_t id) { ::H5##CName##close(id); }             \
    };

namespace vsmc
{

namespace internal
{

template <typename Derived>
class HDF5ID
{
    public:
    HDF5ID(::hid_t id) : id_(id) {}

    ~HDF5ID()
    {
        if (good())
            Derived::close(id_);
    }

    ::hid_t id() const { return id_; }

    bool good() const { return id_ >= 0; }

    bool operator!() const { return !good(); }

    explicit operator bool() const { return good(); }

    private:
    ::hid_t id_;
}; // class HDFID

VSMC_DEFINE_HDF5TYPE(DataSet, D)
VSMC_DEFINE_HDF5TYPE(DataSpace, S)
VSMC_DEFINE_HDF5TYPE(DataType, T)
VSMC_DEFINE_HDF5TYPE(File, F)
VSMC_DEFINE_HDF5TYPE(Group, G)

template <typename T>
class HDF5StoreDataPtr
{
    public:
    template <typename InputIter>
    HDF5StoreDataPtr(std::size_t n, InputIter first) : ptr_(nullptr)
    {
        set(n, first, std::is_convertible<InputIter, const T *>());
    }

    const T *get() const { return ptr_ == nullptr ? data_.data() : ptr_; }

    private:
    Vector<T> data_;
    const T *ptr_;

    template <typename InputIter>
    void set(std::size_t, InputIter first, std::true_type)
    {
        ptr_ = static_cast<const T *>(first);
    }

    template <typename InputIter>
    void set(std::size_t n, InputIter first, std::false_type)
    {
        data_.resize(n);
        std::copy_n(first, n, data_.begin());
    }
}; // class HDF5StoreDataPtr

inline void hdf5io_dim(
    MatrixLayout layout, std::size_t nrow, std::size_t ncol, ::hsize_t *dim)
{
    if (layout == RowMajor) {
        dim[0] = nrow;
        dim[1] = ncol;
    }
    if (layout == ColMajor) {
        dim[0] = ncol;
        dim[1] = nrow;
    }
}

template <typename IntType>
inline bool hdf5io_use_int(std::size_t n, IntType *r, std::false_type)
{
    if (sizeof(int) > sizeof(IntType))
        return true;

    bool flag = true;
    for (std::size_t i = 0; i != n; ++i) {
        if (r[i] > std::numeric_limits<int>::max()) {
            flag = false;
            break;
        }
    }

    return flag;
}

template <typename IntType>
inline bool hdf5io_use_int(std::size_t n, IntType *r, std::true_type)
{
    if (sizeof(int) > sizeof(IntType))
        return true;

    bool flag = true;
    for (std::size_t i = 0; i != n; ++i) {
        if (r[i] < std::numeric_limits<int>::min()) {
            flag = false;
            break;
        }
        if (r[i] > std::numeric_limits<int>::max()) {
            flag = false;
            break;
        }
    }

    return flag;
}

inline ::hid_t hdf5io_datafile(
    const std::string &filename, bool append, bool read_only)
{
    if (!append)
        return ::H5Fcreate(
            filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    unsigned flag = 0;
    if (read_only)
        flag = H5F_ACC_RDONLY;
    else
        flag = H5F_ACC_RDWR;

    return ::H5Fopen(filename.c_str(), flag, H5P_DEFAULT);
}

} // namespace vsmc::internal

/// \brief HDF5 data type
/// \ingroup HDFIO
template <typename>
inline ::hid_t hdf5io_datatype()
{
    return -1;
}

/// \brief HDF5 data type specialization for char
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<char>()
{
    return ::H5Tcopy(H5T_NATIVE_CHAR);
}

/// \brief HDF5 data type specialization for signed char
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<signed char>()
{
    return ::H5Tcopy(H5T_NATIVE_SCHAR);
}

/// \brief HDF5 data type specialization for unsigned char
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<unsigned char>()
{
    return ::H5Tcopy(H5T_NATIVE_UCHAR);
}

/// \brief HDF5 data type specialization for short
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<short>()
{
    return ::H5Tcopy(H5T_NATIVE_SHORT);
}

/// \brief HDF5 data type specialization for unsigned short
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<unsigned short>()
{
    return ::H5Tcopy(H5T_NATIVE_UCHAR);
}

/// \brief HDF5 data type specialization for int
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<int>()
{
    return ::H5Tcopy(H5T_NATIVE_INT);
}

/// \brief HDF5 data type specialization for unsigned int
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<unsigned int>()
{
    return ::H5Tcopy(H5T_NATIVE_UINT);
}

/// \brief HDF5 data type specialization for long
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<long>()
{
    return ::H5Tcopy(H5T_NATIVE_LONG);
}

/// \brief HDF5 data type specialization for unsigned long
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<unsigned long>()
{
    return ::H5Tcopy(H5T_NATIVE_ULONG);
}

/// \brief HDF5 data type specialization for long long
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<long long>()
{
    return ::H5Tcopy(H5T_NATIVE_LLONG);
}

/// \brief HDF5 data type specialization for unsigned long
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<unsigned long long>()
{
    return ::H5Tcopy(H5T_NATIVE_ULLONG);
}

/// \brief HDF5 data type specialization for float
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<float>()
{
    return ::H5Tcopy(H5T_NATIVE_FLOAT);
}

/// \brief HDF5 data type specialization for double
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<double>()
{
    return ::H5Tcopy(H5T_NATIVE_DOUBLE);
}

/// \brief HDF5 data type specialization for long double
/// \ingroup HDF5IO
template <>
inline ::hid_t hdf5io_datatype<long double>()
{
    return ::H5Tcopy(H5T_NATIVE_LDOUBLE);
}

/// \brief The number of elements in HDF5 data
/// \ingroup HDF5IO
inline std::size_t hdf5load_size(
    const std::string &filename, const std::string &dataname)
{
    internal::HDF5File datafile(
        internal::hdf5io_datafile(filename, true, true));
    if (!datafile)
        return 0;

    internal::HDF5DataSet dataset(
        ::H5Dopen(datafile.id(), dataname.c_str(), H5P_DEFAULT));
    if (!dataset)
        return 0;

    internal::HDF5DataSpace dataspace(::H5Dget_space(dataset.id()));
    if (!dataspace)
        return 0;

    ::hssize_t n = ::H5Sget_simple_extent_npoints(dataspace.id());
    if (n < 0)
        return 0;

    return static_cast<std::size_t>(n);
}

/// \brief Load HDF5 data
/// \ingroup HDF5IO
template <typename OutputIter>
inline OutputIter hdf5load(
    const std::string &filename, const std::string &dataname, OutputIter first)
{
    internal::HDF5File datafile(
        internal::hdf5io_datafile(filename, true, true));
    if (!datafile)
        return first;

    internal::HDF5DataSet dataset(
        ::H5Dopen(datafile.id(), dataname.c_str(), H5P_DEFAULT));
    if (!dataset)
        return first;

    internal::HDF5DataSpace dataspace(::H5Dget_space(dataset.id()));
    if (!dataspace)
        return first;

    ::hssize_t n = ::H5Sget_simple_extent_npoints(dataspace.id());
    if (n < 0)
        return first;

    using T = typename std::iterator_traits<OutputIter>::value_type;
    std::size_t N = static_cast<std::size_t>(n);
    ::herr_t err = 0;

    internal::HDF5DataType src_datatype(::H5Dget_type(dataset.id()));
    if (!src_datatype)
        return first;

    internal::HDF5DataType dst_datatype(hdf5io_datatype<T>());
    if (!dst_datatype)
        return first;

    ::htri_t is_eq = ::H5Tequal(src_datatype.id(), dst_datatype.id());
    if (is_eq < 0)
        return first;

    std::size_t src_size = ::H5Tget_size(src_datatype.id());
    std::size_t dst_size = ::H5Tget_size(dst_datatype.id());
    std::size_t buf_size = std::max(src_size, dst_size);
    std::size_t buf_rate =
        buf_size / dst_size + (buf_size % dst_size == 0 ? 0 : 1);
    Vector<T> buf(N * buf_rate);
    err = ::H5Dread(dataset.id(), src_datatype.id(), H5S_ALL, H5S_ALL,
        H5P_DEFAULT, buf.data());
    if (err < 0)
        return first;

    if (is_eq == 0) {
        err = ::H5Tconvert(src_datatype.id(), dst_datatype.id(), N, buf.data(),
            nullptr, H5P_DEFAULT);
        if (err < 0)
            return first;
    }

    return std::copy_n(buf.begin(), N, first);
}

/// \brief Load HDF5 data
template <typename T>
inline Vector<T> hdf5load(
    const std::string &filename, const std::string &dataname)
{
    Vector<T> vector(hdf5load_size(filename, dataname));
    hdf5load(filename, dataname, vector.data());
}

/// \brief Create a new HDF5 file
/// \ingroup HDF5IO
inline void hdf5store(const std::string &filename)
{
    internal::HDF5File datafile(
        internal::hdf5io_datafile(filename, false, false));
}

/// \brief Create a new HDF5 group
/// \ingroup HDF5IO
inline void hdf5store(
    const std::string &filename, const std::string dataname, bool append)
{
    internal::HDF5File datafile(
        internal::hdf5io_datafile(filename, append, false));
    if (static_cast<bool>(datafile)) {
        internal::HDF5Group datagroup(::H5Gcreate2(datafile.id(),
            dataname.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT));
    }
}

/// \brief Store one dimensional vector
/// \ingroup HDF5IO
template <typename InputIter>
inline void hdf5store(std::size_t N, InputIter first,
    const std::string &filename, const std::string &dataname, bool append)
{
    using T = typename std::iterator_traits<InputIter>::value_type;

    if (N == 0)
        return;

    internal::HDF5File datafile(
        internal::hdf5io_datafile(filename, append, false));
    if (!datafile)
        return;

    internal::HDF5DataType datatype(hdf5io_datatype<T>());
    if (!datatype)
        return;

    ::hsize_t dim[1] = {N};
    internal::HDF5DataSpace dataspace(::H5Screate_simple(1, dim, nullptr));
    if (!dataspace)
        return;

    internal::HDF5DataSet dataset(::H5Dcreate2(datafile.id(), dataname.c_str(),
        datatype.id(), dataspace.id(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT));
    if (!dataset)
        return;

    internal::HDF5StoreDataPtr<T> dataptr(N, first);
    ::H5Dwrite(dataset.id(), datatype.id(), H5S_ALL, H5S_ALL, H5P_DEFAULT,
        dataptr.get());
}

/// \brief Store one dimensional vector
/// \ingroup HDF5IO
template <typename T, typename Alloc>
inline void hdf5store(const std::vector<T, Alloc> &vector,
    const std::string &filename, const std::string &dataname, bool append)
{
    hdf5store(vector.size(), vector.data(), filename, dataname, append);
}

/// \brief Store a matrix in the HDF5 format from an input iterator
/// \ingroup HDF5IO
///
/// \note
/// HDF5 store data in row major layout. For example,
/// ~~~{.cpp}
/// double data[6] = {1, 2, 3, 4, 5, 6};
///
/// // Store matrix
/// // 1 2
/// // 3 4
/// // 5 6
/// hdf5store_matrix<double>(RowMajor, 3, 2, data, "row.hdf5", "data");
///
/// // Store matrix
/// // 1 4
/// // 2 5
/// // 3 6
/// hdf5store_matrix<double>(ColMajor, 3, 2, data, "col.hdf5", "data");
/// ~~~
/// When the results are read by other program, for example R, a transpose
/// may
/// be needed. For instance,
/// ~~~{.r}
/// library(rhdf5)
/// row <- h5read("row.hdf5", "/data")
/// print(row) # produce
/// #      [,1] [,2] [,3]
/// # [1,]    1    3    5
/// # [2,]    2    4    6
/// col <- h5read("col.hdf5", "/data")
/// print(col)  # produce
/// #      [,1] [,2]
/// # [1,]    1    4
/// # [2,]    2    5
/// # [3,]    3    6
/// #
/// ~~~
/// That is, when the data is stored in column major layout in C++ memory,
/// then
/// the read in R produces exactly the same output. If the data is stored
/// as
/// row major matrix in C++ memory, the read in R produces the transpose
/// the
/// original matrix though they are identical in memory.
template <typename InputIter>
inline void hdf5store(MatrixLayout layout, std::size_t nrow, std::size_t ncol,
    InputIter first, const std::string &filename, const std::string &dataname,
    bool append)
{
    using T = typename std::iterator_traits<InputIter>::value_type;

    if (nrow == 0 || ncol == 0)
        return;

    internal::HDF5File datafile(
        internal::hdf5io_datafile(filename, append, false));
    if (!datafile)
        return;

    internal::HDF5DataType datatype(hdf5io_datatype<T>());
    if (!datatype)
        return;

    ::hsize_t dim[2];
    internal::hdf5io_dim(layout, nrow, ncol, dim);
    internal::HDF5DataSpace dataspace(::H5Screate_simple(2, dim, nullptr));
    if (!dataspace)
        return;

    internal::HDF5DataSet dataset(::H5Dcreate2(datafile.id(), dataname.c_str(),
        datatype.id(), dataspace.id(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT));
    if (!dataset)
        return;

    internal::HDF5StoreDataPtr<T> dataptr(nrow * ncol, first);
    ::H5Dwrite(dataset.id(), datatype.id(), H5S_ALL, H5S_ALL, H5P_DEFAULT,
        dataptr.get());
}

/// \brief Store a StateMatrix in the HDF5 format
/// \ingroup HDF5IO
template <MatrixLayout Layout, std::size_t Dim, typename T>
inline void hdf5store(const StateMatrix<Layout, Dim, T> &state_matrix,
    const std::string &filename, const std::string &dataname, bool append)
{
    hdf5store(Layout, state_matrix.size(), state_matrix.dim(),
        state_matrix.data(), filename, dataname, append);
}

/// \brief Store a Particle in the HDF5 format
/// \ingroup HDF5IO
template <typename T>
inline void hdf5store(const Particle<T> &particle, const std::string &filename,
    const std::string &dataname, bool append)
{
    hdf5store(filename, dataname, append);
    hdf5store(particle.value(), filename, dataname + "/Value", true);
    hdf5store(static_cast<std::size_t>(particle.weight().size()),
        particle.weight().data(), filename, dataname + "/Weight", true);
}

/// \brief Store a Monitor in the HDF5 format
/// \ingroup HDF5IO
template <typename T>
inline void hdf5store(const Monitor<T> &monitor, const std::string &filename,
    const std::string &dataname, bool append)
{
    std::size_t nrow = monitor.iter_size();
    std::size_t ncol = monitor.dim();

    hdf5store(filename, dataname, append);
    Vector<std::size_t> index(nrow);
    monitor.read_index(index.data());
    bool use_int = internal::hdf5io_use_int(
        index.size(), index.data(), std::is_signed<std::size_t>());
    if (use_int) {
        Vector<int> index_small(index.size());
        std::copy(index.begin(), index.end(), index_small.begin());
        hdf5store(
            nrow, index_small.data(), filename, dataname + "/Index", true);
    } else {
        hdf5store(nrow, index.data(), filename, dataname + "/Index", true);
    }

    hdf5store(filename, dataname + "/Record", true);
    Vector<double> record(nrow * ncol);
    monitor.read_record_matrix(ColMajor, record.data());
    const double *record_ptr = record.data();
    for (std::size_t j = 0; j != ncol; ++j, record_ptr += nrow) {
        std::string vname = monitor.name(j);
        if (vname.empty())
            vname = "X." + std::to_string(j);
        hdf5store(
            nrow, record_ptr, filename, dataname + "/Record/" + vname, true);
    }
}

/// \brief Store a Sampler in the HDF5 format
/// \ingroup HDF5IO
template <typename T>
inline void hdf5store(const Sampler<T> &sampler, const std::string &filename,
    const std::string &dataname, bool append)
{
    using size_type = typename Sampler<T>::size_type;

    std::size_t nrow = sampler.iter_size();

    if (nrow == 0)
        return;

    hdf5store(filename, dataname, append);

    std::size_t ncol_int = sampler.summary_header_size_int();
    Vector<std::string> header_int(ncol_int);
    Vector<size_type> data_int(nrow * ncol_int);
    sampler.summary_header_int(header_int.begin());
    sampler.summary_data_int(ColMajor, data_int.begin());
    bool use_int = internal::hdf5io_use_int(
        data_int.size(), data_int.data(), std::is_signed<size_type>());
    if (use_int) {
        Vector<int> data_int_small(data_int.size());
        std::copy(data_int.begin(), data_int.end(), data_int_small.begin());
        const int *ptr = data_int_small.data();
        for (std::size_t j = 0; j != ncol_int; ++j, ptr += nrow) {
            hdf5store(
                nrow, ptr, filename, dataname + "/" + header_int[j], true);
        }
    } else {
        const size_type *ptr = data_int.data();
        for (std::size_t j = 0; j != ncol_int; ++j, ptr += nrow) {
            hdf5store(
                nrow, ptr, filename, dataname + "/" + header_int[j], true);
        }
    }

    std::size_t ncol = sampler.summary_header_size();
    Vector<std::string> header(ncol);
    Vector<double> data(nrow * ncol);
    sampler.summary_header(header.begin());
    sampler.summary_data(ColMajor, data.begin());
    const double *ptr = data.data();
    for (std::size_t j = 0; j != ncol; ++j, ptr += nrow)
        hdf5store(nrow, ptr, filename, dataname + "/" + header[j], true);
}

} // namespace vsmc

#endif // VSMC_UTILITY_HDF5IO_HPP
