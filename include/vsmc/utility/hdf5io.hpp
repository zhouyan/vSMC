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

namespace vsmc
{

namespace internal
{

inline void hdf5io_matrix_dim(
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

template <typename T>
class HDF5LoadDataPtr
{
    public:
    HDF5LoadDataPtr() : ptr_(nullptr) {}

    template <typename OutputIter>
    void set(std::size_t n, OutputIter)
    {
        data_.resize(n);
    }

    void set(std::size_t, T *ptr) { ptr_ = ptr; }

    T *get() { return ptr_ == nullptr ? data_.data() : ptr_; }

    bool is_raw_ptr() const { return ptr_ == nullptr; }

    private:
    T *ptr_;
    Vector<T> data_;
}; // class HDF5LoadDataPtr

template <typename T>
class HDF5StoreDataPtr
{
    public:
    HDF5StoreDataPtr() : ptr_(nullptr) {}

    template <typename InputIter>
    InputIter set(std::size_t n, InputIter first)
    {
        data_.resize(n);
        T *dst = data_.data();
        for (std::size_t i = 0; i != n; ++i, ++first)
            dst[i] = *first;

        return first;
    }

    T *set(std::size_t n, T *ptr)
    {
        ptr_ = ptr;
        return ptr + n;
    }

    const T *set(std::size_t n, const T *ptr)
    {
        ptr_ = ptr;
        return ptr + n;
    }

    const T *get() const { return ptr_ == nullptr ? data_.data() : ptr_; }

    bool is_raw_ptr() const { return ptr_ == nullptr; }

    private:
    const T *ptr_;
    Vector<T> data_;
}; // class HDF5StoreDataPtr

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

/// \brief Get the number of bytes of the data in the HDF5 format
/// \ingroup HDF5IO
inline ::hsize_t hdf5size(
    const std::string &filename, const std::string &dataname)
{
    ::hid_t datafile =
        ::H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    ::hid_t dataset = ::H5Dopen(datafile, dataname.c_str(), H5P_DEFAULT);
    ::hid_t dataspace = ::H5Dget_space(dataset);
    ::hid_t datatype = ::H5Dget_type(dataset);
    std::size_t n =
        static_cast<std::size_t>(H5Sget_simple_extent_npoints(dataspace));
    std::size_t b = H5Tget_size(datatype);
    std::size_t bytes = n * b;

    ::H5Tclose(datatype);
    ::H5Sclose(dataspace);
    ::H5Dclose(dataset);
    ::H5Fclose(datafile);

    return bytes;
}

/// \brief Get the number of elements of the data in the HDF5 format given type
/// \ingroup HDF5IO
template <typename T>
inline ::hsize_t hdf5size(
    const std::string &filename, const std::string &dataname)
{
    return hdf5size(filename, dataname) / sizeof(T);
}

/// \brief Load raw data in the HDF5 format
/// \ingroup HDF5IO
template <typename T, typename OutputIter>
inline OutputIter hdf5load(
    const std::string &filename, const std::string &dataname, OutputIter first)
{
    std::size_t n = hdf5size(filename, dataname) / sizeof(T);
    internal::HDF5LoadDataPtr<T> data_ptr;
    data_ptr.set(n, first);
    T *data = data_ptr.get();

    ::hid_t datafile =
        ::H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    ::hid_t dataset = ::H5Dopen(datafile, dataname.c_str(), H5P_DEFAULT);
    ::hid_t datatype = ::H5Dget_type(dataset);
    ::H5Dread(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
    if (data_ptr.is_raw_ptr()) {
        first += n;
    } else {
        for (std::size_t i = 0; i != n; ++i, ++first)
            *first = data[i];
    }

    ::H5Tclose(datatype);
    ::H5Dclose(dataset);
    ::H5Fclose(datafile);

    return first;
}

/// \brief Create a new HDF5 file for store data
/// \ingroup HDF5IO
inline void hdf5store_new(const std::string &filename)
{
    ::hid_t datafile =
        ::H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    ::H5Fclose(datafile);
}

/// \brief Store a matrix in the HDF5 format from an input iterator
/// \ingroup HDF5IO
///
/// \tparam T Type of the data
/// \param layout Storage layout (RowMajor or ColMajor)
/// \param nrow Number of rows
/// \param ncol Number of columns
/// \param filename Name of the HDF5 file
/// \param dataname Name of the matrix data
/// \param first An input iterator to sequence of length nrow * ncol
/// \param append If true the data is appended into an existing file,
/// otherwise
/// save in a new file
///
/// \note
/// HDF5 store data in row major layout. For example,
/// ~~~{.cpp}
/// double data[6] = {1, 2, 3, 4, 5, 6};
/// // Store matrix
/// // 1 2
/// // 3 4
/// // 5 6
/// hdf5store_matrix<double>(RowMajor, 3, 2, "row.hdf5", "data", data);
/// // Store matrix
/// // 1 4
/// // 2 5
/// // 3 6
/// hdf5store_matrix<double>(ColMajor, 3, 2, "col.hdf5", "data", data);
/// ~~~
/// When the results are read by other program, for example R, a transpose may
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
/// That is, when the data is stored in column major layout in C++ memory, then
/// the read in R produces exactly the same output. If the data is stored as
/// row major matrix in C++ memory, the read in R produces the transpose the
/// original matrix though they are identical in memory.
template <typename T, typename InputIter>
inline void hdf5store_matrix(MatrixLayout layout, std::size_t nrow,
    std::size_t ncol, const std::string &filename, const std::string &dataname,
    InputIter first, bool append)
{
    if (nrow == 0 || ncol == 0)
        return;

    std::string dataset_name("/" + dataname);
    ::hsize_t dim[2];
    internal::hdf5io_matrix_dim(layout, nrow, ncol, dim);
    internal::HDF5StoreDataPtr<T> data_ptr;
    data_ptr.set(nrow * ncol, first);
    const T *data = data_ptr.get();

    ::hid_t datafile = append ?
        ::H5Fopen(filename.c_str(), H5F_ACC_RDWR, H5P_DEFAULT) :
        ::H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    ::hid_t dataspace = ::H5Screate_simple(2, dim, nullptr);
    ::hid_t datatype = hdf5io_datatype<T>();
    ::hid_t dataset = ::H5Dcreate(datafile, dataset_name.c_str(), datatype,
        dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    ::H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

    ::H5Dclose(dataset);
    ::H5Tclose(datatype);
    ::H5Sclose(dataspace);
    ::H5Fclose(datafile);
}

/// \brief Create an empty list
/// \ingroup HDF5IO
///
/// \param filename Name of the HDF5 file
/// \param dataname Name of the list
/// \param append If true the data is appended into an existing file,
/// otherwise
/// save in a new file
inline void hdf5store_list_empty(
    const std::string &filename, const std::string &dataname, bool append)
{
    std::string group_name("/" + dataname);

    ::hid_t datafile;
    if (append) {
        datafile = ::H5Fopen(filename.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    } else {
        datafile = ::H5Fcreate(
            filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    }
    ::hid_t datagroup = ::H5Gcreate(
        datafile, group_name.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    ::H5Gclose(datagroup);
    ::H5Fclose(datafile);
}

/// \brief Store a list in the HDF5 format from an iterator to iterators
/// \ingroup HDF5IO
///
/// \tparam T Type of the data
/// \tparam InputIterIter The input iterator type, which points to input
/// iterators
/// \tparam SInputIter The input iterator type of names
/// \param nrow Number of elements in each element of the list
/// \param ncol Number of elements in the list
/// \param filename Name of the HDF5 file
/// \param dataname Name of the list
/// \param first An iterator points to a sequence of iterators of length ncol.
/// Each derefence of an iterator is iteself an iterator that points to the
/// beginning of an element of the list.
/// \param sfirst An iterator points to the beginning of a sequence of strings
/// that store the names of each column. The dereference need to be
/// convertible
/// to std::string
/// \param append If true the data is appended into an existing file,
/// otherwise
/// save in a new file
///
/// \note
/// Each element in the list is assumed to be a vector of the same length. If
/// the list contains elements of different length, then create an empty list,
/// which is exactly an empty group in HDF5 terminology and use
/// `hdf5store_list_insert` to insert each element.
template <typename T, typename InputIterIter, typename SInputIter>
inline void hdf5store_list(std::size_t nrow, std::size_t ncol,
    const std::string &filename, const std::string &dataname,
    InputIterIter first, SInputIter sfirst, bool append)
{
    std::string group_name("/" + dataname);
    ::hsize_t dim[1] = {nrow};

    ::hid_t datafile = append ?
        ::H5Fopen(filename.c_str(), H5F_ACC_RDWR, H5P_DEFAULT) :
        ::H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    ::hid_t datagroup = ::H5Gcreate(
        datafile, group_name.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    if (nrow != 0 && ncol != 0) {
        ::hid_t dataspace = ::H5Screate_simple(1, dim, nullptr);
        ::hid_t datatype = hdf5io_datatype<T>();
        internal::HDF5StoreDataPtr<T> data_ptr;
        for (std::size_t j = 0; j != ncol; ++j, ++first, ++sfirst) {
            data_ptr.set(nrow, *first);
            const T *data = data_ptr.get();
            std::string dataset_name(group_name + "/" + (*sfirst));
            ::hid_t dataset = ::H5Dcreate(datafile, dataset_name.c_str(),
                datatype, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            ::H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
            ::H5Dclose(dataset);
        }
        ::H5Tclose(datatype);
        ::H5Sclose(dataspace);
    }

    ::H5Gclose(datagroup);
    ::H5Fclose(datafile);
}

/// \brief Insert a variable into an existing list saved in HDF5 format
/// \ingroup HDF5IO
///
/// \param N The length of the variable vector. It may be different from that
/// of the existing list.
/// \param filename Name of the HDF5 file
/// \param dataname Name of the list
/// \param first An iterator points to the beginning of the variable vector
/// \param vname Name of the new variable
template <typename T, typename InputIter>
inline void hdf5store_list_insert(std::size_t N, const std::string &filename,
    const std::string &dataname, InputIter first, const std::string &vname)
{
    if (N == 0)
        return;

    std::string dataset_name("/" + dataname + "/" + vname);
    ::hsize_t dim[1] = {N};

    ::hid_t datafile = ::H5Fopen(filename.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    ::hid_t dataspace = ::H5Screate_simple(1, dim, nullptr);
    ::hid_t datatype = hdf5io_datatype<T>();
    internal::HDF5StoreDataPtr<T> data_ptr;
    data_ptr.set(N, first);
    const T *data = data_ptr.get();
    ::hid_t dataset = ::H5Dcreate(datafile, dataset_name.c_str(), datatype,
        dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    ::H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

    ::H5Dclose(dataset);
    ::H5Tclose(datatype);
    ::H5Sclose(dataspace);
    ::H5Fclose(datafile);
}

namespace internal
{

template <typename InputIter, typename... InputIters>
inline void hdf5store_list_insert_tuple(std::size_t nrow,
    const std::string &filename, const std::string &dataname,
    const std::tuple<InputIter, InputIters...> &first, const std::string *sptr,
    std::integral_constant<std::size_t, 0>)
{
    using value_type =
        typename std::iterator_traits<typename std::tuple_element<0,
            std::tuple<InputIter, InputIters...>>::type>::value_type;
    HDF5StoreDataPtr<value_type> data_ptr;
    data_ptr.set(nrow, std::get<0>(first));
    const value_type *data = data_ptr.get();
    hdf5store_list_insert<value_type>(nrow, filename, dataname, data, *sptr);
}

template <typename InputIter, typename... InputIters, std::size_t Pos>
inline void hdf5store_list_insert_tuple(std::size_t nrow,
    const std::string &filename, const std::string &dataname,
    const std::tuple<InputIter, InputIters...> &first, const std::string *sptr,
    std::integral_constant<std::size_t, Pos>)
{
    using value_type =
        typename std::iterator_traits<typename std::tuple_element<Pos,
            std::tuple<InputIter, InputIters...>>::type>::value_type;
    HDF5StoreDataPtr<value_type> data_ptr;
    data_ptr.set(nrow, std::get<Pos>(first));
    const value_type *data = data_ptr.get();
    hdf5store_list_insert<value_type>(nrow, filename, dataname, data, *sptr);
    hdf5store_list_insert_tuple(nrow, filename, dataname, first, --sptr,
        std::integral_constant<std::size_t, Pos - 1>());
}

} // namespace vsmc::internal

/// \brief Store a list in the HDF5 format from tuple of iterators
/// \ingroup HDF5IO
///
/// \details
/// A list is similar to that in R. It is much like a matrix except that
/// each column will be stored seperatedly as an variable and given a name.
///
/// \param nrow Number of rows
/// \param filename Name of the HDF5 file
/// \param dataname Name of the list
/// \param first A `std::tuple` type object whose element types are iterators.
/// Each element is the beginning of an element of the list.
/// \param sfirst An iterator points to the beginning of a sequence of strings
/// that store the names of each column. The dereference need to be
/// convertible
/// to std::string
/// \param append If true the data is appended into an existing file,
/// otherwise
/// save in a new file
template <typename SInputIter, typename InputIter, typename... InputIters>
inline void hdf5store_list(std::size_t nrow, const std::string &filename,
    const std::string &dataname,
    const std::tuple<InputIter, InputIters...> &first, SInputIter sfirst,
    bool append)
{
    static constexpr std::size_t dim = sizeof...(InputIters) + 1;
    internal::HDF5StoreDataPtr<std::string> vnames;
    vnames.set(dim, sfirst);
    const std::string *sptr = vnames.get() + dim;
    hdf5store_list_empty(filename, dataname, append);
    internal::hdf5store_list_insert_tuple(nrow, filename, dataname, first,
        --sptr, std::integral_constant<std::size_t, dim - 1>());
}

namespace internal
{

template <typename IntType>
inline bool hdf5store_int(std::size_t n, IntType *r, std::false_type)
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
inline bool hdf5store_int(std::size_t n, IntType *r, std::true_type)
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

} // namespace vsmc::internal

/// \brief Store a StateMatrix in the HDF5 format
/// \ingroup HDF5IO
template <MatrixLayout Layout, std::size_t Dim, typename T>
inline void hdf5store(const StateMatrix<Layout, Dim, T> &state_matrix,
    const std::string &filename, const std::string &dataname, bool append)
{
    hdf5store_matrix<T>(Layout, state_matrix.size(), state_matrix.dim(),
        filename, dataname, state_matrix.data(), append);
}

/// \brief Store a Particle in the HDF5 format
/// \ingroup HDF5IO
template <typename T>
inline void hdf5store(const Particle<T> &particle, const std::string &filename,
    const std::string &dataname, bool append)
{
    hdf5store_list_empty(filename, dataname, append);
    hdf5store(particle.value(), filename, dataname + "/value", true);
    hdf5store_matrix<double>(ColMajor,
        static_cast<std::size_t>(particle.size()), 1, filename,
        dataname + "/weight", particle.weight().data(), true);
}

/// \brief Store a Monitor in the HDF5 format
/// \ingroup HDF5IO
template <typename T>
inline void hdf5store(const Monitor<T> &monitor, const std::string &filename,
    const std::string &dataname, bool append)
{
    std::size_t nrow = monitor.iter_size();
    std::size_t ncol = monitor.dim();

    hdf5store_list_empty(filename, dataname, append);
    Vector<std::size_t> index(nrow);
    monitor.read_index(index.data());
    bool use_int = internal::hdf5store_int(
        index.size(), index.data(), std::is_signed<std::size_t>());
    if (use_int) {
        Vector<int> index_small(index.size());
        std::copy(index.begin(), index.end(), index_small.begin());
        hdf5store_list_insert<int>(
            nrow, filename, dataname, index_small.data(), "Index");
    } else {
        hdf5store_list_insert<std::size_t>(
            nrow, filename, dataname, index.data(), "Index");
    }

    std::string record_name = dataname + "/Record";
    hdf5store_list_empty(filename, record_name, true);
    Vector<double> record(nrow * ncol);
    monitor.read_record_matrix(ColMajor, record.data());
    const double *record_ptr = record.data();
    std::stringstream ss;
    for (std::size_t j = 0; j != ncol; ++j, record_ptr += nrow) {
        std::string vname = monitor.name(j);
        if (vname.empty())
            vname = "X." + internal::itos(j);
        hdf5store_list_insert<double>(
            nrow, filename, record_name, record_ptr, vname);
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

    hdf5store_list_empty(filename, dataname, append);

    std::size_t ncol_int = sampler.summary_header_size_int();
    Vector<std::string> header_int(ncol_int);
    Vector<size_type> data_int(nrow * ncol_int);
    sampler.summary_header_int(header_int.begin());
    sampler.template summary_data_int(ColMajor, data_int.begin());
    bool use_int = internal::hdf5store_int(
        data_int.size(), data_int.data(), std::is_signed<size_type>());
    if (use_int) {
        Vector<int> data_int_small(data_int.size());
        std::copy(data_int.begin(), data_int.end(), data_int_small.begin());
        const int *data_ptr_int = data_int_small.data();
        for (std::size_t j = 0; j != ncol_int; ++j, data_ptr_int += nrow) {
            hdf5store_list_insert<int>(
                nrow, filename, dataname, data_ptr_int, header_int[j]);
        }
    } else {
        const size_type *data_ptr_int = data_int.data();
        for (std::size_t j = 0; j != ncol_int; ++j, data_ptr_int += nrow) {
            hdf5store_list_insert<size_type>(
                nrow, filename, dataname, data_ptr_int, header_int[j]);
        }
    }

    std::size_t ncol = sampler.summary_header_size();
    Vector<std::string> header(ncol);
    Vector<double> data(nrow * ncol);
    sampler.summary_header(header.begin());
    sampler.template summary_data(ColMajor, data.begin());
    const double *data_ptr = data.data();
    for (std::size_t j = 0; j != ncol; ++j, data_ptr += nrow) {
        hdf5store_list_insert<double>(
            nrow, filename, dataname, data_ptr, header[j]);
    }
}

} // namespace vsmc

#endif // VSMC_UTILITY_HDF5IO_HPP
