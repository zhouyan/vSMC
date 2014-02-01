#ifndef VSMC_UTILITY_HDF5_SAVE_HPP
#define VSMC_UTILITY_HDF5_SAVE_HPP

#include <vsmc/internal/common.hpp>
#include <hdf5.h>

namespace vsmc {

namespace internal {

template <MatrixOrder>
inline void hdf5_matrix_dim (std::size_t, std::size_t, hsize_t *);

template <>
inline void hdf5_matrix_dim<RowMajor> (std::size_t nrow, std::size_t ncol,
        hsize_t *dim)
{
    dim[0] = ncol;
    dim[1] = nrow;
}

template <>
inline void hdf5_matrix_dim<ColMajor> (std::size_t nrow, std::size_t ncol,
        hsize_t *dim)
{
    dim[1] = ncol;
    dim[0] = nrow;
}

template <typename T>
class HDF5DataPtr
{
    public :

    template <typename InputIter>
    InputIter set (std::size_t n, InputIter first)
    {
        data_.resize(n);
        T *dst = &data_[0];
        for (std::size_t i = 0; i != n; ++i, ++first, ++dst)
            *dst = *first;
        ptr_ = &data_[0];

        return first;
    }

    T *set (std::size_t n, T *ptr) {ptr_ = ptr; return ptr + n;}

    const T *set (std::size_t n, const T *ptr) {ptr_ = ptr; return ptr + n;}

    const T *get () const {return ptr_;}

    private :

    const T *ptr_;
    std::vector<T> data_;
};

} // namespace vsmc::internal

/// \brief HDF5 data type
/// \ingroup HDFSave
template <typename> inline hid_t hdf5_datatype () {return -1;}

/// \brief HDF5 data type specialization for char
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<char> ()
{return H5Tcopy(H5T_NATIVE_CHAR);}

/// \brief HDF5 data type specialization for signed char
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<signed char> ()
{return H5Tcopy(H5T_NATIVE_SCHAR);}

/// \brief HDF5 data type specialization for unsigned char
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<unsigned char> ()
{return H5Tcopy(H5T_NATIVE_UCHAR);}

/// \brief HDF5 data type specialization for short
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<short> ()
{return H5Tcopy(H5T_NATIVE_SHORT);}

/// \brief HDF5 data type specialization for unsigned short
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<unsigned short> ()
{return H5Tcopy(H5T_NATIVE_UCHAR);}

/// \brief HDF5 data type specialization for int
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<int> ()
{return H5Tcopy(H5T_NATIVE_INT);}

/// \brief HDF5 data type specialization for unsigned int
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<unsigned int> ()
{return H5Tcopy(H5T_NATIVE_UINT);}

/// \brief HDF5 data type specialization for long
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<long> ()
{return H5Tcopy(H5T_NATIVE_LONG);}

/// \brief HDF5 data type specialization for unsigned long
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<unsigned long> ()
{return H5Tcopy(H5T_NATIVE_ULONG);}

#if VSMC_HAS_LONG_LONG

/// \brief HDF5 data type specialization for long long
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<long long> ()
{return H5Tcopy(H5T_NATIVE_LLONG);}

/// \brief HDF5 data type specialization for unsigned long
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<unsigned long long> ()
{return H5Tcopy(H5T_NATIVE_ULLONG);}

#endif // VSMC_HAS_LONG_LONG

/// \brief HDF5 data type specialization for float
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<float> ()
{return H5Tcopy(H5T_NATIVE_FLOAT);}

/// \brief HDF5 data type specialization for double
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<double> ()
{return H5Tcopy(H5T_NATIVE_DOUBLE);}

/// \brief HDF5 data type specialization for long double
/// \ingroup HDFSave
template <>
inline hid_t hdf5_datatype<long double> ()
{return H5Tcopy(H5T_NATIVE_LDOUBLE);}

/// \brief Save a matrix in the HDF5 format from an input iterator
/// \ingroup HDF5Save
///
/// \details
/// \tparam Order Storage order (RowMajor or ColMajor)
/// \tparam T Tyep of the data
/// \param nrow Number of rows
/// \param ncol Number of columns
/// \param file_name Name of the HDF5 file
/// \param data_name Name of the matrix data
/// \param first An input iterator to sequence of length nrow * ncol
/// \param append If true the data is appended into an existing file, otherwise
/// save in a new file
template <MatrixOrder Order, typename T, typename InputIter>
inline InputIter hdf5_save_matrix (std::size_t nrow, std::size_t ncol,
        const std::string &file_name, const std::string &data_name,
        InputIter first, bool append = false)
{
    using std::advance;

    if (nrow == 0 || ncol == 0)
        return first;

    std::string dataset_name("/" + data_name);
    hsize_t dim[2];
    internal::hdf5_matrix_dim<Order>(nrow, ncol, dim);
    internal::HDF5DataPtr<T> data_ptr;
    InputIter last = data_ptr.set(nrow * ncol, first);
    const T *data = data_ptr.get();

    hid_t datafile;
    if (append) {
        datafile = H5Fopen(file_name.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    } else {
        datafile = H5Fcreate(file_name.c_str(),
                H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    }
    hid_t dataspace = H5Screate_simple(2, dim, NULL);
    hid_t datatype = hdf5_datatype<T>();
    hid_t dataset = H5Dcreate(datafile, dataset_name.c_str(),
            datatype, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

    H5Dclose(dataset);
    H5Tclose(datatype);
    H5Sclose(dataspace);
    H5Fclose(datafile);

    return last;
}

/// \brief Save a data frame in the HDF5 format from an iterator to iterators
/// \ingroup HDF5Save
///
/// \details
/// A data frame is similar to that in R. It is much like a matrix except that
/// each column will be stored seperatedly as an variable and given a name.
///
/// \tparam T Tyep of the data
/// \tparam InputIterIter The input iterator type, which points to input
/// iterators
/// \tparam SInputIter The input iterator type of names
/// \param nrow Number of rows
/// \param ncol Number of columns
/// \param file_name Name of the HDF5 file
/// \param data_name Name of the data frame
/// \param first An iterator points to a sequence of iterations of length ncol.
/// Each derefence of the iterator is iteself an iterator that points to the
/// beginning of a column of the data frame.
/// \param sfirst An iterator points to the beginning of a sequence of strings
/// that store the names of each column. The dereference need to be convertible
/// to std::string
/// \param append If true the data is appended into an existing file, otherwise
/// save in a new file
template <typename T, typename InputIterIter, typename SInputIter>
inline void hdf5_save_data_frame (std::size_t nrow, std::size_t ncol,
        const std::string &file_name, const std::string &data_name,
        InputIterIter first, SInputIter sfirst, bool append = false)
{
    std::string group_name("/" + data_name);
    hsize_t dim[1] = {nrow};

    hid_t datafile;
    if (append) {
        datafile = H5Fopen(file_name.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    } else {
        datafile = H5Fcreate(file_name.c_str(),
                H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    }
    hid_t datagroup = H5Gcreate(datafile, group_name.c_str(),
            H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    if (nrow != 0 && ncol != 0) {
        hid_t dataspace = H5Screate_simple(1, dim, NULL);
        hid_t datatype = hdf5_datatype<T>();
        internal::HDF5DataPtr<T> data_ptr;
        for (std::size_t j = 0; j != ncol; ++j, ++first, ++sfirst) {
            data_ptr.set(nrow, *first);
            const T *data = data_ptr.get();
            std::string dataset_name(group_name + "/" + (*sfirst));
            hid_t dataset = H5Dcreate(datafile, dataset_name.c_str(),
                    datatype, dataspace,
                    H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
            H5Dclose(dataset);
        }
        H5Tclose(datatype);
        H5Sclose(dataspace);
    }

    H5Gclose(datagroup);
    H5Fclose(datafile);
}

/// \brief Insert a variable into an existing data frame saved in HDF5 format
/// \ingroup HDF5Save
///
/// \details
/// \param N The length of the variable vector. It may be different from that
/// of the existing data frame.
/// \param file_name Name of the HDF5 file
/// \param data_name Name of the data frame
/// \param first An iterator points to the beginning of the variable vector
/// \param vname Name of the new variable
template <typename T, typename InputIter>
inline void hdf5_insert_data_frame (std::size_t N,
        const std::string &file_name, const std::string &data_name,
        InputIter first, const std::string &vname)
{
    if (N == 0)
        return;

    std::string dataset_name("/" + data_name + "/" + vname);
    hsize_t dim[1] = {N};

    hid_t datafile = H5Fopen(file_name.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    hid_t dataspace = H5Screate_simple(1, dim, NULL);
    hid_t datatype = hdf5_datatype<T>();
    internal::HDF5DataPtr<T> data_ptr;
    data_ptr.set(N, first);
    const T *data = data_ptr.get();
    hid_t dataset = H5Dcreate(datafile, dataset_name.c_str(),
            datatype, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

    H5Dclose(dataset);
    H5Tclose(datatype);
    H5Sclose(dataspace);
    H5Fclose(datafile);
}

/// \brief Save a Sampler in the HDF5 format
/// \ingroup HDF5Save
template <typename T>
inline void hdf5_save (const Sampler<T> &sampler,
        const std::string &file_name, const std::string &data_name,
        bool append = false)
{
    std::size_t nrow = sampler.iter_size();
    std::size_t ncol = sampler.summary_header_size();
    std::vector<std::string> header(ncol);
    std::vector<double> data(nrow * ncol);
    std::vector<int> resampled(nrow);
    sampler.summary_header(header.begin());
    sampler.template summary_data<ColMajor>(data.begin());
    sampler.read_resampled_history(resampled.begin());
    std::vector<const double *> data_ptr(ncol);
    for (std::size_t j = 0; j != ncol; ++j)
        data_ptr[j] = &data[j * nrow];

    hdf5_save_data_frame<double>(nrow, ncol, file_name, data_name,
            data_ptr.begin(), header.begin(), append);
    hdf5_insert_data_frame<int>(ncol, file_name, data_name,
            resampled.begin(), "Resampled");
}

/// \brief Save a StateMatrix in the HDF5 format
/// \ingroup HDF5Save
template <MatrixOrder Order, std::size_t Dim, typename T>
inline void hdf5_save (const StateMatrix<Order, Dim, T> &state,
        const std::string &file_name, const std::string &data_name,
        bool append = false)
{
    hdf5_save_matrix<Order, T>(state.size(), state.dim(), file_name, data_name,
            state.data(), append);
}

#if VSMC_HAS_CXX11_VARIADIC_TEMPLATES

namespace internal {

template <typename T, typename... Types>
inline void hdf5_save_state_tuple(
    const StateTuple<RowMajor, T, Types...> &state,
    const std::string &file_name, const std::string &data_name, Position<0>)
{
    typedef StateTuple<RowMajor, T, Types...> state_type;
    typedef typename state_type::state_tuple_base_type::template
        state_type<0>::type dtype;
    std::string vname("V0");
    std::vector<dtype> data_vec(state.size());
    state.read_state(Position<0>(), data_vec.begin());
    const dtype *data = &data_vec[0];
    hdf5_insert_data_frame<dtype>(state.size(), file_name, data_name,
            data, vname);
}

template <typename T, typename... Types, std::size_t Pos>
inline void hdf5_save_state_tuple(
    const StateTuple<RowMajor, T, Types...> &state,
    const std::string &file_name, const std::string &data_name, Position<Pos>)
{
    typedef StateTuple<RowMajor, T, Types...> state_type;
    typedef typename state_type::state_tuple_base_type::template
        state_type<Pos>::type dtype;
    std::stringstream ss;
    ss << 'V' << Pos;
    std::string vname(ss.str());
    std::vector<dtype> data_vec(state.size());
    state.read_state(Position<Pos>(), data_vec.begin());
    const dtype *data = &data_vec[0];
    hdf5_insert_data_frame<dtype>(state.size(), file_name, data_name,
            data, vname);
    hdf5_save_state_tuple(state, file_name, data_name, Position<Pos - 1>());
}

template <typename T, typename... Types>
inline void hdf5_save_state_tuple(
    const StateTuple<ColMajor, T, Types...> &state,
    const std::string &file_name, const std::string &data_name, Position<0>)
{
    typedef StateTuple<RowMajor, T, Types...> state_type;
    typedef typename state_type::state_tuple_base_type::template
        state_type<0>::type dtype;
    std::string vname("V0");
    const dtype *data = state.data(Position<0>());
    hdf5_insert_data_frame<dtype>(state.size(), file_name, data_name,
            data, vname);
}

template <typename T, typename... Types, std::size_t Pos>
inline void hdf5_save_state_tuple(
    const StateTuple<ColMajor, T, Types...> &state,
    const std::string &file_name, const std::string &data_name, Position<Pos>)
{
    typedef StateTuple<RowMajor, T, Types...> state_type;
    typedef typename state_type::state_tuple_base_type::template
        state_type<Pos>::type dtype;
    std::stringstream ss;
    ss << 'V' << Pos;
    std::string vname(ss.str());
    const dtype *data = state.data(Position<Pos>());
    hdf5_insert_data_frame<dtype>(state.size(), file_name, data_name,
            data, vname);
    hdf5_save_state_tuple(state, file_name, data_name, Position<Pos - 1>());
}

} // namespace vsmc::internal

/// \brief Save a StateTuple in the HDF5 format
/// \ingroup HDF5Save
template <MatrixOrder Order, typename T, typename... Types>
inline void hdf5_save (const StateTuple<Order, T, Types...> &state,
        const std::string &file_name, const std::string &data_name,
        bool append = false)
{
    static const std::size_t dim = sizeof...(Types) + 1;
    hdf5_save_data_frame<int>(0, 0, file_name, data_name,
            (int **) VSMC_NULLPTR, (std::string *) VSMC_NULLPTR, append);
    internal::hdf5_save_state_tuple(state, file_name, data_name,
            Position<dim - 1>());
}

#endif // VSMC_HAS_CXX11_VARIADIC_TEMPLATES

/// \brief Save a StateCL in the HDF5 format
/// \ingroup HDF5Save
template <MatrixOrder Order, typename T,
         std::size_t StateSize, typename FPType, typename ID>
inline void hdf5_save (const StateCL<StateSize, FPType, ID> &state,
        const std::string &file_name, const std::string &data_name,
        bool append = false)
{
    std::size_t nrow = state.size();
    std::size_t ncol = state.state_size() / sizeof(T);
    std::size_t N = nrow * ncol;
    std::vector<T> data(N);
    state.manager().template read_buffer<T>(state.state_buffer(), N, &data[0]);
    hdf5_save_matrix<Order, T>(nrow, ncol, file_name, data_name,
            &data[0], append);
}

} // namespace vsmc

#endif // VSMC_UTILITY_HDF5_SAVE_HPP
