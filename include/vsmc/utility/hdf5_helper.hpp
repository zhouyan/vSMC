#ifndef VSMC_UTILITY_HDF5_HELPER_HPP
#define VSMC_UTILITY_HDF5_HELPER_HPP

#include <vsmc/internal/common.hpp>
#include <hdf5.h>

namespace vsmc {

namespace internal {

template <typename> inline hid_t hdf5_datatype () {return -1;}

template <>
inline hid_t hdf5_datatype<char> ()
{return H5Tcopy(H5T_NATIVE_CHAR);}

template <>
inline hid_t hdf5_datatype<signed char> ()
{return H5Tcopy(H5T_NATIVE_SCHAR);}

template <>
inline hid_t hdf5_datatype<unsigned char> ()
{return H5Tcopy(H5T_NATIVE_UCHAR);}

template <>
inline hid_t hdf5_datatype<short> ()
{return H5Tcopy(H5T_NATIVE_SHORT);}

template <>
inline hid_t hdf5_datatype<unsigned short> ()
{return H5Tcopy(H5T_NATIVE_UCHAR);}

template <>
inline hid_t hdf5_datatype<int> ()
{return H5Tcopy(H5T_NATIVE_INT);}

template <>
inline hid_t hdf5_datatype<unsigned int> ()
{return H5Tcopy(H5T_NATIVE_UINT);}

template <>
inline hid_t hdf5_datatype<long> ()
{return H5Tcopy(H5T_NATIVE_LONG);}

template <>
inline hid_t hdf5_datatype<unsigned long> ()
{return H5Tcopy(H5T_NATIVE_ULONG);}

#if __cplusplus >= 201103L

template <>
inline hid_t hdf5_datatype<long long> ()
{return H5Tcopy(H5T_NATIVE_LLONG);}

template <>
inline hid_t hdf5_datatype<unsigned long long> ()
{return H5Tcopy(H5T_NATIVE_ULLONG);}

#endif // __cplusplus >= 201103L

template <>
inline hid_t hdf5_datatype<float> ()
{return H5Tcopy(H5T_NATIVE_FLOAT);}

template <>
inline hid_t hdf5_datatype<double> ()
{return H5Tcopy(H5T_NATIVE_DOUBLE);}

template <>
inline hid_t hdf5_datatype<long double> ()
{return H5Tcopy(H5T_NATIVE_LDOUBLE);}

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

template <typename OutputIter, typename InputIter>
inline OutputIter hdf5_copy_data (std::size_t N, OutputIter dst, InputIter src)
{
    for (std::size_t i = 0; i != N; ++i, ++dst, ++src)
        *dst = *src;

    return dst;
}

template <typename T>
inline const T *hdf5_data_ptr (std::size_t N, T *first, T *)
{return first;}

template <typename T>
inline const T *hdf5_data_ptr (std::size_t N, const T *first, T *)
{return first;}

template <typename T, typename InputIter>
inline const T *hdf5_data_ptr (std::size_t N, InputIter first, T *tmp)
{
    for (std::size_t i = 0; i != N; ++i, ++first)
        tmp[i] = *first;

    return tmp;
}

} // namespace vsmc::internal

template <MatrixOrder Order, typename T>
inline const T *hdf5_write_matrix (std::size_t nrow, std::size_t ncol,
        const std::string &data_name, const std::string &file_name,
        const T *first, bool append = false)
{
    if (nrow == 0 || ncol == 0)
        return first;

    std::string dataset_name("/" + data_name);
    hsize_t dim[2];
    internal::hdf5_matrix_dim<Order>(nrow, ncol, dim);

    hid_t datafile;
    if (append) {
        datafile = H5Fopen(file_name.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    } else {
        datafile = H5Fcreate(file_name.c_str(),
                H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    }
    hid_t dataspace = H5Screate_simple(2, dim, NULL);
    hid_t datatype = internal::hdf5_datatype<T>();
    hid_t dataset = H5Dcreate(datafile, dataset_name.c_str(),
            datatype, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, first);

    H5Dclose(dataset);
    H5Tclose(datatype);
    H5Sclose(dataspace);
    H5Fclose(datafile);

    return first + nrow * ncol;
}

template <MatrixOrder Order, typename T>
inline T *hdf5_write_matrix (std::size_t nrow, std::size_t ncol,
        const std::string &data_name, const std::string &file_name,
        T *first, bool append = false)
{
    const T *cfirst = first;
    hdf5_write_matrix<Order>(
            nrow, ncol, data_name, file_name, cfirst, append);

    return first + nrow * ncol;
}

template <MatrixOrder Order, typename T, typename InputIter>
inline InputIter hdf5_write_matrix (std::size_t nrow, std::size_t ncol,
        const std::string &data_name, const std::string &file_name,
        InputIter first, bool append = false)
{
    std::size_t N = nrow * ncol;
    std::vector<T> data(N);
    for (std::size_t i = 0; i != N; ++i, ++first)
        data[i] = *first;
    hdf5_write_matrix<Order>(
            nrow, ncol, data_name, file_name, &data[0], append);

    return first;
}

template <typename T, typename InputIterIter>
inline void hdf5_write_matrix (std::size_t nrow, std::size_t ncol,
        const std::string &data_name, const std::string &file_name,
        InputIterIter first, bool append = false)
{
    std::vector<T> data(nrow * ncol);
    T *dst = &data[0];
    for (std::size_t c = 0; c != ncol; ++c, ++first)
        dst = internal::hdf5_copy_data(nrow, &dst, *first);
    hdf5_write_matrix<ColMajor>(
            nrow, ncol, data_name, file_name, &data[0], append);
}

template <typename T, typename InputIterIter, typename SInputIter>
inline void hdf5_write_data_frame (std::size_t nrow, std::size_t ncol,
        const std::string &data_name, const std::string &file_name,
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
        hid_t datatype = internal::hdf5_datatype<T>();
        T *data_tmp = new T[nrow];
        for (std::size_t j = 0; j != ncol; ++j, ++first, ++sfirst) {
            const T *data = internal::hdf5_data_ptr(nrow, *first, data_tmp);
            std::string dataset_name(group_name + "/" + (*sfirst));
            hid_t dataset = H5Dcreate(datafile, dataset_name.c_str(),
                    datatype, dataspace,
                    H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
            H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
            H5Dclose(dataset);
        }
        delete [] data_tmp;
        H5Tclose(datatype);
        H5Sclose(dataspace);
    }

    H5Gclose(datagroup);
    H5Fclose(datafile);
}

template <typename T, typename InputIter>
inline void hdf5_append_data_frame (std::size_t N,
        const std::string &data_name, const std::string &file_name,
        InputIter first, const std::string &vname)
{
    if (N == 0)
        return;

    std::string dataset_name("/" + data_name + "/" + vname);
    hsize_t dim[1] = {N};
    T *data_tmp = new T[N];

    hid_t datafile = H5Fopen(file_name.c_str(), H5F_ACC_RDWR, H5P_DEFAULT);
    hid_t dataspace = H5Screate_simple(1, dim, NULL);
    hid_t datatype = internal::hdf5_datatype<T>();
    const T *data = internal::hdf5_data_ptr(N, first, data_tmp);
    hid_t dataset = H5Dcreate(datafile, dataset_name.c_str(),
            datatype, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(dataset, datatype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

    H5Dclose(dataset);
    H5Tclose(datatype);
    H5Sclose(dataspace);
    H5Fclose(datafile);
    delete [] data_tmp;
}

} // namespace vsmc

#endif // VSMC_UTILITY_HDF5_HELPER_HPP
