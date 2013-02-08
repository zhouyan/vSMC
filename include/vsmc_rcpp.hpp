#ifndef VSMC_RCPP_HPP
#define VSMC_RCPP_HPP

#include <vsmc.hpp>
#include <Rcpp.h>

/// \defgroup Rcpp Rcpp wrappers
/// \brief Rcpp wrappers for vSMC classes

namespace vsmc {

/// \brief Rcpp wrapper of Sampler
/// \ingroup Rcpp
template <typename T>
inline SEXP wrap (const Sampler<T> &sampler)
{
    Rcpp::NumericMatrix summary_data(
            sampler.iter_size(),
            sampler.summary_header_size());
    sampler.summary_data(ColMajor, summary_data.begin());

    Rcpp::IntegerVector index(sampler.iter_size());
    for (std::size_t i = 0; i != sampler.iter_size(); ++i)
        index[i] = i;

    Rcpp::CharacterVector rownames(sampler.summary_header_size());
    sampler.summary_header(rownames.begin());

    summary_data.attr("dimnames") = Rcpp::List::create(index, rownames);

    return Rcpp::DataFrame(summary_data);
}

/// \brief Rcpp wrapper of Monitor
/// \ingroup Rcpp
template <typename T>
inline SEXP wrap (const Monitor<T> &monitor)
{
    Rcpp::IntegerVector index(monitor.iter_size());
    monitor.read_index(index.begin());

    Rcpp::NumericMatrix record(monitor.iter_size(), monitor.dim());
    monitor.read_record(ColMajor, record.begin());

    record.attr("dimnames") = Rcpp::List::create(
            index, Rcpp::CharacterVector::create());

    return Rcpp::DataFrame(record);
}

/// \brief Rcpp wrapper of Path
/// \ingroup Rcpp
template <typename T>
inline SEXP wrap (const Path<T> &path)
{
    Rcpp::IntegerVector index(path.iter_size());
    path.read_index(index.begin());

    Rcpp::NumericMatrix record(path.iter_size(), 2);
    path.read_integrand(record.column(0).begin());
    path.read_grid(record.column(1).begin());

    Rcpp::CharacterVector rownames;
    rownames.push_back("Integrand");
    rownames.push_back("Grid");
    record.attr("dimnames") = Rcpp::List::create(index, rownames);

    return Rcpp::List::create(
            Rcpp::Named("Estimate") = path.zconst(),
            Rcpp::Named("Path") = Rcpp::DataFrame(record));
}

} // namespace vsmc

#endif // VSMC_RCPP_HPP
