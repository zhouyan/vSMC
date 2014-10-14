//============================================================================
// include/vsmc/mpi/mpi_manager.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_MPI_MPI_MANAGER_HPP
#define VSMC_MPI_MPI_MANAGER_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/rng/seed.hpp>
#include <boost/mpi.hpp>

namespace vsmc {

namespace internal {

template <typename ResultType, typename IntType1, typename IntType2>
inline void mpi_init_seed (ResultType &, IntType1 D, IntType2 R)
{
    Seed::instance().modulo(
            static_cast<Seed::skip_type>(D), static_cast<Seed::skip_type>(R));
}

template <typename T, std::size_t K, typename IntType1, typename IntType2>
inline void mpi_init_seed (vsmc::Array<T, K> &s, IntType1, IntType2 R)
{s.back() = static_cast<Seed::skip_type>(R);}

} // namespace vsmc::internal

/// \brief MPI Environment
/// \ingroup MPI
///
/// \details
/// Use this class in place of `boost::mpi::environment` to correctly
/// initialize Seed
class MPIEnvironment
{
    public :

#ifdef BOOST_MPI_HAS_NOARG_INITIALIZATION
    explicit MPIEnvironment (bool abort_on_exception = true) :
        env_(abort_on_exception) {init_seed();}
#endif

    MPIEnvironment(int &argc, char **&argv, bool abort_on_exception = true) :
        env_(argc, argv, abort_on_exception) {init_seed();}

    private :

    ::boost::mpi::environment env_;

    void init_seed () const
    {
        ::boost::mpi::communicator world;
        Seed::result_type s(Seed::instance().get());
        internal::mpi_init_seed(s, world.size(), world.rank());
        Seed::instance().set(s);
        world.barrier();
    }
}; // class MPIEnvironment

/// \brief MPI Communicator
/// \ingroup MPI
///
/// \details
/// Use specialization of the singleton to configure different StateMPI
template <typename ID>
class MPICommunicator
{
    public :

    static MPICommunicator<ID> &instance ()
    {
        static MPICommunicator<ID> comm;

        return comm;
    }

    const MPI_Comm &get () const {return comm_;}

    void set (const MPI_Comm &comm) {comm_ = comm;}

    private :

    MPI_Comm comm_;

    MPICommunicator () : comm_(MPI_COMM_WORLD) {}
    MPICommunicator (const MPICommunicator<ID> &);
    MPICommunicator<ID> &operator= (const MPICommunicator<ID> &);
}; // class MPICommunicator

} // namespace vsmc

#endif // VSMC_MPI_MPI_MANAGER_HPP
