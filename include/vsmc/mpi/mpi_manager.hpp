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

/// \brief MPI Environment
/// \ingroup MPI
///
/// \details
/// Use this class in place of boost::mpi::environment to correctly initialize
/// Seed
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

    boost::mpi::environment env_;

    void init_seed () const
    {
        boost::mpi::communicator world;
        Seed::instance().modulo(
                static_cast<Seed::result_type>(world.size()),
                static_cast<Seed::result_type>(world.rank()));
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
