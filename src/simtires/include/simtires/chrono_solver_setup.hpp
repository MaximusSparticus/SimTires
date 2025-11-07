// Utility function for selecting the Chrono solver and integrator.

#pragma once

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

namespace simtires {

/// Set up Chrono solver and integrator for the given system
/// @param sys The Chrono system to configure
/// @param solver_type The desired solver type
/// @param integrator_type The desired integrator type
/// @param num_threads_mkl Number of threads for MKL (if using Pardiso)
/// @param verbose Whether to print configuration messages
/// @return true if successful, false if configuration failed
bool SetupChronoSolver(chrono::ChSystem& sys,
                      const chrono::ChSolver::Type& solver_type,
                      const chrono::ChTimestepper::Type& integrator_type,
                      int num_threads_mkl = 1,
                      bool verbose = true);

} // namespace simtires
