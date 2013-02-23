FUNCTION (ADD_SMP_EXECUTABLE base header source smp_name)
    STRING (TOUPPER "${smp_name}" SMP)
    STRING (TOLOWER "${smp_name}" smp)

    IF (${smp} STREQUAL "cilk")
        SET (HALF_NUM_THREADS
            "int tnum = __cilkrts_get_nworkers();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "tnum = tnum > 1 ? tnum / 2 : tnum;")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "std::stringstream tnum_ss; tnum_ss << tnum;")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "__cilkrts_set_param(\"nworkers\", tnum_ss.str().c_str());")
    ELSEIF (${smp} STREQUAL "omp")
        SET (HALF_NUM_THREADS
            "int tnum = omp_get_max_threads();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "omp_set_num_threads(tnum > 1 ? tnum / 2 : tnum);")
    ELSEIF (${smp} STREQUAL "std")
        SET (HALF_NUM_THREADS
            "vsmc::ThreadInfo &tinfo = vsmc::ThreadInfo::instance();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "std::size_t tnum = tinfo.thread_num();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "tinfo.thread_num(tnum > 1 ? tnum / 2 : tnum);")
    ELSEIF (${smp} STREQUAL "tbb")
        SET (HALF_NUM_THREADS
            "int tnum = tbb::task_scheduler_init::default_num_threads();")
        SET (HALF_NUM_THREADS ${HALF_NUM_THREADS}
            "tbb::task_scheduler_init init(tnum > 1 ? tnum / 2 : tnum);")
    ENDIF (${smp} STREQUAL "cilk")

    CONFIGURE_FILE (
        ${PROJECT_SOURCE_DIR}/${header}.hpp
        ${PROJECT_SOURCE_DIR}/${header}-${smp}.hpp)
    CONFIGURE_FILE (
        ${PROJECT_SOURCE_DIR}/${source}.cpp
        ${PROJECT_SOURCE_DIR}/${source}-${smp}.cpp)

    SET (GENERATED_SOURCE ${GENERATED_SOURCE}
        ${PROJECT_SOURCE_DIR}/${header}-${smp}.hpp
        CACHE INTERNAL "Generated source")
    SET (GENERATED_SOURCE ${GENERATED_SOURCE}
        ${PROJECT_SOURCE_DIR}/${source}-${smp}.cpp
        CACHE INTERNAL "Generated source")

    ADD_EXECUTABLE (${source}-${smp} ${source}-${smp}.cpp)

    IF (${smp} STREQUAL "gcd")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${GCD_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "omp")
        SET_TARGET_PROPERTIES (${source}-${smp} PROPERTIES COMPILE_FLAGS
            "${OpenMP_CXX_FLAGS}")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${OpenMP_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "std")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${VSMC_THREAD_LINK_LIBRARIES})
    ELSEIF (${smp} STREQUAL "tbb")
        TARGET_LINK_LIBRARIES (${source}-${smp} ${TBB_LINK_LIBRARIES})
    ENDIF (${smp} STREQUAL "gcd")

    IF (${source} MATCHES "-mpi")
        TARGET_LINK_LIBRARIES (${source}-${smp}
            ${Boost_LIBRARIES} ${MPI_CXX_LIBRARIES})
    ENDIF (${source} MATCHES "-mpi")

    ADD_DEPENDENCIES (${base} ${source}-${smp})
ENDFUNCTION (ADD_SMP_EXECUTABLE)

FUNCTION (ADD_EXAMPLE base algs)
    FOREACH (smp ${EXAMPLE_EXECUTABLES})
        IF (${smp} STREQUAL "tbb")
            ADD_TBB_RUNTIME(${base})
        ENDIF (${smp} STREQUAL "tbb")
        FOREACH (alg ${algs})
            ADD_SMP_EXECUTABLE(${base} ${base} ${base}-${alg} ${smp})
            TARGET_LINK_LIBRARIES (${base}-${alg}-${smp}
                ${EXAMPLE_LINK_LIBRARIES})
        ENDFOREACH(alg)
    ENDFOREACH (smp)
ENDFUNCTION (ADD_EXAMPLE)

FUNCTION (COPY_FILE basename filename)
    ADD_CUSTOM_COMMAND (
        OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
        DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy
        ${PROJECT_SOURCE_DIR}/${filename} ${PROJECT_BINARY_DIR}/${filename})
    ADD_CUSTOM_TARGET (${basename}-${filename}
        DEPENDS ${PROJECT_BINARY_DIR}/${filename})
    ADD_DEPENDENCIES (${basename}-files ${basename}-${filename})
ENDFUNCTION (COPY_FILE)

FUNCTION (COPY_DIRECTORY basename dirname fullpath)
    FILE (GLOB_RECURSE dirfiles
        ${fullpath}/*.h ${fullpath}/*.hpp
        ${fullpath}/*.c ${fullpath}/*.cpp ${fullpath}/*.cl)
    ADD_CUSTOM_COMMAND (
        OUTPUT ${PROJECT_BINARY_DIR}/${dirname} DEPENDS ${dirfiles}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_directory
        ${fullpath} ${PROJECT_BINARY_DIR}/${dirname})
    ADD_CUSTOM_TARGET (${basename}-${dirname}
        DEPENDS ${PROJECT_BINARY_DIR}/${dirname})
    ADD_DEPENDENCIES (${basename}-files ${basename}-${dirname})
ENDFUNCTION (COPY_DIRECTORY)

FUNCTION (COPY_FILE_OPTIONAL basename filename)
    IF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
        COPY_FILE (${basename} ${filename})
    ELSE (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
            MESSAGE (STATUS "File: ${basename}: ${filename} not available")
    ENDIF (EXISTS ${PROJECT_SOURCE_DIR}/${filename})
ENDFUNCTION (COPY_FILE_OPTIONAL)
