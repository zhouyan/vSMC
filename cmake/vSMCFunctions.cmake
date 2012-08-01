FUNCTION (ADD_ALGORITHMS basename exes algs compile_flags ld_flags)
    FOREACH (exe ${exes})
        STRING (TOUPPER "-DUSE_${exe}" flags)
        FOREACH (alg ${algs})
            SET (exe_name ${basename}-${alg}-${exe})
            ADD_EXECUTABLE (${exe_name} ${basename}-${alg}.cpp)

            IF (${exe} STREQUAL "omp")
                SET_TARGET_PROPERTIES (${exe_name} PROPERTIES COMPILE_FLAGS
                    "${compile_flags} ${flags} ${OpenMP_CXX_FLAGS}")
            ELSE (${exe} STREQUAL "omp")
                SET_TARGET_PROPERTIES (${exe_name} PROPERTIES COMPILE_FLAGS
                    "${compile_flags} ${flags}")
            ENDIF (${exe} STREQUAL "omp")

            IF (${exe} STREQUAL "tbb")
                TARGET_LINK_LIBRARIES (${exe_name}
                    ${ld_flags} ${TBB_LINK_LIBRARIES})
            ELSEIF (${exe} STREQUAL "omp")
                TARGET_LINK_LIBRARIES (${exe_name}
                    ${ld_flags} ${OpenMP_LINK_LIBRARIES})
            ELSE (${exe} STREQUAL "tbb")
                TARGET_LINK_LIBRARIES (${exe_name} ${ld_flags})
            ENDIF (${exe} STREQUAL "tbb")

            ADD_DEPENDENCIES (${basename} ${exe_name})
        ENDFOREACH(alg)
    ENDFOREACH (exe)
ENDFUNCTION (ADD_ALGORITHMS)

FUNCTION (COPY_FILE basename filename)
    ADD_CUSTOM_COMMAND (
        OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
        DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
        COMMAND ${CMAKE_COMMAND} ARGS -E copy
        ${PROJECT_SOURCE_DIR}/${filename} ${PROJECT_BINARY_DIR}/${filename})
    ADD_CUSTOM_TARGET (${basename}-${filename}
        DEPENDS ${PROJECT_BINARY_DIR}/${filename})
    ADD_DEPENDENCIES (${basename} ${basename}-${filename})
ENDFUNCTION (COPY_FILE)
