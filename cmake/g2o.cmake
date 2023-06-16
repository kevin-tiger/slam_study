# g2o 使用thirdparty中的
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o/)
set(g2o_libs
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_core.so
	${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_cholmod.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_dense.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_csparse_extension.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_sba.so
        ${CSPARSE_LIBRARY}
        ${CHOLMOD_LIBRARY}
)