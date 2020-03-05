aux_source_directory(${CMAKE_CURRENT_LIST_DIR} SRCFILES)
add_library(psim SHARED ${SRCFILES})

# Tetgen, used by seven2
# add_subdirectory(${CMAKE_SOURCE_DIR}/third-party/tetgen)
# target_include_directories(psim
# 	BEFORE PUBLIC third-party/tetgen
# )
# target_link_libraries(psim PUBLIC tetgen)
