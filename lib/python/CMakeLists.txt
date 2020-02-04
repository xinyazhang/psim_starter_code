add_subdirectory(${CMAKE_SOURCE_DIR}/third-party/pybind11 submodule)

aux_source_directory(. PYPSIM_SRC)
lsdir(subs ${CMAKE_CURRENT_LIST_DIR})
foreach(sub ${subs})
	aux_source_directory(${sub} PYPSIM_SRC)
	message(STATUS "Current PYPSIM_SRC ${PYPSIM_SRC} after ${sub}")
endforeach()

pybind11_add_module(pypsim ${PYPSIM_SRC})
target_include_directories(pypsim BEFORE PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(pypsim PUBLIC psim)
if (PSIM_ENABLE_VISUALIZER)
	target_compile_definitions(pypsim PUBLIC PSIM_ENABLE_VISUALIZER=1)
else ()
	target_compile_definitions(pypsim PUBLIC PSIM_ENABLE_VISUALIZER=0)
endif ()

foreach(sub ${subs})
	string(TOUPPER "${sub}" item)
	target_compile_definitions(pypsim PUBLIC PSIM_HAS_${item})
endforeach()

set_relative_rpath(pypsim)
