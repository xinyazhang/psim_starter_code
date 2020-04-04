#
# Core of libpsim
# 
aux_source_directory(core/ PSIM_SRC)
lsdir(subs ${CMAKE_CURRENT_LIST_DIR}/core)
foreach(sub ${subs})
	message(STATUS "adding ${sub}")
	aux_source_directory(core/${sub} PSIM_SRC)
endforeach()

#
# Visualizer
# 
if (PSIM_ENABLE_VISUALIZER)
	aux_source_directory(vis/ PSIM_VIS_SRC)
	lsdir(subs ${CMAKE_CURRENT_LIST_DIR}/vis)
	foreach(sub ${subs})
		aux_source_directory(vis/${sub} PSIM_VIS_SRC)
	endforeach()
else ()
	SET(PSIM_VIS_SRC)
endif ()

#
# Create the library
#
add_library(psim STATIC ${PSIM_SRC} ${PSIM_VIS_SRC})
target_include_directories(psim BEFORE PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(psim PUBLIC psim_std_dep)

#
# Link psim to tetgen
# 
target_include_directories(psim BEFORE PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../third-party/tetgen")
target_link_libraries(psim PRIVATE tetgen)

if (PSIM_ENABLE_VISUALIZER)
	target_compile_definitions(psim PUBLIC PSIM_ENABLE_VISUALIZER=1)
	target_link_libraries(psim PUBLIC psim_gui_dep)
else ()
	target_compile_definitions(psim PUBLIC PSIM_ENABLE_VISUALIZER=0)
endif ()
set_relative_rpath(psim)


#
# Python bindings
# 
add_subdirectory(python)
