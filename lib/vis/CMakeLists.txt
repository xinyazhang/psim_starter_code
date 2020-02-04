lsdir(subs ${CMAKE_CURRENT_LIST_DIR})
foreach(sub ${subs})
	add_subdirectory(${sub})
endforeach()
