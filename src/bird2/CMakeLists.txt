add_custom_target(link_bird2 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_LIST_DIR}/bird2.py ${CMAKE_SOURCE_DIR}/bin/bird2.py
	SOURCES bird2.py
)
add_custom_target(link_bird2_demo ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_LIST_DIR}/demo_bird2_new_interface.py ${CMAKE_SOURCE_DIR}/bin/demo_bird2_new_interface.py
	SOURCES demo_bird2_new_interface.py
)
add_custom_target(link_reference_bird2 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_LIST_DIR}/reference_bird2.py
	${CMAKE_SOURCE_DIR}/bin/reference_bird2.py
	SOURCES reference_bird2.py
)
add_dependencies(link_bird2 pypsim)
add_dependencies(link_reference_bird2 pypsim)
add_dependencies(link_bird2_demo pypsim)
