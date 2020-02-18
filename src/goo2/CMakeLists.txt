add_custom_target(link_goo2 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_LIST_DIR}/goo2.py ${CMAKE_SOURCE_DIR}/bin/goo2.py
	SOURCES goo2.py
)
add_custom_target(link_reference_goo2 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_LIST_DIR}/reference_goo2.py
	${CMAKE_SOURCE_DIR}/bin/reference_goo2.py
	SOURCES reference_goo2.py
)
add_dependencies(link_goo2 pypsim)
add_dependencies(link_reference_goo2 pypsim)
