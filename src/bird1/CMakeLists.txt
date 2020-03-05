add_custom_target(link_bird1 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_LIST_DIR}/bird1.py ${CMAKE_SOURCE_DIR}/bin/bird1.py
	SOURCES bird1.py
)
add_custom_target(link_reference_bird1 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_LIST_DIR}/reference_bird1.py
	${CMAKE_SOURCE_DIR}/bin/reference_bird1.py
	SOURCES reference_bird1.py
)
add_dependencies(link_bird1 pypsim)
add_dependencies(link_reference_bird1 pypsim)
