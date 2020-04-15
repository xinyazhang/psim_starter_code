add_custom_target(link_cloth1 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_LIST_DIR}/cloth1.py ${CMAKE_SOURCE_DIR}/bin/cloth1.py
	SOURCES cloth1.py
)
add_custom_target(link_reference_cloth1 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_LIST_DIR}/reference_cloth1.py
	${CMAKE_SOURCE_DIR}/bin/reference_cloth1.py
	SOURCES reference_cloth1.py
)
add_dependencies(link_cloth1 pypsim)
add_dependencies(link_reference_cloth1 pypsim)
