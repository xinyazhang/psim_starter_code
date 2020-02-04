add_custom_target(link_goo1 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_LIST_DIR}/goo1.py ${CMAKE_SOURCE_DIR}/bin/goo1.py
	SOURCES goo1.py
)
add_custom_target(link_reference_goo1 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_LIST_DIR}/reference_goo1.py
	${CMAKE_SOURCE_DIR}/bin/reference_goo1.py
	SOURCES reference_goo1.py
)
add_custom_target(link_demo_test_goo1 ALL
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	${CMAKE_CURRENT_LIST_DIR}/demo_test_goo1.py
	${CMAKE_SOURCE_DIR}/bin/demo_test_goo1.py
	SOURCES demo_test_goo1.py
)
add_dependencies(link_goo1 pypsim)
add_dependencies(link_reference_goo1 pypsim)
add_dependencies(link_demo_test_goo1 pypsim)
