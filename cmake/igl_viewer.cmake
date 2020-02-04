set(OpenGL_GL_PREFERENCE GLVND) 
find_package(OpenGL REQUIRED)

# glad
add_library(glad STATIC ${CMAKE_SOURCE_DIR}/bundle/glad/src/glad.c)
target_include_directories(glad
	PUBLIC ${CMAKE_SOURCE_DIR}/bundle/glad/include
)
target_include_directories(glad
	INTERFACE ${CMAKE_SOURCE_DIR}/bundle/glad/include
)
target_link_libraries(glad
	${CMAKE_DL_LIBS}
)

# libigl-imgui
add_subdirectory(${CMAKE_SOURCE_DIR}/third-party/libigl-imgui/)

SET(BUILD_SHARED_LIBS ON)
SET(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
SET(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
SET(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third-party/glfw)
# For older ( < 3.13) cmake
# Newer should use target_include_directories(psim_std_dep ...)
# link_directories(BEFORE ${EXTERNAL_PROJECTS_INSTALL_PREFIX}/lib ${EXTERNAL_PROJECTS_INSTALL_PREFIX}/lib64)

# libigl
# ... Well it's used in header only manner
# END OF libigl

add_library(psim_gui_dep INTERFACE)
add_dependencies(psim_gui_dep ext_glfw)
target_include_directories(psim_gui_dep
	INTERFACE ${CMAKE_SOURCE_DIR}/third-party/libigl/include/ ${EXTERNAL_PROJECTS_INSTALL_PREFIX}/include
)
target_link_libraries(psim_gui_dep INTERFACE imgui glad glfw Eigen3::Eigen Threads::Threads)
