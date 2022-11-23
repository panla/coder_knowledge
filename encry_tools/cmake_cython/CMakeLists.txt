


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# include CMake module for Cython
include(UseCython)
add_custom_target(ReplicatePythonSourceTree ALL ${CMAKE_COMMAND} -P
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ReplicatePythonSourceTree.cmake
  ${CMAKE_CURRENT_BINARY_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(scripts)

#target_link_libraries( snack Common -lpthread -lm)
