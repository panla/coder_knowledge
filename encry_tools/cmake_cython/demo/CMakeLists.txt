cmake_minimum_required(VERSION 3.0.2)

##增加需要的子模块
set(_module_name_list "demo" )

#编译相关的文件
foreach(_module_name ${_module_name_list})
    cython_add_module(${_module_name} ${_module_name}.py)
    set_target_properties(${_module_name}
      PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
    install(TARGETS ${_module_name}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
endforeach()
