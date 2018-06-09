find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
include_directories(${PYTHON_INCLUDE_PATH})

set(STAGES_LIB_NAME _stages)
configure_file(src/stages.cpp.in stages.cpp @ONLY)
add_library(${STAGES_LIB_NAME} SHARED
	src/stages.cpp
	${CMAKE_CURRENT_BINARY_DIR}/stages.cpp
)
target_link_libraries(${STAGES_LIB_NAME} PRIVATE
	${PROJECT_NAME}_stages ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})


set(MOVEIT_PYTHON_MODULE_PATH mtc_demos)
set_target_properties(${STAGES_LIB_NAME}
	PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
	PREFIX "")

install(TARGETS ${STAGES_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
	LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
)

# Add symlinks from source space to devel-space libs
# This is necessary to find them from devel-space too
foreach(tgt ${STAGES_LIB_NAME})
add_custom_command(TARGET ${tgt} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E create_symlink "$<TARGET_FILE:${tgt}>" "${CMAKE_CURRENT_SOURCE_DIR}/../src/${MOVEIT_PYTHON_MODULE_PATH}/$<TARGET_FILE_NAME:${tgt}>"
	COMMENT "Creating symlink to $<TARGET_FILE_NAME:${tgt}> in source space"
)
endforeach()
