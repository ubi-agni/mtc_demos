set(STAGES_LIB_NAME _stages)
configure_file(src/stages.cpp.in stages.cpp @ONLY)
pybind11_add_module(${STAGES_LIB_NAME} MODULE
	src/stages.cpp
	${CMAKE_CURRENT_BINARY_DIR}/stages.cpp
)
target_link_libraries(${STAGES_LIB_NAME} PRIVATE
	${PROJECT_NAME}_stages ${catkin_LIBRARIES})


set(MOVEIT_PYTHON_MODULE_PATH mtc_demos)
install(TARGETS ${STAGES_LIB_NAME}
	ARCHIVE DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
	LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/${MOVEIT_PYTHON_MODULE_PATH}
)

# Add symlinks from source space to devel-space libs
# This is necessary to find them from devel-space too
foreach(tgt ${STAGES_LIB_NAME})
add_custom_command(TARGET ${tgt} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E create_symlink "$<TARGET_FILE:${tgt}>" "${CMAKE_CURRENT_SOURCE_DIR}/../src/${MOVEIT_PYTHON_MODULE_PATH}/$<TARGET_FILE_NAME:${tgt}>"
	COMMENT "Creating symlink to ${tgt} in source space"
)
endforeach()
