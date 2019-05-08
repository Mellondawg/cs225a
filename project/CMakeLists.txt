# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/project)
ADD_EXECUTABLE (controller_project controller_project.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_project simviz_project.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_project_slow simviz_project_slow.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_project ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_project ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_project_slow ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})
