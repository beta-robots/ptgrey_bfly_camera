# Test for camera acquisition
ADD_EXECUTABLE(bfly_capture bfly_capture_main.cpp)
TARGET_LINK_LIBRARIES(bfly_capture ${PROJECT_NAME}_LIB)

ADD_EXECUTABLE(bfly_camera_client_main bfly_camera_client_main.cpp)
TARGET_LINK_LIBRARIES(bfly_camera_client_main ${PROJECT_NAME}_LIB ${catkin_LIBRARIES})

