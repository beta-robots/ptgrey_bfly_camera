# Test for camera acquisition
ADD_EXECUTABLE(bfly_capture bfly_capture_main.cpp)
TARGET_LINK_LIBRARIES(bfly_capture ${PROJECT_NAME}_LIB)
