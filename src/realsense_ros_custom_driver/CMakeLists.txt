cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_custom_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
find_package(OpenCV 4 REQUIRED  PATHS "/usr/local" NO_DEFAULT_PATH)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	message_runtime
)


find_package(OpenGL)
if(NOT OPENGL_FOUND)
	  message(FATAL_ERROR "\n\n OpenGL package is missing!\n\n")
endif()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  )

add_message_files(
  DIRECTORY msg FILES img_stream.msg
)
generate_messages(
  DEPENDENCIES 
  std_msgs
)

set(DEPENDENCIES realsense2 ${OPENGL_LIBRARIES})

if(WIN32)
	  list(APPEND DEPENDENCIES glfw3)
else()
	  # Find glfw header
		find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
			  PATHS /usr/X11R6/include
				      /usr/include/X11
							/opt/graphics/OpenGL/include
							/opt/graphics/OpenGL/contrib/libglfw
							/usr/local/include
							/usr/include/GL
							/usr/include
							)
						# Find glfw library
		find_library(GLFW_LIBRARIES NAMES glfw glfw3
			      PATHS /usr/lib64
						      /usr/lib
									/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
									/usr/local/lib64
									/usr/local/lib
									/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
									/usr/X11R6/lib
									)
endif()

find_package(realsense2)
if(NOT realsense2_FOUND)
	  message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs roscpp
  DEPENDS OpenCV realsense2
  LIBRARIES ${PROJECT_NAME}
)

add_executable(stream_publisher src/stream_publisher.cpp)
add_dependencies(stream_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stream_publisher
	${catkin_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	realsense2
	${OPENGL_LIBRARIES}
	${OpenCV_LIBS}
	glfw
)

add_library(${PROJECT_NAME}
  src/stream_publisher.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
