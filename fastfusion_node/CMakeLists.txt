cmake_minimum_required(VERSION 2.8)
project(fastfusion_node)

#################################################
# 	Perform all set commands                   	#
#################################################
add_definitions(-DCOMPILEDWITHC11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
#set(CMAKE_BUILD_TYPE Debug)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_calib3d)
#set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ${SSE_FLAGS} -fno-omit-frame-pointer") # -mavx2 -funroll-loops -DOWNAVX")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall") # -mavx2 -funroll-loops -DOWNAVX")
if(UNIX AND NOT APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse4.2 -funroll-loops")

set(FORCE_GLUT true)

#################################################
#	Find the necessary Pkgs 					#
#################################################
find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen REQUIRED)
find_package(Threads REQUIRED)
find_package(Pangolin 0.4 REQUIRED)

message(STATUS "Pangolin inc: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "Pangolin lib: ${Pangolin_LIBRARIES}")


#################################################
# 	Include Directories 	                   	#
#################################################
include_directories(
		${PCL_INCLUDE_DIRS}
		${CMAKE_CURRENT_BINARY_DIR}
		${Eigen_INCLUDE_DIRS}
#		/usr/include/eigen3
		${Pangolin_INCLUDE_DIRS}

)

catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#################################################
# 	Add Executables 	 	                   	#
#################################################
add_executable(fastfusion_node
		src/main_fastfusion_node.cpp
		src/fastfusion_node.cpp
		src/online_fusion_ros.cpp
		src/pangolin_viewer.cpp)

#################################################
# 	Link to target Libraries                   	#
#################################################
target_link_libraries(fastfusion_node
		${catkin_LIBRARIES}
		${CATKIN_DEVEL_PREFIX}/lib/libgeometryfusion_mipmap_cpu.a
		${CATKIN_DEVEL_PREFIX}/lib/libgeometryfusion_aos.a
		${CATKIN_DEVEL_PREFIX}/lib/libcamerautils.a
		${CATKIN_DEVEL_PREFIX}/lib/libauxiliary.a
		${OpenCV_LIBS}
		boost_thread
		boost_system
		${PCL_LIBRARIES}
		pcl_common
		pcl_io
		pcl_visualization
		pcl_surface
		${CMAKE_THREAD_LIBS_INIT}
		${Pangolin_LIBRARIES})
add_dependencies(fastfusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(
		pangolin_test
		src/pangolin_test.cpp
		src/fastfusion_node.cpp
		src/online_fusion_ros.cpp
		src/pangolin_viewer.cpp
)
target_link_libraries(pangolin_test
		${catkin_LIBRARIES}
		${CATKIN_DEVEL_PREFIX}/lib/libgeometryfusion_mipmap_cpu.a
		${CATKIN_DEVEL_PREFIX}/lib/libgeometryfusion_aos.a
		${CATKIN_DEVEL_PREFIX}/lib/libcamerautils.a
		${CATKIN_DEVEL_PREFIX}/lib/libauxiliary.a
		${OpenCV_LIBS}
		boost_thread
		boost_system
		${PCL_LIBRARIES}
		pcl_common
		pcl_io
		pcl_visualization
		pcl_surface
		${CMAKE_THREAD_LIBS_INIT}
		${Pangolin_LIBRARIES})

cs_install()
cs_export()
