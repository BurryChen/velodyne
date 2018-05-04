# cmake_minimum_required(VERSION 2.8.3)
# project(calibration_HDL32)
# 
# set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# 
# 添加cmake模块以使用ceres库
# list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
# 
# 
# 寻找Ceres库并添加它的头文件
# find_package( Ceres REQUIRED )
# include_directories( ${CERES_INCLUDE_DIRS} )
# 
# OpenCV
# find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_DIRS} )
# 
# add_executable( curve_fitting ceres_example.cpp )
# 与Ceres和OpenCV链接
# target_link_libraries( curve_fitting 
#          ${CERES_LIBRARIES} 
#          ${OpenCV_LIBS} 
#          ${catkin_LIBRARIES})
#         
#         
# add_executable( ceofficient_calibration ceofficient_calibration.cpp )
# 与Ceres和OpenCV链接
# target_link_libraries( ceofficient_calibration velodyne_rawdata cloud_nodelet
#          ${CERES_LIBRARIES} 
#          ${OpenCV_LIBS} 
#          ${catkin_LIBRARIES}
#          ${catkin_LIBRARIES} 
#          ${YAML_CPP_LIBRARIES})
# 
# 
# install(TARGETS curve_fitting ceofficient_calibration
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#         
#  
#         
# add_library(calibration_nodelet calibration_nodelet.cc )
# add_dependencies(calibration_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(calibration_nodelet velodyne_rawdata
#                       ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
# install(TARGETS calibration_nodelet
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#         
#         
#         
#         