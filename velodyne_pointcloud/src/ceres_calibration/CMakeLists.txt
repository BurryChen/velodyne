

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)


# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( curve_fitting ceres_example.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( curve_fitting 
         ${CERES_LIBRARIES} 
         ${OpenCV_LIBS} 
         ${catkin_LIBRARIES})
        
        
add_executable( calibration_flow calibration_flow.cc calibration_flow.h)
# 与Ceres和OpenCV链接
target_link_libraries( calibration_flow velodyne_rawdata cloud_nodelet
         ${CERES_LIBRARIES} 
         ${OpenCV_LIBS} 
         ${catkin_LIBRARIES}
         ${catkin_LIBRARIES} 
         ${YAML_CPP_LIBRARIES})

# 
install(TARGETS curve_fitting calibration_flow
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
 
        
add_library(calibration_nodelet calibration_nodelet.cc convert.cc)
add_dependencies(calibration_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(calibration_nodelet velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS calibration_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
        
        
        