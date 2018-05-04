#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

#include <fstream>  
#include <sstream>  
#include <string>  
#include <vector>  


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/calibration.h>

//#define Laser_Num 32

using namespace std;

namespace velodyne_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZDVAIR
  {
    PCL_ADD_POINT4D;                // quad-word XYZ
    double distance;               ///长度
    double vert_angle;             ///天顶角
    double azimuth;                  ///方位角
    double intensity;                ///< laser intensity reading
    int ring;                     ///< laser ring number
    int timestamp;                 ///时间戳 ms
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZDVAIR,
                                  (double, x, x)
                                  (double, y, y)
                                  (double, z, z)
                                  (double, distance, distance)
                                  (double, vert_angle, vert_angle)
                                  (double, azimuth, azimuth)
                                  (double, intensity, intensity)
                                  (int, ring, ring)
				  (int, timestamp, timestamp))

// Shorthand typedefs for point cloud representations
typedef velodyne_pointcloud::PointXYZDVAIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef velodyne_pointcloud::LaserCorrection Vceofficient;

std::map<int, velodyne_pointcloud::LaserCorrection> Laser_corrections;
    

//删除字符串中空格，制表符tab等无效字符  
string Trim(string& str)  
{  
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置  
    str.erase(0,str.find_first_not_of(" \t\r\n"));  
    str.erase(str.find_last_not_of(" \t\r\n") + 1);  

    return str;  
}  

// 代价函数的计算模型
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;    // x,y数据
};


