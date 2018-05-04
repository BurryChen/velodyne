
#include "calibration_flow.h"
#include <velodyne_pointcloud/calibration.h>

const int Laser_Num=32;

// 代价函数的计算模型(点云平面约束)
struct PLAN_CONTRAINT_COST
{
    PLAN_CONTRAINT_COST (double distance, double vert_angle, double azimuth, int ring): 
    _distance (distance ),_vert_angle (vert_angle ),_azimuth(azimuth),_ring(ring){}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const estimation,     // 模型参数，5
        T* residual ) const     // 残差
    {      
      T(_distance_sum)=T(_distance)+estimation[0];
      T(_vert_angle_sum)=T(_vert_angle)+estimation[+1];
      T(_azimuth_sum)=T(_azimuth)+estimation[2]; 
      T(_d_h)=estimation[3];
      T(_d_v)=estimation[4];
      
      T(_xy_distance) = T(_distance_sum)* cos(T(_vert_angle_sum))- T(_d_v) * sin(T(_vert_angle_sum));
      // Calculate temporal X, use absolute value.
      T(_x) = T(_xy_distance) * cos(T(_azimuth_sum)) - T(_d_h) * sin(T(_azimuth_sum));
      // Calculate temporal Y, use absolute value
      T(_y) = T(_xy_distance) * sin(T(_azimuth_sum)) + T(_d_h) * cos(T(_azimuth_sum));
      // Calculate temporal Y, use absolute value
      T(_z) = T(_distance_sum) * sin(T(_vert_angle_sum)) + T(_d_v)* cos(T(_vert_angle_sum));    
      residual[0] = T(_x)*T(_g[0])+T(_y)*T(_g[2])+T(_z)*T(_g[3])+T(_g[4]);
    
      return true;
    }
    const double _distance,_vert_angle,_azimuth;;    // 距离,天顶角,方位角 数据
    const int _ring;                                 //laser_id,0-31
    
    double _distance_sum,_vert_angle_sum,_azimuth_sum,_d_h,_d_v,_xy_distance,_x,_y,_z;   //中间变量
    double _g[4]={1,1,1,1};
};


int main ( int argc, char** argv )
{
    VPointCloud pc;
    // 读点云文件 csv 
    cout<<"1 read file."<<endl;
    ifstream inFile("/home/whu/slam_ws/src/calibration_HDL32/velodyne_pointcloud/src/data/2018-02-08-11-28-47_Velodyne-HDL-32-Data.csv");  
    string line;  
    vector<vector<string>> strArray;  
    getline(inFile, line);
    while (getline(inFile, line))  
    {
        //cout <<"原始字符串："<< line << endl; //整行输出 
        istringstream sin(line);            //将整行字符串line读入到字符串流istringstream中  
        vector<string> fields;              //声明一个字符串向量  
        string field;  
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符  
        {  
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中  
        }
        VPoint point;
	point.x = atof(Trim(fields[0]).c_str());     //清除掉向量fields中第一个元素的无效字符，并赋值给x
	point.y = atof(Trim(fields[1]).c_str());     
	point.z = atof(Trim(fields[2]).c_str());     
	point.distance = atof(Trim(fields[9]).c_str());     
	point.vert_angle = atof(Trim(fields[12]).c_str());     
	point.azimuth = atof(Trim(fields[8]).c_str());    
	point.intensity = atof(Trim(fields[6]).c_str());    
	point.ring = atoi(Trim(fields[7]).c_str());     
	point.timestamp = atoi(Trim(fields[10]).c_str());     

        // append this point to the cloud
        pc.push_back(point);
        ++pc.width;
    }  
    cout <<"pointcloud num:"<<pc.width<<endl;
    cout <<"Read File End!"<< endl;
    
    
    //构建约束方程
    double estimation[Laser_Num*5]={0.0};
    // 构建最小二乘问题
    ceres::Problem Vproblem[Laser_Num];                 //32个最小二乘,每根laser单独求解
    for ( int i=0; i<pc.width; i++ )
    {   
        int laser_id=pc[i].ring;
	Vproblem[laser_id].AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<PLAN_CONTRAINT_COST, 1, 5> ( 
                new PLAN_CONTRAINT_COST (pc[i].distance, pc[i].vert_angle, pc[i].azimuth, laser_id )
            ),
            nullptr,            // 核函数，这里不使用，为空
            estimation                 // 待估计参数
        );
    }

    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    for ( int i=0;i<Laser_Num;i++ )
    {
      ceres::Solver::Summary summary;                // 优化信息
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
      ceres::Solve ( options, Vproblem+i, &summary );  // 开始优化
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
      chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
      cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
      // 输出结果
      cout<<summary.BriefReport() <<endl;
      cout<<"estimation= ";
      for ( auto a:estimation ) cout<<a<<" ";
      cout<<endl;
    }

    return 0;
}

