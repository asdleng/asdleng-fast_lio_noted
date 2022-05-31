#include <ros/ros.h>
#include <Eigen/Core>

#include <sensor_msgs/PointCloud2.h>

//#include <preprocess.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
std::string chassis_topic;
std::string raw_lidar_topic;
std::string lidar_topic;
ros::Publisher pub_dyna;
ros::Publisher  pub_lidar;
// 雷神的数据结构
 struct PointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (uint16_t, ring, ring)
                                          (double, time, time)
                                          )
// velodyne数据结构
namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    float time;                     // 时间
    uint16_t ring;                  // 点所属的圈数
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace velodyne_ros

// 注册velodyne_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))
bool first_frame=true;
double last_frame_time;
void lidar_cbk(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    if(first_frame){
        last_frame_time = laserCloudMsg->header.stamp.toSec()*1e6;
        first_frame=false;
        return;
    }
    pcl::PointCloud<PointXYZIRT>::Ptr ls_pl=boost::make_shared <pcl::PointCloud<PointXYZIRT>> ();
    pcl::PointCloud<velodyne_ros::Point> velodyne_pl;
    sensor_msgs::PointCloud2 out_pl;
    pcl::fromROSMsg(*laserCloudMsg, *ls_pl);
        std::sort(ls_pl->begin(), ls_pl->end(),
 [](PointXYZIRT pt1, PointXYZIRT pt2) {return pt1.time < pt2.time; });
    double head_time = ls_pl->begin()->time*1e6;
    double end_time = ls_pl->at(ls_pl->size()-1).time*1e6;
    double diff_time = end_time-head_time;
    double real_diff_time = laserCloudMsg->header.stamp.toSec()*1e6-last_frame_time;
    // ROS_WARN("%f",diff_time);
    // ROS_WARN("%f",real_diff_time);
        velodyne_pl.clear();
    for(double i=0;i<ls_pl->size();i++){
        velodyne_ros::Point p;
        p.x = ls_pl->at(i).x;
        p.y = ls_pl->at(i).y;
        p.z = ls_pl->at(i).z;
        double pro = (ls_pl->points[i].time*1e6-head_time)/diff_time;
        p.time = pro*real_diff_time;

        // ROS_WARN("%f", last_frame_time);
        // ROS_WARN("%f", add);
        // ROS_WARN("%f", p.time);

        p.ring = ls_pl->at(i).ring;
        p.intensity = ls_pl->at(i).intensity;
        velodyne_pl.push_back(p);
    }
    last_frame_time =laserCloudMsg->header.stamp.toSec()*1e6;

    // ROS_WARN("%s","===============");
    // ROS_WARN("%f",laserCloudMsg->header.stamp.toSec());
    // ROS_WARN("%f",velodyne_pl.points.begin()->time);
    // ROS_WARN("%f",velodyne_pl.points.at(ls_pl->size()/2-1).time);
    // ROS_WARN("%f",velodyne_pl.points.at(ls_pl->size()-1).time);
    // ROS_WARN("%s","===============");

    pcl::toROSMsg(velodyne_pl,out_pl);
    out_pl.header = laserCloudMsg->header;
    out_pl.header.frame_id="velodyne";
    pub_lidar.publish(out_pl);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert");
    ros::NodeHandle nh;
    nh.param<std::string>("common/raw_lidar_topic", raw_lidar_topic, "/lslidar_point_cloud");
     nh.param<std::string>("common/lidar_topic", lidar_topic, "/point_cloud");
    ros::Subscriber sub_point_cloud = nh.subscribe(raw_lidar_topic,20000,lidar_cbk);
    pub_lidar = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic,20000);
    while(ros::ok){
        ros::spinOnce();
    }
}