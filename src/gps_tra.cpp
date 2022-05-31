#include <ros/ros.h>
#include <Eigen/Core>

#include <sensor_msgs/PointCloud2.h>

//#include <preprocess.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <common_lib.h>
#include <sensor_msgs/Imu.h>
using namespace Eigen;
using namespace std;
nav_msgs::Path tra;
nav_msgs::Path tra_imu;
nav_msgs::Path tra_gps;

sensor_msgs::Imu last_imu_point;
sensor_msgs::Imu first_imu_point;
nav_msgs::Odometry first_gps_point;

geometry_msgs::PoseStamped last_tmp;
geometry_msgs::PoseStamped last_imu_tmp;

Vector3f last_imu_vel;
ros::Publisher tra_pub;
ros::Publisher tra_imu_pub;
ros::Publisher tra_gps_pub;

bool is_first=true;
bool is_imu_first = true;
bool is_gps_first = true;

int j=0;
int k=0;
int n=0; 

Matrix3f rot_gps;
Matrix3f cor_rot;

void imu_cbk(const sensor_msgs::Imu::ConstPtr& msg){
        if(is_imu_first){
            is_imu_first=false;
            last_imu_point = *msg;
            
            last_imu_vel<<0,0,0;
            geometry_msgs::PoseStamped tmp;
            last_imu_tmp = tmp;
            tra_imu.poses.push_back(tmp);
            tra_imu.header.frame_id = "/camera_init";
            // 给gps用
            first_imu_point = *msg;
            Quaternionf q_gps(first_imu_point.orientation.w,
            first_imu_point.orientation.x,
            first_imu_point.orientation.y,
            first_imu_point.orientation.z);
            rot_gps = q_gps.toRotationMatrix();
            AngleAxisf V1(-M_PI/2 , Vector3f(0, 0, 1));
            rot_gps = V1*rot_gps;
            return;
    }
    double dt = (msg->header.stamp - last_imu_point.header.stamp).toSec();
    
    Quaternionf q_0(last_imu_tmp.pose.orientation.w,
    last_imu_tmp.pose.orientation.x,
    last_imu_tmp.pose.orientation.y,
    last_imu_tmp.pose.orientation.z);
    Matrix3f rot = q_0.toRotationMatrix();

    Vector3f acc, vel, omega;
    acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    omega << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    acc = rot*acc;
    acc(2) = acc(2)-9.8;
    rot = rot*Exp(omega,dt);

    vel = last_imu_vel+acc*dt;
    //cout<<"地面速度："<<vel.transpose()<<endl;
    geometry_msgs::PoseStamped tmp;
    tmp.pose.position.x = last_imu_tmp.pose.position.x+last_imu_vel(0)*dt+0.5*acc(0)*dt*dt;
    tmp.pose.position.y = last_imu_tmp.pose.position.y+last_imu_vel(1)*dt+0.5*acc(1)*dt*dt;
    tmp.pose.position.z = last_imu_tmp.pose.position.z+last_imu_vel(2)*dt+0.5*acc(2)*dt*dt;
    Quaternionf q(rot);
    tmp.pose.orientation.x = q.x();
    tmp.pose.orientation.y = q.y();
    tmp.pose.orientation.z = q.z();
    tmp.pose.orientation.w = q.w();
    last_imu_vel = vel;
    last_imu_tmp = tmp;
    last_imu_point = *msg;
    // if (k % 100 == 0){
    //     tra_imu.poses.push_back(tmp);
    //     tra_imu_pub.publish(tra_imu);
    // }
    // k++;
}
void gps_cbk(const nav_msgs::Odometry::ConstPtr& msg){
    if(is_gps_first){
            is_gps_first=false;
            first_gps_point = *msg;
            geometry_msgs::PoseStamped tmp;
            tra_gps.poses.push_back(tmp);
            tra_gps.header.frame_id = "/camera_init";
            return;
    }
    geometry_msgs::PoseStamped tmp;
    double x = msg->pose.pose.position.x - first_gps_point.pose.pose.position.x;
    double y = msg->pose.pose.position.y - first_gps_point.pose.pose.position.y;
    Vector3f pos_0,pos_1;
    pos_0<<x, y, 0;
    pos_1 = rot_gps*pos_0;
    tmp.pose.position.x = pos_1(0);
    tmp.pose.position.y = pos_1(1);
    tmp.pose.position.z = pos_1(2);

    if (n % 100 == 0){
        tra_gps.poses.push_back(tmp);
        tra_gps_pub.publish(tra_gps);
    }
    n++;
}
int main(int argc, char **argv)
{   ros::init(argc, argv, "gps_tra");
    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/imu2", 200000, imu_cbk);
    ros::Subscriber sub_gps = nh.subscribe("gps_info",20000,gps_cbk); 

    tra_imu_pub = nh.advertise<nav_msgs::Path>("/pure_imu_tra",10000);
    tra_gps_pub = nh.advertise<nav_msgs::Path>("/pure_gps_tra",10000);
    cor_rot.setIdentity();
    while(ros::ok()){
        if(fabs(tra_gps.poses.back().pose.position.z)>5){
            PointVector points;
            Vector3f n_v;
            for(int i=0;i<tra_gps.poses.size();i++){
                PointType point;
                point.x = tra_gps.poses.at(i).pose.position.x;
                point.y = tra_gps.poses.at(i).pose.position.y;
                point.z = tra_gps.poses.at(i).pose.position.z;
                points.push_back(point);
            }
            esti_normvector(n_v,points,float(0.9),points.size());
            Vector3f correct_n_v(0,0,1);
            Matrix3f A = Quaternionf::FromTwoVectors(n_v,correct_n_v).toRotationMatrix();
            cor_rot = A*cor_rot;
            for(int i=0;i<tra_gps.poses.size();i++){
                Vector3f p0;
                
                p0<<tra_gps.poses.at(i).pose.position.x, tra_gps.poses.at(i).pose.position.y, tra_gps.poses.at(i).pose.position.z;
                p0 = cor_rot*p0;
                tra_gps.poses.at(i).pose.position.x = p0(0);
                tra_gps.poses.at(i).pose.position.y = p0(1);
                tra_gps.poses.at(i).pose.position.z = p0(2);
            }
        }
        ros::spinOnce();
    }
    return 0;
}
