#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>


sensor_msgs::PointCloud2 cloud;
sensor_msgs::PointCloud2 cloud_tf;
sensor_msgs::PointCloud2 cloud_map;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_pcl (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf_pcl (new pcl::PointCloud<pcl::PointXYZ>());
Eigen::Matrix4f transform;
Eigen::Matrix4f x_correct;
Eigen::Matrix4f z_correct;
float d = 0.265f;
pcl::VoxelGrid<pcl::PointXYZ> vox;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in) {
  pcl_ros::transformPointCloud(x_correct, *cloud_in, cloud);
  pcl_ros::transformPointCloud(z_correct, cloud, cloud);
}

void tfCallback(const nav_msgs::Odometry::ConstPtr& tf_in) {
  std::cout << "got transform" << std::endl;
  float x = tf_in->pose.pose.position.x;
  float y = tf_in->pose.pose.position.y;
  float z = tf_in->pose.pose.position.z;
  float qx = tf_in->pose.pose.orientation.x;
  float qy = tf_in->pose.pose.orientation.y;
  float qz = tf_in->pose.pose.orientation.z;
  float qw = tf_in->pose.pose.orientation.w;
  tf::Matrix3x3 rotation;
  tf::Quaternion quat(qx,qy,qz,qw);
  rotation.setRotation(quat);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      transform(i,j) = rotation[i][j];
    }
  }
  transform(0,3) = x + d*rotation[0][0];
  transform(1,3) = y + d*rotation[1][0];
  transform(2,3) = z;
  transform(3,3) = 1;
  std::cout << "Applying the following transformation:\n";
  for (int i=0; i<4; ++i) {
    std::cout << "[";
    for (int j=0; j<4; ++j) {
      std::cout << std::setw(12) << transform(i,j) << " ";
    }
    std::cout << "]\n";
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_mapping");

  ros::NodeHandle n;
  
  ros::Subscriber cloud_sub = n.subscribe("camera/depth/points", 1000, cloudCallback);
  ros::Subscriber tf_sub = n.subscribe("vesc/odom", 1000, tfCallback);
  ros::Publisher pc_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 1000);

  ros::Rate loop_rate(10);

  x_correct(0,0) = 1;
  x_correct(1,2) = 1;
  x_correct(2,1) = -1;
  x_correct(3,3) = 1;

  z_correct(0,1) = 1;
  z_correct(1,0) = -1;
  z_correct(2,2) = 1;
  z_correct(3,3) = 1;

  vox.setLeafSize(0.01f, 0.01f, 0.01f);
  vox.setMinimumPointsNumberPerVoxel(3);

  while (ros::ok())
  {
    pcl_ros::transformPointCloud(transform, cloud, cloud_tf);

    pcl::fromROSMsg(cloud_tf, *cloud_tf_pcl);
    vox.setInputCloud(cloud_tf_pcl);
    vox.filter(*cloud_tf_pcl);
    *cloud_map_pcl += *cloud_tf_pcl;
    pcl::toROSMsg(*cloud_map_pcl, cloud_map);
    cloud_map.header.frame_id = "base_link";

    pc_pub.publish(cloud_map);
    std::cout << "Map has " << cloud_map_pcl->points.size() << " points.\n";
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

