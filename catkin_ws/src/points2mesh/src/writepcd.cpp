#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(input, cloud);//從ROS型別訊息轉為PCL型別訊息
  pcl::io::savePLYFileBinary("write_pcd_test.ply", cloud);//儲存pcd
  // pcl::PLYWriter writer;
  // writer.write ("write_pcd_test.ply", cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), binary, use_camera);
  // pcl::io::savePLYFileASCII ("write_pcd_test.xyzrgb", cloud);//儲存pcd
}
main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_write");
  ros::NodeHandle nh;
  ros::Subscriber bat_sub = nh.subscribe("/rtabmap/cloud_map", 10, cloudCB);//接收點雲
  ros::spin();
  return 0;
}
