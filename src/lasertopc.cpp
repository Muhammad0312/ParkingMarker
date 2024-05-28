#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

class LaserToPointCloud
{
    public:
    LaserToPointCloud(){
        ros::NodeHandle nh;

        sub = nh.subscribe("/scan", 10, &LaserToPointCloud::scanCallback, this);

        pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_in", 10);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

        sensor_msgs::PointCloud2 cloud_msg;
        projector.projectLaser(*scan_msg, cloud_msg);

        pub.publish(cloud_msg);

    }
    private:
    ros::Subscriber sub;
    ros::Publisher pub;
    laser_geometry::LaserProjection projector;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_to_pointcloud");

    LaserToPointCloud ltpc;

    ros::spin();

    return 0;
}
