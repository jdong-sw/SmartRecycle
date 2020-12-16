#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class CloudProcessor
{
protected:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    CloudProcessor()
    {
        sub = nh.subscribe<PointCloud>("camera/depth_registered/points", 1, &CloudProcessor::Callback, this);
        pub = nh.advertise<PointCloud>("camera/processed_cloud", 1);
    }

private:
    void Callback(const PointCloud::ConstPtr &msg)
    {
        pub.publish(msg);
    }
};

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "process_cloud");
    CloudProcessor processor();
    ros::spin();
}