#include <ros/ros.h>
#include <irp_sen_msgs/encoder.h>
#include <sensor_msgs/Imu.h>
#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>


void encoder_cb(const irp_sen_msgs::encoder::ConstPtr& enc_msg)
{
    std::cout << "Got encoder" << std::endl;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    std::cout << "Got imu" << std::endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "compute_odometry");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ros::Subscriber sub_enc = nh.subscribe<irp_sen_msgs::encoder>("encoder_count", 1, encoder_cb);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu/data_raw", 1, imu_cb);

    ros::spin();
    return 0;
}