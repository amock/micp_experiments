#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <geometry_msgs/PoseArray.h>

namespace rm = rmagine;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avz_pose_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ros::Publisher pose_pub = nh_p.advertise<geometry_msgs::PoseArray>("poses", 1);


    std::vector<rm::Transform> sensor_poses;

    XmlRpc::XmlRpcValue sensor_poses_xml;
    if(nh_p.getParam("sensor_poses", sensor_poses_xml))
    {
        ROS_ASSERT(sensor_poses_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < sensor_poses_xml.size(); ++i) 
        {
            ROS_ASSERT(sensor_poses_xml[i].getType() == XmlRpc::XmlRpcValue::TypeArray);

            if(sensor_poses_xml[i].size() == 6)
            {
                rm::Transform T;
                // x, y, z, roll, pitch, yaw
                T.t.x = static_cast<double>(sensor_poses_xml[i][0]);
                T.t.y = static_cast<double>(sensor_poses_xml[i][1]);
                T.t.z = static_cast<double>(sensor_poses_xml[i][2]);
                rm::EulerAngles e;
                e.roll = static_cast<double>(sensor_poses_xml[i][3]);
                e.pitch = static_cast<double>(sensor_poses_xml[i][4]);
                e.yaw = static_cast<double>(sensor_poses_xml[i][5]);
                T.R = e;
                sensor_poses.push_back(T);
            } else if(sensor_poses_xml[i].size() == 7) {
                // x, y, z, qx, qy, qz, qw
                rm::Transform T;
                T.t.x = static_cast<double>(sensor_poses_xml[i][0]);
                T.t.y = static_cast<double>(sensor_poses_xml[i][1]);
                T.t.z = static_cast<double>(sensor_poses_xml[i][2]);
                T.R.x = static_cast<double>(sensor_poses_xml[i][3]);
                T.R.y = static_cast<double>(sensor_poses_xml[i][4]);
                T.R.z = static_cast<double>(sensor_poses_xml[i][5]);
                T.R.w = static_cast<double>(sensor_poses_xml[i][6]);
                sensor_poses.push_back(T);
            }
        }
    }


    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "map";

    std::cout << "Publishing poses:" << std::endl;
    for(size_t i = 0; i<sensor_poses.size(); i++)
    {
        auto Tsm = sensor_poses[i];
        std::cout << i+1 << ": " << Tsm << std::endl;
        geometry_msgs::Pose pose;
        pose.position.x = Tsm.t.x;
        pose.position.y = Tsm.t.y;
        pose.position.z = Tsm.t.z;
        pose.orientation.x = Tsm.R.x;
        pose.orientation.y = Tsm.R.y;
        pose.orientation.z = Tsm.R.z;
        pose.orientation.w = Tsm.R.w;
        poses.poses.push_back(pose);
    }
    

    ros::Rate r(10);

    while(ros::ok())
    {
        poses.header.stamp = ros::Time::now();
        pose_pub.publish(poses);

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
