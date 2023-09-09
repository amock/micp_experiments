#include <ros/ros.h>
#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <robot_localization/navsat_conversions.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>



namespace rm = rmagine;



std::string base_frame;
std::string odom_frame;
bool tf_reversed;


bool first_msg = true;
ros::Time first_stamp;

ros::Time state_stamp;
rm::Transform state;

ros::Time gps_glob_stamp_start;
rm::Vector3d gps_glob_start;
rm::Quaterniond gps_heading_start;

ros::Time gps_stamp_last;
rm::Vector3d gps_last;

rm::Vector3d gps_lin_vel;
double gps_vel = 0.0;
rm::Vector3d gps_cov;


size_t n_gps_messages = 0;
bool gps_heading_found = false;

size_t n_imu_messages = 0;

// imu state
rm::Vector3d imu_pos = {0.0, 0.0, 0.0};
rm::Vector3d imu_lin_vel = {0.0, 0.0, 0.0};
rm::Vector3d imu_lin_acc = {0.0, 0.0, 0.0};
double imu_vel;
size_t n_imu_since_last = 0;



double max_acc = 1.0; // max velocity change per second while accelerating
double max_dec = 2.0;
double max_speed = 50.0 / 3.6;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

ros::Publisher pub_gps_point;

sensor_msgs::Imu last_imu_msg;


rm::Vector3d pos;

rm::Quaterniond imu_heading_start;
rm::Quaterniond imu_orientation;

rm::Transform T_base_imu;


// fuse_mode
// 0: speed from gps, orientation from imu
// 1: speed from gps and imu lin acc (kalman filter), orientation from imu
unsigned int fuse_mode = 0;



Eigen::Matrix2d makeA(double dt)
{
    return (Eigen::Matrix2d() << 1, dt, 0, 1).finished();
}

Eigen::Matrix2d makeQ(double dt)
{
    return (Eigen::Matrix2d() << dt, 0, 0, dt).finished();
}



struct State
{
    double x;
    double Ex;
};

class KalmanFilter1D
{
public:
    KalmanFilter1D()
    {
        m_state.x = 0.0;
        m_state.Ex = 10000.0;
        m_Q = 1000000.0;
    }

    // predict with imu acceleration
    void predict(double dt)
    {
        State state_ = m_state;
        state_.Ex = m_state.Ex + m_Q * dt;

        m_state = state_;
    }

    // correct with gps velocity
    void correct_gps(State gps, double dt)
    {
        State state_ = correct(gps, dt);
        std::cout << "correct " << "(" << m_state.x << ", " << m_state.Ex << ")"  << " with GPS " 
          << "(" << gps.x << ", " << gps.Ex << ", dt = " << dt << ")" << " -> " <<  "(" << state_.x << ", " << state_.Ex << ")" << std::endl;

        m_state = state_;
    }

    void correct_imu(State imu_acc, double dt)
    {
        State imu_vel;
        imu_vel.x = m_state.x + dt * imu_acc.x;
        imu_vel.Ex = m_state.Ex + imu_acc.Ex * dt;

        State state_ = correct(imu_vel, dt);

        // std::cout << "correct " << "(" << m_state.x << ", " << m_state.Ex << ")"  << " with IMU " 
        //     << "(" << imu_vel.x << ", " << imu_vel.Ex << ", dt = " << dt << ")" << " -> " <<  "(" << state_.x << ", " << state_.Ex << ")" << std::endl;

        m_state = state_;
    }


    State state()
    {
        return m_state;
    }


private:

    State correct(State gps, double dt)
    {
        double gps_cov = gps.Ex * dt;

        double tot = gps_cov + m_state.Ex;
        double p_state = gps_cov / tot;
        double p_gps = m_state.Ex / tot;

        State state_;
        state_.x = p_state * m_state.x + p_gps * gps.x;
        state_.Ex = m_state.Ex * p_state;
        return state_;
        m_state = state_;
    }

    State m_state;
    double m_t;
    double m_Q;
};

KalmanFilter1D kf;


geometry_msgs::Point to_ros(rm::Vector3d p)
{
    geometry_msgs::Point p_ros;
    p_ros.x = p.x;
    p_ros.y = p.y;
    p_ros.z = p.z;
    return p_ros;
}

rm::Quaterniond to_rm(geometry_msgs::Quaternion q)
{
    return {q.x, q.y, q.z, q.w};
}

rm::Vector3d to_rm(geometry_msgs::Vector3 v)
{
    return {v.x, v.y, v.z};
}

rm::Transformd to_rm(geometry_msgs::Transform T)
{
    rm::Transformd ret;
    ret.R = to_rm(T.rotation);
    ret.t = to_rm(T.translation);
    return ret;
}

geometry_msgs::Transform to_ros(rm::Transformd T)
{
    geometry_msgs::Transform ret;
    ret.rotation.x = T.R.x;
    ret.rotation.y = T.R.y;
    ret.rotation.z = T.R.z;
    ret.rotation.w = T.R.w;
    ret.translation.x = T.t.x;
    ret.translation.y = T.t.y;
    ret.translation.z = T.t.z;
    return ret;
}

geometry_msgs::Transform to_ros(rm::Transformf T)
{
    geometry_msgs::Transform ret;
    ret.rotation.x = T.R.x;
    ret.rotation.y = T.R.y;
    ret.rotation.z = T.R.z;
    ret.rotation.w = T.R.w;
    ret.translation.x = T.t.x;
    ret.translation.y = T.t.y;
    ret.translation.z = T.t.z;
    return ret;
}


void imuCB(const sensor_msgs::Imu::ConstPtr& imu_msg)
{

    std::cout << std::fixed << std::setprecision(2);
    
    if(first_msg)
    {
        first_stamp = imu_msg->header.stamp;
        first_msg = false;
    }

    if(n_imu_messages == 0)
    {
        pos.x = 0.0;
        pos.y = 0.0;
        pos.z = 0.0;
        imu_heading_start = to_rm(imu_msg->orientation);

        geometry_msgs::TransformStamped tfGeom;
        try {
            tfGeom = tf_buffer->lookupTransform(imu_msg->header.frame_id, base_frame, imu_msg->header.stamp);
        } catch (tf2::TransformException &e) {
            std::cout << "ERROR" << std::endl;
        }

        T_base_imu.R = to_rm(tfGeom.transform.rotation).cast<float>();
        T_base_imu.t = to_rm(tfGeom.transform.translation).cast<float>();
    }

    rm::Quaterniond orient_glob = to_rm(imu_msg->orientation);
    imu_orientation = ~imu_heading_start * orient_glob;

    {
        rm::Vector3d lin_acc = to_rm(imu_msg->linear_acceleration);

        State meas;
        meas.x = lin_acc.x;
        meas.Ex = 1000.0;

        kf.predict(0.01);
        kf.correct_imu(meas, 0.01);
        
        n_imu_since_last++;
    }

    if(n_imu_messages > 0)
    {
        // ros::Duration time_since_start = imu_msg->header.stamp - first_stamp;
        double dt = (imu_msg->header.stamp - last_imu_msg.header.stamp).toSec();
        double dist = 0.0;

        if(fuse_mode == 0)
        {
            dist = gps_vel * dt;
        } else if (fuse_mode == 1) {
            auto state = kf.state();
            dist = state.x * dt;
        }

        rm::Vector3d next_pos_delta = imu_orientation * rm::Vector3d{dist, 0.0, 0.0};

        pos += next_pos_delta;
        
        rm::Transform T_imu_odom;
        T_imu_odom.R = imu_orientation.cast<float>();
        T_imu_odom.t = pos.cast<float>();

        rm::Transform T_base_odom = T_imu_odom * T_base_imu;
        rm::Transform T_odom_base = ~T_base_odom;

        static tf2_ros::TransformBroadcaster br;
        {
            geometry_msgs::TransformStamped transformStamped;

            if(tf_reversed)
            {
                transformStamped.header.stamp = imu_msg->header.stamp;
                transformStamped.header.frame_id = base_frame;
                transformStamped.child_frame_id = odom_frame;
                transformStamped.transform = to_ros(T_odom_base);
            } else {
                transformStamped.header.stamp = imu_msg->header.stamp;
                transformStamped.header.frame_id = odom_frame;
                transformStamped.child_frame_id = base_frame;
                transformStamped.transform = to_ros(T_base_odom);
            }
            
            br.sendTransform(transformStamped);
        }
        
        geometry_msgs::PointStamped pos_point;
        pos_point.header.frame_id = "gt";
        pos_point.header.stamp = imu_msg->header.stamp;
        pos_point.point = to_ros(pos);

        pub_gps_point.publish(pos_point);
    }
    

    last_imu_msg = *imu_msg;
    n_imu_messages++;
}

void gpsCB(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    if(first_msg == 0)
    {
        first_stamp = gps_msg->header.stamp;
    }

    ros::Duration time_since_start = gps_msg->header.stamp - first_stamp;

    rm::Vector3d gps_glob_current;
    gps_glob_current.z = gps_msg->altitude;
    robot_localization::navsat_conversions::UTM(
        gps_msg->latitude, gps_msg->longitude, 
        &gps_glob_current.x, &gps_glob_current.y);

    if(n_gps_messages == 0)
    {
        gps_glob_stamp_start = gps_msg->header.stamp;
        gps_glob_start = gps_glob_current;
    }

    rm::Vector3d gps = gps_glob_current - gps_glob_start;

    
    if(n_gps_messages > 0)
    {
        rm::Vector3d gps_local = gps - gps_last;
        double gps_dt = (gps_msg->header.stamp - gps_stamp_last).toSec();
        gps_lin_vel = gps_local / gps_dt;

        double gps_vel_old = gps_vel;
        gps_vel = gps_lin_vel.l2norm();
        
        if( gps_vel - gps_vel_old > max_acc )
        {
            gps_vel = gps_vel_old + max_acc;
        }

        if( gps_vel - gps_vel_old < -max_dec )
        {
            gps_vel = gps_vel_old - max_dec;
        }

        if( gps_vel > max_speed)
        {
            gps_vel = max_speed;
        }

        if( gps_vel < -max_speed )
        {
            gps_vel = -max_speed;
        }

        
        if(fuse_mode == 1)
        {
            double gps_vel_cov = gps_cov.l2norm();

            if(gps_vel_cov > 1500.0)
            {
                std::cout << "!!!!!!!!!!!" << std::endl;
            }

            State meas;
            meas.x = gps_vel;
            meas.Ex = gps_vel_cov;

            kf.correct_gps(meas, gps_dt);

            n_imu_since_last = 0;
        }
    }

    gps_last = gps;
    gps_stamp_last = gps_msg->header.stamp;
    n_gps_messages++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fuse_gps_imu");
    std::cout << "fuse_gps_imu" << std::endl;
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    nh_p.param<bool>("tf_reversed", tf_reversed, false);
    nh_p.param<std::string>("base_frame", base_frame, "base_link");
    nh_p.param<std::string>("odom_frame", odom_frame, "odom_dummy");

    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    state = rm::Transform::Identity();

    auto imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data", 1000, imuCB);
    auto gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("gps/fix", 1000, gpsCB);

    pub_gps_point = nh_p.advertise<geometry_msgs::PointStamped>("gps_point", 1);

    ros::spin();

    return 0;
}