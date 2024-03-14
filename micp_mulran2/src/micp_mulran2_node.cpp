#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <micp_mulran2/dataset_loading.h>
#include <micp_mulran2/voxel_filter.hpp>
#include <rmw/qos_profiles.h>
#include <robot_localization/navsat_conversions.hpp>

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmcl/correction/O1DnCorrectorEmbree.hpp>
#include <rmcl/math/math_batched.h>
#include <rmcl/math/math.h>
#include <rmcl/util/ros_helper.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <imu_filter_madgwick/imu_filter.h>






using namespace micp_mulran2;
using namespace std::chrono_literals;
namespace rm = rmagine;

bool enable_visualization = true;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr    pub_pcl;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           pub_imu;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr     pub_gps;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_corr;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_normals;


rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr    pub_dataset;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr    pub_model;


ImuFilter imu_filter;

bool first_msg_imu_gps = true;
size_t first_stamp_imu_gps_ns;


bool first_msg = true;
rclcpp::Time first_stamp;

rclcpp::Time state_stamp;
rm::Transform state;

size_t gps_glob_stamp_start_ns;
rm::Vector3d gps_glob_start;
rm::Quaterniond gps_heading_start;

size_t gps_stamp_last_ns;
rm::Vector3d gps_last;

rm::Vector3d gps_lin_vel;
double gps_vel_cov = 100000.0;
double gps_vel = 0.0;
rm::Vector3d gps_cov;


size_t n_gps_messages = 0;
bool gps_heading_found = false;

size_t n_imu_messages = 0;
size_t last_imu_stamp_ns;


std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_static;


size_t fuse_mode = 0;

double max_acc = 1.0; // max velocity change per second while accelerating
double max_dec = 2.0;
double max_speed = 50.0 / 3.6;

sensor_msgs::msg::Imu last_imu_msg;


rm::Vector3d pos;

rm::Quaterniond imu_heading_start;
rm::Quaterniond imu_orientation;

// rm::Transform T_base_imu;

// all the transformations we need:

// my own tf tree
rm::Transformd T_sensor_ouster; // static. set in main function. 
// I observed that sometimes the ouster cloud is not in the actual measurement frame. 
// Therefore, keeping this identity would result in slightly wrong ray origins which leads in rare occasions leads to wrong hits
rm::Transformd T_ouster_base; // static. set in main function
rm::Transformd T_base_odom; // dynamic. estimated by odometry
rm::Transformd T_odom_map; // dynamic. estimated by MICP-L

rm::Transformd T_odom_map_init;


rm::Transformd T_gps_base;
rm::Transformd T_imu_base;


std::string dataset_root;
rm::EmbreeMapPtr mesh;
rmcl::O1DnCorrectorEmbreePtr corr;
rmcl::CorrectionParams corr_params;
rmcl::CorrectionPtr umeyama;
rm::O1DnModel ouster_model;

// MICP-L params
// TODO: more. Ceres, GN
// 0: Umeyama
size_t optimizer = 0;


// 0: P2L
// 1: P2P
size_t metric = 0;

// 0: Closest Point Correspondences
// 1: Raycasting Correspondences
size_t correspondence_type = 0;

int outer_iterations = 20;
int inner_iterations = 1;
bool disable_registration = false;
bool generate_evaluation = false;

double corr_dist_thresh_min = 0.2;
double corr_dist_thresh_max = 2.0;
double corr_dist_thresh = corr_dist_thresh_max;
double min_range = 0.3;
double max_range = 80.0;

double voxel_filter_size = -10.0;


// GLOBAL BUFFER STORAGE
// store valid scan points as mask
rm::Memory<float> scan_ranges;
rm::Memory<unsigned int> scan_mask; 

rm::Memory<rm::Point> dataset_points;
rm::Memory<rm::Point> model_points;
rm::Memory<rm::Vector> model_normals;
rm::Memory<unsigned int> corr_valid;


bool first_cloud = true;
size_t first_cloud_stamp_ns;

bool first_iteration = true;


double eval_stop_time = 10000000.0;
std::string dataset_name;
std::ofstream eval_file;
std::ofstream eval_file_tum;
// std::ofstream eval_file_ouster;
double outlier_dist = 5.0;

size_t delay_ms = 100;

std::string get_optimizer_name()
{
  if(optimizer == 0)
  {
    return "UM"; 
  } else {
    throw std::runtime_error("NOT IMPLEMENTED");
  }
  return "";
}

std::string get_metric_name()
{
  if(metric == 0)
  {
    return "P2L";
  } else if(metric == 1) {
    return "P2P";
  } else {
    throw std::runtime_error("NOT IMPLEMENTED");
  }
  return "";
}

std::string get_correspondences_type_name()
{
  if(correspondence_type == 0)
  {
    return "RC";
  } else if(correspondence_type == 1) {
    return "CP";
  } else {
    throw std::runtime_error("NOT IMPLEMENTED");
  }
  return "";
}

double to_seconds(size_t stamp_ns)
{
  return static_cast<double>(stamp_ns) / (1000.0 * 1000.0 * 1000.0);
}

void publish_dataset(
  const rm::MemoryView<rm::Point>& dataset_points, 
  const rm::MemoryView<unsigned int>& dataset_mask,
  rclcpp::Time stamp)
{
  sensor_msgs::msg::PointCloud msg;
  msg.header.frame_id = "base";
  msg.header.stamp = stamp;

  for(size_t i=0; i<dataset_points.size(); i++)
  {
    if(dataset_mask[i] > 0)
    {
      geometry_msgs::msg::Point32 p;
      p.x = dataset_points[i].x;
      p.y = dataset_points[i].y;
      p.z = dataset_points[i].z;
      msg.points.push_back(p);
    }
  }

  pub_dataset->publish(msg);
}


void publish_model(
  const rm::MemoryView<rm::Point>& model_points, 
  const rm::MemoryView<unsigned int>& model_mask,
  rclcpp::Time stamp)
{
  sensor_msgs::msg::PointCloud msg;
  msg.header.frame_id = "base";
  msg.header.stamp = stamp;

  for(size_t i=0; i<model_points.size(); i++)
  {
    if(model_mask[i] > 0)
    {
      geometry_msgs::msg::Point32 p;
      p.x = model_points[i].x;
      p.y = model_points[i].y;
      p.z = model_points[i].z;
      msg.points.push_back(p);
    }
  }

  pub_model->publish(msg);
}

// this callback handles the odometry (prior estimate) update
void imuCB(size_t stamp_ns, const sensor_msgs::msg::Imu& imu_msg)
{
  if(first_msg_imu_gps)
  {
    first_stamp_imu_gps_ns = stamp_ns;
    first_msg_imu_gps = false;
  }

  rm::Quaterniond orient_glob{imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w};

  if(n_imu_messages == 0)
  {
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.0;

    imu_heading_start = orient_glob;
    imu_filter.setOrientation(imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
  } else {

    double dt = to_seconds(stamp_ns - last_imu_stamp_ns);

    // std::cout << "LIN ACC: " << imu_msg.linear_acceleration.x << ", " << imu_msg.linear_acceleration.y << ", " << imu_msg.linear_acceleration.z << std::endl;
    imu_filter.madgwickAHRSupdateIMU(
                  imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                  imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z, dt);

    imu_filter.getOrientation(orient_glob.w, orient_glob.x, orient_glob.y, orient_glob.z);


    rm::Quaterniond imu_orientation = ~imu_heading_start * orient_glob;

    double dist = gps_vel * dt;

    rm::Vector3d next_pos_delta = imu_orientation * rm::Vector3d{dist, 0.0, 0.0};
    pos = pos + next_pos_delta;

    // strategy
    // - pos is intregrated along current IMU orientation
    // IMU orientation stays absolute

    // actually, T_imu_odom. If it doesnt work check this first
    T_base_odom.R = imu_orientation;
    T_base_odom.t = pos;
  }

  if(enable_visualization)
  {
    geometry_msgs::msg::TransformStamped T;
    T.header.stamp = rclcpp::Time(stamp_ns);
    T.header.frame_id = "odom"; // to
    T.child_frame_id = "base"; // from
    T.transform.translation.x = T_base_odom.t.x;
    T.transform.translation.y = T_base_odom.t.y;
    T.transform.translation.z = T_base_odom.t.z;
    T.transform.rotation.x = T_base_odom.R.x;
    T.transform.rotation.y = T_base_odom.R.y;
    T.transform.rotation.z = T_base_odom.R.z;
    T.transform.rotation.w = T_base_odom.R.w;

    tf_broadcaster->sendTransform(T);

    pub_imu->publish(imu_msg);
  }

  last_imu_stamp_ns = stamp_ns;
  n_imu_messages = n_imu_messages + 1;
}

// this callback is updating the current velocity of the robot
// using GPS measurements
void gpsCB(size_t stamp_ns, const sensor_msgs::msg::NavSatFix& gps_msg)
{
  if(first_msg_imu_gps)
  {
    first_stamp_imu_gps_ns = stamp_ns;
    first_msg_imu_gps = false;
  }

  std::cout << "GPS cov (x,y,z): " << gps_msg.position_covariance[0] << ", " << gps_msg.position_covariance[4] << ", " << gps_msg.position_covariance[8] << std::endl;

  rm::Vector3d gps_glob_current;
  gps_glob_current.z = gps_msg.altitude;
  robot_localization::navsat_conversions::UTM(
      gps_msg.latitude, gps_msg.longitude, 
      &gps_glob_current.x, &gps_glob_current.y);

  if(n_gps_messages == 0)
  {
    gps_glob_stamp_start_ns = stamp_ns;
    gps_glob_start = gps_glob_current;
  }

  rm::Vector3d gps = gps_glob_current - gps_glob_start;
  
  if(n_gps_messages > 0)
  {
    rm::Vector3d gps_local = gps - gps_last;
    double gps_dt = to_seconds(stamp_ns - gps_stamp_last_ns);
    rm::Vector3d gps_lin_vel = gps_local / gps_dt;

    // a priori: velocity stays the same (constant velocity). uncertainty rises
    double Q = 1.0;
    gps_vel_cov += Q;

    // a posteriori
    // double gps_vel_old = gps_vel;
    double gps_vel_new = gps_lin_vel.l2norm();
    double gps_vel_cov_new = (gps_msg.position_covariance[0] + gps_msg.position_covariance[4] + gps_msg.position_covariance[9]) / 3.0;

    // if(gps_vel_cov_new > 500.0)
    // {
    //   std::cout << "!!!!!!!!!!!!!!!!!!!! GPS COV: " <<  gps_vel_cov_new << std::endl;
    // }

    if(gps_vel_new - gps_vel > max_acc)
    {
      gps_vel_new = gps_vel + max_acc;
    }
            
    if(gps_vel_new - gps_vel < -max_dec)
    {
      gps_vel_new = gps_vel - max_dec;
    }

    if(gps_vel_new > max_speed)
    {
      gps_vel_new = max_speed;
    }

    if(gps_vel_new < -max_speed)
    {
      gps_vel_new = max_speed;
    }

    bool kf = false;
    if(kf)
    {
      std::cout << "OLD GPS vel: " << gps_vel << ", cov: " << gps_vel_cov << std::endl;

      // update gps_vel
      double w_new = gps_vel_cov / (gps_vel_cov_new + gps_vel_cov); // kalman gain
      double w_old = gps_vel_cov_new / (gps_vel_cov_new + gps_vel_cov);

      std::cout << "w_old: " << w_old << ", w_new: " << w_new << std::endl;
      
      gps_vel = w_new * gps_vel_new + w_old * gps_vel;
      gps_vel_cov = w_old * gps_vel_cov;
      
      std::cout << "GPS vel: " << gps_vel << ", cov: " << gps_vel_cov << std::endl;
    } else {
      gps_vel = gps_vel_new;
    }

    if(fuse_mode == 1)
    {
      throw std::runtime_error("fuse_mode 1 - NOT IMPLEMENTED");
    }
  }

  if(enable_visualization)
  {
    pub_gps->publish(gps_msg);
  }

  gps_last = gps;
  gps_stamp_last_ns = stamp_ns;
  n_gps_messages += 1;
}



void drawCorrespondences(
  rm::Transform Tbm,
  float dist_thresh,
  const rm::MemoryView<rm::Point>& dataset_points,
  const rm::MemoryView<unsigned int>& dataset_mask,
  const rm::MemoryView<rm::Point>& model_points,
  const rm::MemoryView<rm::Vector>& model_normals, 
  const rm::MemoryView<unsigned int>& corr_valid,
  rclcpp::Time stamp)
{
  float corr_scale = 0.05;
  unsigned int step = 1;

  visualization_msgs::msg::Marker corr;
  corr.header.frame_id = "map";
  corr.header.stamp = stamp;
  corr.id = 0;
  corr.type = visualization_msgs::msg::Marker::LINE_LIST;
  corr.action = visualization_msgs::msg::Marker::ADD;
  corr.pose.orientation.w = 1.0;
  corr.scale.x = corr_scale;
  corr.scale.y = corr_scale;
  corr.scale.z = corr_scale;

  std_msgs::msg::ColorRGBA red;
  red.r = 1.0;
  red.g = 0.0;
  red.b = 0.0;
  red.a = 1.0;

  std_msgs::msg::ColorRGBA green;
  green.r = 0.0;
  green.g = 1.0;
  green.b = 0.0;
  green.a = 1.0;


  std_msgs::msg::ColorRGBA blue;
  blue.r = 0.0;
  blue.g = 0.0;
  blue.b = 1.0;
  blue.a = 1.0;

  std_msgs::msg::ColorRGBA black;
  black.r = 0.0;
  black.g = 0.0;
  black.b = 0.0;
  black.a = 1.0;

  for(size_t i=0; i<dataset_points.size(); i += step)
  {
    if(corr_valid[i] > 0 && dataset_mask[i] > 0)
    {
      // transform from base coords to map coords
      rm::Point d = Tbm * dataset_points[i];
      rm::Point m = Tbm * model_points[i];
      rm::Vector n = Tbm.R * model_normals[i];

      const float dist = (d - m).l2norm();
      const float signed_plane_dist = (d - m).dot(n);

      // nearest point on model: m2
      const rm::Vector m2 = d - n * signed_plane_dist;

      geometry_msgs::msg::Point dros;
      geometry_msgs::msg::Point mros;

      dros.x = d.x;
      dros.y = d.y;
      dros.z = d.z;

      if(metric == 0) // p2l
      {
        if(fabs(signed_plane_dist) < dist_thresh)
        {
          mros.x = m2.x;
          mros.y = m2.y;
          mros.z = m2.z;

          corr.points.push_back(dros);
          corr.points.push_back(mros);

          corr.colors.push_back(green);
          corr.colors.push_back(green);
        } else {
          mros.x = m2.x;
          mros.y = m2.y;
          mros.z = m2.z;

          corr.points.push_back(dros);
          corr.points.push_back(mros);

          corr.colors.push_back(black);
          corr.colors.push_back(black);
        }
      } else if(metric == 1) { // p2p
        if(dist < dist_thresh)
        {
          mros.x = m.x;
          mros.y = m.y;
          mros.z = m.z;
          
          corr.points.push_back(dros);
          corr.points.push_back(mros);

          corr.colors.push_back(red);
          corr.colors.push_back(red);
        } else {
          mros.x = m.x;
          mros.y = m.y;
          mros.z = m.z;
          
          corr.points.push_back(dros);
          corr.points.push_back(mros);

          corr.colors.push_back(black);
          corr.colors.push_back(black);
        }
      }
    }
  }

  std::cout << "Draw " << corr.points.size() / 2 << " correspondences ..." << std::endl;
  pub_corr->publish(corr);
}

// MICP-L
void cloudCB(size_t stamp_ns, const sensor_msgs::msg::PointCloud& pcl)
{
  double Tbo_time_error = 0.0;

  if(first_cloud)
  {
    first_cloud_stamp_ns = stamp_ns;
    first_cloud = false;
  }

  size_t n_scan_points = pcl.points.size();

  // size_t buffer_overestimate = 0; // dont do too many mallocs

  if(scan_ranges.size() != n_scan_points)
  {
    // std::cout << "RESIZE GLOBAL BUFFERS" << std::endl;
    scan_ranges.resize(n_scan_points);
    scan_mask.resize(n_scan_points);
  }

  // get model
  // I guess n_scan_points == 65536 for KAIST / MulRan
  ouster_model.dirs.resize(n_scan_points);
  ouster_model.width = pcl.points.size();
  ouster_model.height = 1;

  // transform from laser origin (measurement origin) to sensor frame
  // rm::Transform Tls = rm::Transform::Identity();
  
  // inverse for later use
  const rm::Transform T_ouster_sensor = (~T_sensor_ouster).cast<float>();
  
  ouster_model.orig = {0.0, 0.0, 0.0};

  size_t n_valid = 0;
  for (size_t i = 0; i < n_scan_points; i++)
  {
    rm::Vector P_ouster{pcl.points[i].x, pcl.points[i].y, pcl.points[i].z};

    if(!std::isnan(P_ouster.x) && !std::isnan(P_ouster.y) && !std::isnan(P_ouster.z))
    {
      // ouster cloud is in sensor's base coordinate frame but not in actual measurement (sensor) coordinate frame.
      rm::Vector3 P_sensor = T_ouster_sensor * P_ouster; // transform to sensor coords
      float range_est = P_sensor.l2norm();

      if (range_est > ouster_model.range.min 
          && range_est < ouster_model.range.max)
      {
        if(P_sensor.l2normSquared() > 0.0001)
        {
          ouster_model.dirs[i] = P_sensor.normalize();
          scan_ranges[i] = range_est;
          scan_mask[i] = 1;

          n_valid++;
        } else {
          // dir too short
          scan_ranges[i] = -0.1;
          scan_mask[i] = 0;
        }
      }
      else
      {
        scan_ranges[i] = -0.1;
        scan_mask[i] = 0;
      }
    }
    else
    {
        // TODO: write appropriate things into buffers
        scan_ranges[i] = -0.1;
        scan_mask[i] = 0;
    }
  }

  corr->setModel(ouster_model);
  corr->setInputData(scan_ranges);


  corr_params.max_distance = corr_dist_thresh;
  corr->setParams(corr_params);

  std::cout << n_valid << "/" << n_scan_points << " valid measurements" << std::endl;

  size_t num_registration = inner_iterations * outer_iterations;

  if(dataset_points.size() != n_scan_points)
  {   
    // Resize buffers!
    dataset_points.resize(n_scan_points);
    corr_valid.resize(n_scan_points);
    model_points.resize(n_scan_points);
    model_normals.resize(n_scan_points);
  }

  if(!disable_registration && num_registration > 0)
  {
    

    // std::cout << "Start Registration Iterations (" << iterations << ")" << std::endl;

    rm::StopWatchHR sw;
    double el;

    
    // T_base_map = [MICP] * [EKF]
    rm::Transformd T_base_map = T_odom_map * T_base_odom;

    rm::Transform Tbm = T_base_map.cast<float>();
    rm::Transform Tsb = (T_ouster_base * T_sensor_ouster).cast<float>();

    rmcl::CorrectionPreResults<rm::RAM> res;
    res.ds.resize(1);
    res.ms.resize(1);
    res.Cs.resize(1);
    res.Ncorr.resize(1);


    size_t n_iterations_inner = inner_iterations;
    size_t n_iterations_outer = outer_iterations;

    // sw();
    
    // el = sw();
    // std::cout << "- findRCC: " << el * 1000.0 << " ms" << std::endl;

    std::cout << "MICP:" << std::endl;
    std::cout << "- Tbm Before: " << T_base_map << std::endl;

    for(size_t i = 0; i < n_iterations_outer; i++)
    {
      corr_params.max_distance = corr_dist_thresh;
      corr->setParams(corr_params);
      
      if(correspondence_type == 0)
      {
        corr->findRCC(Tbm, dataset_points, model_points, model_normals, corr_valid);
      } else if(correspondence_type == 1) {
        corr->findCPC(Tbm, dataset_points, model_points, model_normals, corr_valid);
      }
      
      // rm::Transform T_delta_total;
      rm::Memory<rm::Transform> T_delta_total(1);
      T_delta_total[0] = rm::Transform::Identity();

      // sw();
      for(size_t j = 0; j < n_iterations_inner; j++)
      {
        if(optimizer == 0) // FAST UMEYAMA
        {
          if(metric == 0) // Point to Plane
          {
            rmcl::means_covs_p2l_online_batched(
              T_delta_total,
              dataset_points, scan_mask, // from
              model_points, model_normals, // to
              corr_valid,
              corr_dist_thresh,
              res.ds, res.ms, // outputs
              res.Cs, res.Ncorr);
          } else if(metric == 1) { // Point to Point
            rmcl::means_covs_p2p_online_batched(
              T_delta_total,
              dataset_points, scan_mask, // from
              model_points, // to
              corr_valid,
              corr_dist_thresh,
              res.ds, res.ms, // outputs
              res.Cs, res.Ncorr);
          }

          auto Tdeltas = umeyama->correction_from_covs(res);

          // update total delta
          T_delta_total[0] = T_delta_total[0] * Tdeltas[0];
        } else {
          throw std::runtime_error("OPTIMIZER NOT IMPLEMENTED");
        }

        // double ratio_valid_hits = static_cast<double>(res.Ncorr[0]) / static_cast<double>(n_valid);
        // corr_dist_thresh = corr_dist_thresh_max + (corr_dist_thresh_min - corr_dist_thresh_max) * ratio_valid_hits;
        // std::cout << "Setting corr_dist_thresh to " << corr_dist_thresh << std::endl;
      }
      
      Tbm = Tbm * T_delta_total[0];
    }

    T_base_map = Tbm.cast<double>();

    // Update Tom
    T_odom_map = T_base_map * ~T_base_odom;
    // el = sw();
    std::cout << "- Tbm Registered: " << T_base_map << ", valid corr: " << res.Ncorr[0] << std::endl;    
  }
  else
  {
    std::cout << "Registration disabled" << std::endl;
  }

  if (generate_evaluation)
  {
    rm::Transformd T_base_map = T_odom_map * T_base_odom;
    rm::Transformd T_ouster_map = T_base_map * T_ouster_base;

    // // T_b_bold = T_bold_map^-1 * T_b_map
    // rm::Transform Tbm_rel = ~Tbm_start * Tbm;

    // // get last correspondences and determine correspondence error
    // rm::Transform T_base_mesh = ~T_mesh_to_map * Tbm;

    // rm::Memory<rm::Transform> Tbms(1);
    // Tbms[0] = T_base_mesh;

    // rm::Memory<rm::Point> dataset_points;
    // rm::Memory<rm::Point> model_points;
    // rm::Memory<unsigned int> corr_valid;

    // rmcl::CorrectionParams corr_params_eval = corr_params;
    // corr_params_eval.max_distance = outlier_dist;

    // corr->setParams(corr_params_eval);
    // corr->findSPC(Tbms, dataset_points, model_points, corr_valid);
    // corr->setParams(corr_params);

    // std::vector<double> p2ms;

    // double p2m_min = 0.0;
    // double p2m_max = 0.0;
    // double p2m_mean = 0.0;
    // double p2m_median = 0.0;

    // for (size_t i = 0; i < corr_valid.size(); i++)
    // {
    //     if (corr_valid[i] > 0)
    //     {
    //         rm::Point diff = dataset_points[i] - model_points[i];
    //         // mean_point_to_mesh_distance += diff.l2norm();
    //         // n_corr++;
    //         p2ms.push_back(diff.l2norm());
    //     }
    // }

    // size_t n_corr = p2ms.size();

    // if (n_corr > 0)
    // {
    //     std::sort(p2ms.begin(), p2ms.end());

    //     p2m_min = p2ms.front();
    //     p2m_max = p2ms.back();
    //     p2m_median = p2ms[p2ms.size() / 2];

    //     for(auto v : p2ms)
    //     {
    //         p2m_mean += v;
    //     }

    //     p2m_mean /= static_cast<double>(n_corr);
    // }
    // else
    // {
    //     // something wrong
    // }

    // { // print to command line
    //     std::cout << std::endl;
    //     std::cout << "---------------------------------" << std::endl;
    //     std::cout << "Evalutation Statistics:" << std::endl;
    //     std::cout << "- Tbm: " << Tbm << std::endl;
    //     std::cout << "- Tbm_rel: " << Tbm_rel << std::endl;
    //     std::cout << "- Time error: " << Tbo_time_error << " s" << std::endl;
    //     std::cout << "- P2M:" << std::endl;
    //     std::cout << "  - N: " << p2ms.size() << std::endl;
    //     std::cout << "  - Min, max: " << p2m_min << " m, " << p2m_max << " m" << std::endl;
    //     std::cout << "  - Mean: " << p2m_mean << " m" << std::endl;
    //     std::cout << "  - Median: " << p2m_median << " m" << std::endl;
    //     std::cout << "---------------------------------" << std::endl;
    //     std::cout << std::endl;
    // }
    

    // Write everything to a file
    // if (!eval_file.is_open())
    // {
    //   eval_file.open("micp_eval.csv", std::ios::out);
    //   eval_file.precision(std::numeric_limits<double>::max_digits10 + 2);
    //   eval_file << std::fixed;
    //   eval_file << "# Tbm.stamp_ns, Tbm.t.x, Tbm.t.y, Tbm.t.z, Tbm.R.x, Tbm.R.y, Tbm.R.z, Tbm.R.w";
    // }

    // // rclcpp::Time pcl_stamp = pcl->header.stamp;
    // eval_file << T_base_map.t.x << ", " << T_base_map.t.y << ", " << T_base_map.t.z << ", " << T_base_map.R.x << ", " << T_base_map.R.y << ", " << T_base_map.R.z << ", " << T_base_map.R.w << ", " << stamp_ns;
    // // eval_file << ", ";
    // // eval_file << Tbm_rel.t.x << ", " << Tbm_rel.t.y << ", " << Tbm_rel.t.z << ", " << Tbm_rel.R.x << ", " << Tbm_rel.R.y << ", " << Tbm_rel.R.z << ", " << Tbm_rel.R.w << ", " << pcl_stamp.seconds();
    // // eval_file << ", ";
    // // eval_file << Tbo_time_error << ", " << n_corr << ", " << p2m_min << ", " << p2m_max << ", " << p2m_mean << ", " << p2m_median;
    // eval_file << "\n";
  
    if(!eval_file_tum.is_open())
    {
      std::string filename = dataset_name + "-" + std::string("MICP") + "-" + get_correspondences_type_name() + "-" + get_metric_name() + ".tum";
      eval_file_tum.open(filename, std::ios::out);
      eval_file_tum.precision(std::numeric_limits<double>::max_digits10 + 2);
      eval_file_tum << std::fixed;
    }

    eval_file_tum << to_seconds(stamp_ns) << " " << T_base_map.t.x << " " << T_base_map.t.y << " " << T_base_map.t.z << " " << T_base_map.R.x << " " << T_base_map.R.y << " " << T_base_map.R.z << " " << T_base_map.R.w << "\n";
  }

  if(enable_visualization)
  {
    rm::Transformd T_base_map = T_odom_map * T_base_odom;

    // draw most recent correspondences
    // rm::Transform Tsb = (T_ouster_base * T_sensor_ouster).cast<float>();
    rm::Transform Tbm = T_base_map.cast<float>();
    if(correspondence_type == 0)
    {
      corr->findRCC(Tbm, dataset_points, model_points, model_normals, corr_valid);
    } else if(correspondence_type == 1) {
      corr->findCPC(Tbm, dataset_points, model_points, model_normals, corr_valid);
    }
    
    drawCorrespondences(Tbm, corr_dist_thresh, dataset_points, scan_mask, model_points, model_normals, corr_valid, pcl.header.stamp);
    publish_dataset(dataset_points, scan_mask, pcl.header.stamp);
    publish_model(model_points, corr_valid, pcl.header.stamp);

    pub_pcl->publish(pcl);
  
    // broadcast
    geometry_msgs::msg::TransformStamped T;
    T.header.stamp = pcl.header.stamp;
    T.header.frame_id = "map"; // to
    T.child_frame_id = "odom"; // from
    T.transform.translation.x = T_odom_map.t.x;
    T.transform.translation.y = T_odom_map.t.y;
    T.transform.translation.z = T_odom_map.t.z;
    T.transform.rotation.x = T_odom_map.R.x;
    T.transform.rotation.y = T_odom_map.R.y;
    T.transform.rotation.z = T_odom_map.R.z;
    T.transform.rotation.w = T_odom_map.R.w;
    tf_broadcaster->sendTransform(T);
  }
}

void initNode()
{
  rclcpp::NodeOptions options = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

  node = std::make_shared<rclcpp::Node>("micp_mulran2_node", options);
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*node);
  tf_broadcaster_static = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
  pub_pcl = node->create_publisher<sensor_msgs::msg::PointCloud>("cloud", rclcpp::SensorDataQoS());
  pub_imu = node->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
  pub_gps = node->create_publisher<sensor_msgs::msg::NavSatFix>("gps", rclcpp::SensorDataQoS());
  pub_corr = node->create_publisher<visualization_msgs::msg::Marker>("corr", rclcpp::SensorDataQoS());
  pub_normals = node->create_publisher<visualization_msgs::msg::Marker>("normals", rclcpp::SensorDataQoS());

  pub_dataset = node->create_publisher<sensor_msgs::msg::PointCloud>("dataset", rclcpp::SensorDataQoS());
  pub_model = node->create_publisher<sensor_msgs::msg::PointCloud>("model", rclcpp::SensorDataQoS());

  // parameter declarations
}

// has to be called after "initNode"
void initStaticTransforms()
{
  T_sensor_ouster = rm::Transformd::Identity();
  // cant find the transform in the mulran data
  T_sensor_ouster.t = rm::Vector3d{0.0, 0.0, 0.0};
  T_sensor_ouster.R = rm::EulerAnglesd{0.0, 0.0, 0.0};

  // ouster -> base
  T_ouster_base.t = rm::Vector3d{1.704, -0.021, 1.805};
  // T_ouster_base.t = rm::Vector3d{1.7042, -0.021, 1.8047}; // from mulran file player?
  T_ouster_base.R = rm::EulerAnglesd{0.0, 0.0, 3.136};

  // gps -> base
  T_gps_base.t = rm::Vector3d{-0.32, 0.0, 1.7};
  T_gps_base.R = rm::EulerAnglesd{0.0, 0.0, 0.0};
  
  // imu -> base
  T_imu_base.t = rm::Vector3d{-0.07, 0.0, 1.7};
  T_imu_base.R = rm::EulerAnglesd{0.0, 0.0, 0.0};


  rm::Transformd T_base_map_init = rm::Transformd::Identity();
  // // KAIST01
  // T_base_map_init.t = rm::Vector3d{-112.45, 108.0, 19.32};
  // T_base_map_init.R = rm::EulerAnglesd{0.037806, 0.0252877, 2.03837};

  auto initial_guess_params_opt = rmcl::get_parameter(node, "initial_guess");
  if(initial_guess_params_opt)
  {
    std::vector<double> initial_guess_params
            = (*initial_guess_params_opt).as_double_array();

    if (initial_guess_params.size() == 6)
    {
      T_base_map_init.t = rm::Vector3d{
          initial_guess_params[0],
          initial_guess_params[1],
          initial_guess_params[2]};
      T_base_map_init.R = rm::EulerAnglesd{
          initial_guess_params[3],
          initial_guess_params[4],
          initial_guess_params[5]};
    }
    else if (initial_guess_params.size() == 7)
    {
      T_base_map_init.t = rm::Vector3d{
          initial_guess_params[0],
          initial_guess_params[1],
          initial_guess_params[2]};
      T_base_map_init.R = rm::Quaterniond{
          initial_guess_params[3],
          initial_guess_params[4],
          initial_guess_params[5],
          initial_guess_params[6]};
    }
  }


  T_base_odom = rm::Transformd::Identity();
  T_odom_map_init = T_base_map_init * ~T_base_odom;
  T_odom_map = T_odom_map_init;

  if(enable_visualization)
  {
    // broadcast
    geometry_msgs::msg::TransformStamped T;
    T.header.stamp = node->get_clock()->now();
    T.header.frame_id = "base"; // to
    T.child_frame_id = "ouster"; // from
    T.transform.translation.x = T_ouster_base.t.x;
    T.transform.translation.y = T_ouster_base.t.y;
    T.transform.translation.z = T_ouster_base.t.z;
    T.transform.rotation.x = T_ouster_base.R.x;
    T.transform.rotation.y = T_ouster_base.R.y;
    T.transform.rotation.z = T_ouster_base.R.z;
    T.transform.rotation.w = T_ouster_base.R.w;
    tf_broadcaster_static->sendTransform(T);
  }
}

// must be called after initStaticTransforms
// sorry for the code style
void loadParams()
{
  std::string map_file;
  auto map_file_opt = rmcl::get_parameter(node, "map_file");
  if(!map_file_opt)
  {
    std::cout << "ERROR: No mesh map found in parameter set. Please set the parameter 'map_file' before running this node!" << std::endl;
    throw std::runtime_error("MAP PATH WRONG OR NOT EXISTING");
  } else {
    map_file = (*map_file_opt).as_string();
  }

  auto dataset_root_opt = rmcl::get_parameter(node, "dataset_root");
  if(!dataset_root_opt)
  {
    std::cout << "ERROR: No mulran data root specified!" << std::endl;
    throw std::runtime_error(" No mulran data root specified!");
  } else {
    dataset_root = (*dataset_root_opt).as_string();
  }

  auto dataset_name_opt = rmcl::get_parameter(node, "dataset_name");
  if(!dataset_name_opt)
  {
    std::cout << "ERROR: No mulran dataset name specified!" << std::endl;
    throw std::runtime_error(" No mulran dataset name specified!");
  } else {
    dataset_name = (*dataset_name_opt).as_string();
  }

  mesh = rm::import_embree_map(map_file);
  
  corr = std::make_shared<rmcl::O1DnCorrectorEmbree>(mesh);
  umeyama = std::make_shared<rmcl::Correction>();
  
  outer_iterations = rmcl::get_parameter<int>(node, "outer_iterations", 20);
  inner_iterations = rmcl::get_parameter<int>(node, "inner_iterations", 1);
  disable_registration = rmcl::get_parameter<bool>(node, "disable_registration", false);
  
  corr_dist_thresh_min = rmcl::get_parameter<double>(node, "corr_dist_thresh_min", 0.2);
  corr_dist_thresh_max = rmcl::get_parameter<double>(node, "corr_dist_thresh_max", 0.5);
  corr_dist_thresh = corr_dist_thresh_max;

  voxel_filter_size = rmcl::get_parameter<double>(node, "voxel_filter_size", -10.0);

  min_range = rmcl::get_parameter<double>(node, "min_range", 0.3);
  max_range = rmcl::get_parameter<double>(node, "max_range", 80.0);
  

  ouster_model.range.min = min_range;
  ouster_model.range.max = max_range;


  

  const rm::Transformd T_sensor_base = T_ouster_base * T_sensor_ouster;
  corr->setTsb(T_sensor_base.cast<float>());

  generate_evaluation = rmcl::get_parameter<bool>(node, "generate_evaluation", false);
  eval_stop_time = rmcl::get_parameter<double>(node, "eval_stop_time", 10000000.0);
  outlier_dist = rmcl::get_parameter<double>(node, "outlier_dist", 5.0);

  delay_ms = rmcl::get_parameter<int>(node, "delay_ms", 1);

  correspondence_type = rmcl::get_parameter<int>(node, "correspondence_type", 0);
  optimizer = rmcl::get_parameter<int>(node, "optimizer", 0);
  metric = rmcl::get_parameter<int>(node, "metric", 0);

  enable_visualization = rmcl::get_parameter<bool>(node, "enable_visualization", true);

  double madgwick_gain = rmcl::get_parameter<double>(node, "madgwick.gain", 0.1);
  double madgwick_zeta = rmcl::get_parameter<double>(node, "madgwick.zeta", 0.0);

  imu_filter.setAlgorithmGain(madgwick_gain);
  imu_filter.setDriftBiasGain(madgwick_zeta);
  imu_filter.setWorldFrame(WorldFrame::ENU);

  // # simple odometry
  // <node pkg="imu_filter_madgwick" type="imu_filter_node" name="imu_filter_node">
  //     <remap from="/imu/data" to="/imu/data"/>
  //     <remap from="/imu/data_raw" to="/imu/data_raw" />

  //     <param name="gain" value="0.1" />
  //     <param name="zeta" value="0.0" />

  //     <param name="use_mag" value="false" />
  //     <param name="remove_gravity_vector" value="false" />

  //     <param name="fixed_frame" value="imu_madgwick" />
  //     <param name="publish_tf" value="true" />
  //     <param name="reverse_tf" value="true" />
  // </node>

}

std::vector<geometry_msgs::msg::Point32> filter_ranges(
  const std::vector<geometry_msgs::msg::Point32>& points, 
  float range_min, float range_max)
{
  std::vector<geometry_msgs::msg::Point32> points_out;
  
  const float range_max_sq = range_max * range_max;
  const float range_min_sq = range_min * range_min;

  for(const auto p : points)
  {
    if(!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
    {
      const float range_sq = p.x * p.x + p.y * p.y + p.z * p.z;
      if(range_sq > range_min_sq && range_sq < range_max_sq)
      {
        points_out.push_back(p);
      }
    }
  }
  return points_out;
}


void printInfo()
{
  std::cout << "SETTINGS:" << std::endl;
  
  std::cout << "- Correspondences: " << get_correspondences_type_name() << std::endl;
  std::cout << "- Optimizer: " << get_optimizer_name() << std::endl;
  std::cout << "- Metric: " << get_metric_name() << std::endl;

  std::cout << "- LiDAR range: " << ouster_model.range.min << " - " << ouster_model.range.max << std::endl;
  
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  

  std::cout << "STARTING MICP MULRAN EVALUATION" << std::endl;

  initNode();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  initStaticTransforms();

  loadParams();

  printInfo();

  // WAIT TO SEE THE INFO
  node->get_clock()->sleep_for(std::chrono::milliseconds(5s));
  executor.spin_some();


  std::optional<size_t> first_stamp;


  MulranDataset ds(dataset_root);



  // for(auto msg = ds.next_message(); msg.first > 0; msg = ds.next_message())
  // {
  //   size_t stamp = msg.first;
  //   if(!first_stamp)
  //   {
  //     first_stamp = stamp;
  //   }
  //   size_t stamp_since_start = stamp - *first_stamp;

  //   // std::cout << "seconds since beginning: " << to_seconds(stamp_since_start) << std::endl;

  //   if(msg.second.type() == typeid(sensor_msgs::msg::NavSatFix))
  //   {
  //     sensor_msgs::msg::NavSatFix gps = std::any_cast<sensor_msgs::msg::NavSatFix>(msg.second);
  //     rm::Vector3d cov = {gps.position_covariance[0], gps.position_covariance[4], gps.position_covariance[8]};
  //     std::cout << to_seconds(stamp_since_start) << ": \t" << cov << std::endl;
  //   }
  // }

  for(auto msg = ds.next_message(); msg.first > 0; msg = ds.next_message())
  {
    size_t stamp = msg.first;
    if(!first_stamp)
    {
      first_stamp = stamp;
    }

    size_t stamp_since_start = stamp - *first_stamp;

    std::cout << "seconds since beginning: " << to_seconds(stamp_since_start) << std::endl;
    if(msg.second.type() == typeid(sensor_msgs::msg::PointCloud))
    {
      std::cout << stamp << ": PCL" << std::endl;

      sensor_msgs::msg::PointCloud pcl = std::any_cast<sensor_msgs::msg::PointCloud>(msg.second);
      pcl.points = filter_ranges(pcl.points, ouster_model.range.min, ouster_model.range.max);

      if(voxel_filter_size >= 0.0)
      {
        VoxelFilter vf(voxel_filter_size);
        vf.insert(pcl.points);
        pcl.points = vf.get_filtered_points(1);
        std::cout << "Reduced to " << pcl.points.size() << " points" << std::endl;
      }

      cloudCB(msg.first, pcl);
      if(to_seconds(msg.first - first_cloud_stamp_ns) > eval_stop_time)
      {
        std::cout << "REACHED EARLY STOPPING" << std::endl;
        break;
      }
    } else if(msg.second.type() == typeid(sensor_msgs::msg::Imu)) {
      std::cout << stamp << ": IMU" << std::endl;
      imuCB(msg.first, std::any_cast<sensor_msgs::msg::Imu>(msg.second));
    } else if(msg.second.type() == typeid(sensor_msgs::msg::NavSatFix)) {
      std::cout << stamp << ": GPS" << std::endl;
      gpsCB(msg.first, std::any_cast<sensor_msgs::msg::NavSatFix>(msg.second));
    }
    node->get_clock()->sleep_for(std::chrono::milliseconds(delay_ms));
    executor.spin_some();
  }

  return 0;
}