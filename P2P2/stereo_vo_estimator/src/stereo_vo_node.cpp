#include "stereo_vo.h"

#include <cmath>

namespace stereo_vo {

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header       = img_msg->header;
    img.height       = img_msg->height;
    img.width        = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step         = img_msg->step;
    img.data         = img_msg->data;
    img.encoding     = "mono8";
    ptr              = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else {
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  }

  return ptr->image.clone();
}

void stereo_vo::imageCallback(const sensor_msgs::ImageConstPtr& img0,
                              const sensor_msgs::ImageConstPtr& img1) {
  if ((img0->header.stamp - image_time_).toSec() < 1.0 / MAX_FREQ) {
    return;
  }

  ros::Time tic = ros::Time::now();
  image_time_ = img0->header.stamp;

  // 处理双目图像并更新估计器状态。
  estimator.inputImage(image_time_, getImageFromMsg(img0), getImageFromMsg(img1));

  // 每帧发布最新状态，便于 RViz 持续显示轨迹和点云。
  pub_odometry();
  pub_path();
  pub_pointcloud();
  pub_quality();

  std::cout << "image process time: " << (ros::Time::now() - tic).toSec() << std::endl;
}

void stereo_vo::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  estimator.inputIMU(imu_msg);
}

void stereo_vo::pub_odometry() {
  nav_msgs::Odometry odometry;
  odometry.header.stamp    = estimator.latest_time;
  odometry.header.frame_id = "world";
  odometry.child_frame_id  = "world";

  odometry.pose.pose.position.x = estimator.latest_P.x();
  odometry.pose.pose.position.y = estimator.latest_P.y();
  odometry.pose.pose.position.z = estimator.latest_P.z();

  odometry.pose.pose.orientation.x = estimator.latest_Q.x();
  odometry.pose.pose.orientation.y = estimator.latest_Q.y();
  odometry.pose.pose.orientation.z = estimator.latest_Q.z();
  odometry.pose.pose.orientation.w = estimator.latest_Q.w();

  odom_pub_.publish(odometry);

  // T_wc = T_wb * T_bc，发布相机坐标系在世界坐标系下的位姿。
  Eigen::Vector3d t_wc = estimator.latest_P + estimator.latest_Q.toRotationMatrix() * TIC[0];
  Eigen::Quaterniond q_wc = estimator.latest_Q * Eigen::Quaterniond(RIC[0]);
  q_wc.normalize();

  geometry_msgs::PoseStamped cam_pose;
  cam_pose.header = odometry.header;
  cam_pose.pose.position.x = t_wc.x();
  cam_pose.pose.position.y = t_wc.y();
  cam_pose.pose.position.z = t_wc.z();
  cam_pose.pose.orientation.x = q_wc.x();
  cam_pose.pose.orientation.y = q_wc.y();
  cam_pose.pose.orientation.z = q_wc.z();
  cam_pose.pose.orientation.w = q_wc.w();

  cam_pose_pub_.publish(cam_pose);

  relative_pose rel_pose;
  rel_pose.header.stamp = estimator.latest_time;
  rel_pose.header.frame_id = "world";
  rel_pose.key_stamp = estimator.rel_key_time;

  Eigen::Quaterniond rel_Q = estimator.latest_rel_Q;
  Eigen::Vector3d rel_t = estimator.latest_rel_P;

  rel_pose.relative_pose.position.x = rel_t.x();
  rel_pose.relative_pose.position.y = rel_t.y();
  rel_pose.relative_pose.position.z = rel_t.z();

  rel_pose.relative_pose.orientation.w = rel_Q.w();
  rel_pose.relative_pose.orientation.x = rel_Q.x();
  rel_pose.relative_pose.orientation.y = rel_Q.y();
  rel_pose.relative_pose.orientation.z = rel_Q.z();

  rel_pose_pub_.publish(rel_pose);

  geometry_msgs::PoseStamped rel_pose_vis;
  rel_pose_vis.header = rel_pose.header;
  rel_pose_vis.pose = rel_pose.relative_pose;
  rel_pose_vis_pub_.publish(rel_pose_vis);
}

void stereo_vo::pub_path() {
  path_.header.stamp = estimator.latest_time;
  path_.header.frame_id = "world";

  geometry_msgs::PoseStamped path_pose;
  path_pose.header.stamp = estimator.latest_time;
  path_pose.header.frame_id = "world";
  path_pose.pose.position.x = estimator.latest_P.x();
  path_pose.pose.position.y = estimator.latest_P.y();
  path_pose.pose.position.z = estimator.latest_P.z();
  path_pose.pose.orientation.x = estimator.latest_Q.x();
  path_pose.pose.orientation.y = estimator.latest_Q.y();
  path_pose.pose.orientation.z = estimator.latest_Q.z();
  path_pose.pose.orientation.w = estimator.latest_Q.w();

  if (path_.poses.empty()) {
    path_.poses.push_back(path_pose);
  } else {
    const auto& last_pose = path_.poses.back();
    double dx = path_pose.pose.position.x - last_pose.pose.position.x;
    double dy = path_pose.pose.position.y - last_pose.pose.position.y;
    double dz = path_pose.pose.position.z - last_pose.pose.position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double time_diff = (path_pose.header.stamp - last_pose.header.stamp).toSec();

    if (dist > 0.01 || time_diff > 0.1) {
      path_.poses.push_back(path_pose);
    } else {
      path_.poses.back() = path_pose;
    }
  }

  if (path_.poses.size() > 1000) {
    path_.poses.erase(path_.poses.begin());
  }

  path_pub_.publish(path_);
}

void stereo_vo::pub_pointcloud() {
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Matrix3d R_wb = estimator.latest_Q.toRotationMatrix();
  Eigen::Vector3d t_wb = estimator.latest_P;
  Eigen::Matrix3d R_bc = RIC[0];
  Eigen::Vector3d t_bc = TIC[0];

  // 当前点云在相机坐标系下，发布前转换到世界坐标系。
  Eigen::Matrix3d R_wc = R_wb * R_bc;
  Eigen::Vector3d t_wc = R_wb * t_bc + t_wb;

  for (const auto& p : estimator.latest_pointcloud) {
    if (p.z < 0.2 || p.z > 20.0) {
      continue;
    }

    Eigen::Vector3d pt_cam(p.x, p.y, p.z);
    Eigen::Vector3d pt_w = R_wc * pt_cam + t_wc;

    pcl::PointXYZ pt;
    pt.x = pt_w.x();
    pt.y = pt_w.y();
    pt.z = pt_w.z();
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = estimator.latest_time;
  cloud_msg.header.frame_id = "world";

  point_cloud_pub_.publish(cloud_msg);
}

void stereo_vo::pub_quality() {
  std_msgs::Float32 confidence_msg;
  confidence_msg.data = static_cast<float>(estimator.latest_metrics.vo_confidence);
  vo_confidence_pub_.publish(confidence_msg);

  std_msgs::String quality_msg;
  quality_msg.data = estimator.latestQualityText();
  frame_quality_text_pub_.publish(quality_msg);
}

}  // namespace stereo_vo

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_vo_node");
  ros::NodeHandle n("~");

  if (argc != 2) {
    printf("please input: rosrun stereo_vo stereo_vo [config file]\n"
           "for example: rosrun stereo_vo stereo_vo "
           "src/stereo_vo_estimator/config/realsense_1/realsense_n3_unsync.yaml\n");
    return 1;
  }

  string config_file = argv[1];
  std::cout << "config file: " << config_file << std::endl;

  readParameters(config_file);

  stereo_vo::stereo_vo vo(n);
  vo.estimator.setParameter();

  ros::spin();
  return 0;
}
