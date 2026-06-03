/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#pragma once

#include <algorithm>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "parameters.h"

using namespace std;
using namespace Eigen;

// 每个特征观测的质量记录，用于质量筛选、PnP 输入选择和运行时诊断。
struct FeatureObservation {
  cv::Point2f left_px;
  cv::Point2f right_px;
  cv::Point2f cur_px;
  cv::Point3f point_3d;
  float disparity = 0.0f;
  float depth = 0.0f;
  float stereo_lk_error = 0.0f;
  float fb_error = 0.0f;
  float temporal_lk_error = 0.0f;
  float reprojection_error = 0.0f;
  float weight = 1.0f;
  bool valid = true;
  bool pnp_inlier = false;
};

// 每帧质量摘要，用于日志、ROS topic 输出、置信度评估和失败诊断。
struct FrameQualityMetrics {
  int detected_count = 0;
  int stereo_match_count = 0;
  int triangulated_count = 0;
  int temporal_track_count = 0;
  int pnp_input_count = 0;
  int pnp_inlier_count = 0;
  double pnp_inlier_ratio = 0.0;
  double reprojection_rmse = 0.0;
  double mean_disparity = 0.0;
  double mean_depth = 0.0;
  double mean_stereo_lk_error = 0.0;
  double mean_fb_error = 0.0;
  double mean_temporal_lk_error = 0.0;
  double mean_weight = 1.0;
  double vo_confidence = 1.0;
  double imu_visual_rotation_error = 0.0;
  double imu_visual_consistency = 1.0;
  bool keyframe_changed = false;
  bool visual_ok = true;
  bool imu_used = false;
  bool coasting_used = false;
  std::string failure_reason;
};

class Estimator {
public:
  class frame {
  public:
    ros::Time frame_time;
    cv::Mat img;
    vector<cv::Point2f> uv;
    vector<cv::Point3f> xyz;
    vector<FeatureObservation> observations;
    FrameQualityMetrics metrics;
    Matrix3d w_R_c;
    Vector3d w_t_c;
  };

  Estimator();

  void reset();
  void setParameter();
  void readIntrinsicParameter(const vector<string>& calib_file);

  bool inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1 = cv::Mat());
  void inputIMU(const sensor_msgs::ImuConstPtr& imu_msg);

  bool trackFeatureBetweenFrames(const Estimator::frame& key_frame, const cv::Mat& cur_img,
                                 vector<cv::Point3f>& key_pts_3d,
                                 vector<cv::Point2f>& cur_pts_2d,
                                 vector<FeatureObservation>* tracked_observations = nullptr);
  bool estimateTBetweenFrames(vector<cv::Point3f>& key_pts_3d, vector<cv::Point2f>& cur_pts_2d,
                              Matrix3d& R, Vector3d& t,
                              FrameQualityMetrics* metrics = nullptr,
                              vector<FeatureObservation>* observations = nullptr);

  void extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv);
  bool trackFeatureLeftRight(const cv::Mat& _img, const cv::Mat& _img1,
                             vector<cv::Point2f>& left_pts,
                             vector<cv::Point2f>& right_pts,
                             vector<FeatureObservation>* observations = nullptr);
  void generate3dPoints(const vector<cv::Point2f>& left_pts,
                        const vector<cv::Point2f>& right_pts,
                        vector<cv::Point3f>& cur_pts_3d,
                        vector<cv::Point2f>& cur_pts_2d,
                        vector<FeatureObservation>* observations = nullptr);

  bool inBorder(const cv::Point2f& pt, const int& row, const int& col);
  double distance(cv::Point2f pt1, cv::Point2f pt2);

  template <typename Derived>
  void reduceVector(vector<Derived>& v, vector<uchar> status);

  vector<cv::Point2f> undistortedPts(vector<cv::Point2f>& pts, camodocal::CameraPtr cam);

  void triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0,
                        Eigen::Matrix<double, 3, 4>& Pose1,
                        Eigen::Vector2d& point0,
                        Eigen::Vector2d& point1,
                        Eigen::Vector3d& point_3d);

  double reprojectionError(Matrix3d& R, Vector3d& t,
                           cv::Point3f& key_pts_3d,
                           cv::Point2f& cur_pts_2d);

  void updateLatestStates(frame& latest_frame);
  std::string latestQualityText() const;

  frame key_frame;
  frame prev_frame;
  int fail_cnt;
  bool init_finish;

  Matrix3d ric[2];
  Vector3d tic[2];
  Matrix4d Tlr;

  vector<camodocal::CameraPtr> m_camera;

  ros::Time latest_time;
  ros::Time rel_key_time;
  Eigen::Matrix3d c_R_k;
  Eigen::Vector3d c_t_k;

  Eigen::Vector3d latest_P;
  Eigen::Quaterniond latest_Q;
  Eigen::Vector3d latest_rel_P;
  Eigen::Quaterniond latest_rel_Q;
  vector<cv::Point3f> latest_pointcloud;
  FrameQualityMetrics latest_metrics;

  Eigen::Vector3d estimated_velocity;
  Eigen::Vector3d smoothed_position;
  bool velocity_initialized;

  static const int FILTER_WINDOW_SIZE = 9;
  std::vector<Eigen::Vector3d> translation_samples;
  void filterTranslation(Eigen::Vector3d& T);

private:
  struct ImuSample {
    ros::Time stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
  };

  std::deque<ImuSample> imu_buffer;
  mutable std::mutex imu_mutex;

  bool predictIMURotation(const ros::Time& start_time, const ros::Time& end_time,
                          Eigen::Matrix3d& delta_R, double* duration = nullptr);
  void computeObservationWeights(vector<FeatureObservation>& observations);
  void filterByQuality(vector<cv::Point3f>& pts_3d,
                       vector<cv::Point2f>& pts_2d,
                       vector<FeatureObservation>* observations,
                       FrameQualityMetrics* metrics);
  void updateFrameConfidence(FrameQualityMetrics& metrics, int pnp_candidate_count);
};
