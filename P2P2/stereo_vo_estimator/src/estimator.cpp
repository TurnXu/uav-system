#include "estimator.h"

#include <numeric>
#include <sstream>
#include <cmath>

namespace {

// Logging helper for MVP metrics. It is intentionally side-effect free.
template <typename Getter>
double meanObservationValue(const vector<FeatureObservation>& observations, Getter getter) {
  if (observations.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (const auto& observation : observations) {
    sum += getter(observation);
  }
  return sum / static_cast<double>(observations.size());
}

void logFrameQuality(const FrameQualityMetrics& metrics) {
  ROS_INFO_THROTTLE(
      2.0,
      "VO quality: detect=%d stereo=%d tri=%d temporal=%d pnp=%d/%d ratio=%.2f rmse=%.4f "
      "disp=%.2f depth=%.2f lk(stereo/temp)=%.2f/%.2f fb=%.2f weight=%.2f conf=%.2f "
      "imu=%d coast=%d key=%d fail=%s",
      metrics.detected_count,
      metrics.stereo_match_count,
      metrics.triangulated_count,
      metrics.temporal_track_count,
      metrics.pnp_inlier_count,
      metrics.pnp_input_count,
      metrics.pnp_inlier_ratio,
      metrics.reprojection_rmse,
      metrics.mean_disparity,
      metrics.mean_depth,
      metrics.mean_stereo_lk_error,
      metrics.mean_temporal_lk_error,
      metrics.mean_fb_error,
      metrics.mean_weight,
      metrics.vo_confidence,
      metrics.imu_used ? 1 : 0,
      metrics.coasting_used ? 1 : 0,
      metrics.keyframe_changed ? 1 : 0,
      metrics.failure_reason.c_str());
}

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

double errorScore(double error, double sigma) {
  if (sigma <= 1e-6) {
    return error <= 1e-6 ? 1.0 : 0.0;
  }
  return std::exp(-std::max(0.0, error) / sigma);
}

double rotationAngle(const Matrix3d& R) {
  double cos_angle = (R.trace() - 1.0) * 0.5;
  return std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
}

}  // namespace

void drawImage(const cv::Mat& img, const vector<cv::Point2f>& pts, string name) {
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++) {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}

Estimator::Estimator() {
  ROS_INFO("Estimator init begins.");
  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = Eigen::Vector3d(0, 0, 0);
  prev_frame.w_R_c = Eigen::Matrix3d::Identity();
  fail_cnt = 0;
  init_finish = false;

  latest_time = ros::Time(0.0);
  latest_P.setZero();
  latest_Q = Eigen::Quaterniond::Identity();
  latest_rel_P.setZero();
  latest_rel_Q = Eigen::Quaterniond::Identity();
  latest_metrics = FrameQualityMetrics();

  estimated_velocity.setZero();
  smoothed_position.setZero();
  velocity_initialized = false;
}

void Estimator::reset() {
  ROS_ERROR("Lost, reset!");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
  estimated_velocity.setZero();
  velocity_initialized = false;
  latest_metrics = FrameQualityMetrics();
}

void Estimator::setParameter() {
  for (int i = 0; i < 2; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " extrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  readIntrinsicParameter(CAM_NAMES);

  Matrix4d Tl, Tr;
  Tl.setIdentity();
  Tl.block(0, 0, 3, 3) = ric[0];
  Tl.block(0, 3, 3, 1) = tic[0];
  Tr.setIdentity();
  Tr.block(0, 0, 3, 3) = ric[1];
  Tr.block(0, 3, 3, 1) = tic[1];
  Tlr = Tl.inverse() * Tr;
}

void Estimator::readIntrinsicParameter(const vector<string>& calib_file) {
  for (size_t i = 0; i < calib_file.size(); i++) {
    ROS_INFO("reading parameter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera.push_back(camera);
  }
}

void Estimator::inputIMU(const sensor_msgs::ImuConstPtr& imu_msg) {
  if (!USE_IMU || !imu_msg) {
    return;
  }

  ImuSample sample;
  sample.stamp = imu_msg->header.stamp;
  sample.acc = Vector3d(imu_msg->linear_acceleration.x,
                        imu_msg->linear_acceleration.y,
                        imu_msg->linear_acceleration.z);
  sample.gyro = Vector3d(imu_msg->angular_velocity.x,
                         imu_msg->angular_velocity.y,
                         imu_msg->angular_velocity.z);

  std::lock_guard<std::mutex> lock(imu_mutex);
  if (!imu_buffer.empty() && sample.stamp < imu_buffer.back().stamp) {
    imu_buffer.clear();
  }
  imu_buffer.push_back(sample);

  while (imu_buffer.size() > 2000 ||
         (!imu_buffer.empty() && (sample.stamp - imu_buffer.front().stamp).toSec() > 5.0)) {
    imu_buffer.pop_front();
  }
}

bool Estimator::predictIMURotation(const ros::Time& start_time, const ros::Time& end_time,
                                   Eigen::Matrix3d& delta_R, double* duration) {
  delta_R.setIdentity();
  if (!USE_IMU || start_time.toSec() <= 0 || end_time <= start_time) {
    return false;
  }

  double total_dt = (end_time - start_time).toSec();
  if (total_dt <= 0.0 || total_dt > IMU_PREDICTION_MAX_TIME) {
    return false;
  }

  std::deque<ImuSample> samples;
  {
    std::lock_guard<std::mutex> lock(imu_mutex);
    for (const auto& sample : imu_buffer) {
      if (sample.stamp >= start_time && sample.stamp <= end_time) {
        samples.push_back(sample);
      }
    }
  }

  if (samples.size() < 2) {
    return false;
  }

  for (size_t i = 1; i < samples.size(); ++i) {
    double dt = (samples[i].stamp - samples[i - 1].stamp).toSec();
    if (dt <= 0.0 || dt > IMU_MAX_INTERVAL * 5.0) {
      continue;
    }

    Vector3d omega = 0.5 * (samples[i - 1].gyro + samples[i].gyro) * GYRO_WEIGHT;
    double angle = omega.norm() * dt;
    if (angle < 1e-9) {
      continue;
    }

    AngleAxisd aa(angle, omega.normalized());
    delta_R = delta_R * aa.toRotationMatrix();
  }

  if (duration) {
    *duration = total_dt;
  }
  return true;
}

void Estimator::computeObservationWeights(vector<FeatureObservation>& observations) {
  if (!USE_QUALITY_AWARE) {
    for (auto& obs : observations) {
      obs.weight = 1.0f;
    }
    return;
  }

  for (auto& obs : observations) {
    double disparity_score = clamp01(obs.disparity / std::max(1e-6, MIN_DISPARITY_Q));
    double depth_score = obs.depth > 0.0f ? clamp01(MAX_DEPTH_Q / std::max<double>(obs.depth, 1e-6)) : 0.0;
    double stereo_score = errorScore(obs.stereo_lk_error, WEIGHT_STEREO_SIGMA);
    double fb_score = errorScore(obs.fb_error, WEIGHT_FB_SIGMA);
    double temporal_score = errorScore(obs.temporal_lk_error, WEIGHT_TEMP_SIGMA);

    double weight = disparity_score * depth_score * stereo_score * fb_score * temporal_score;
    obs.weight = static_cast<float>(std::max(MIN_QUALITY_WEIGHT, weight));
  }
}

void Estimator::filterByQuality(vector<cv::Point3f>& pts_3d,
                                vector<cv::Point2f>& pts_2d,
                                vector<FeatureObservation>* observations,
                                FrameQualityMetrics* metrics) {
  if (!observations || observations->size() != pts_3d.size() || pts_3d.size() != pts_2d.size()) {
    return;
  }

  computeObservationWeights(*observations);

  vector<int> indices;
  indices.reserve(observations->size());
  for (size_t i = 0; i < observations->size(); ++i) {
    const auto& obs = (*observations)[i];
    bool quality_ok = obs.weight >= MIN_QUALITY_WEIGHT &&
                      obs.disparity >= MIN_DISPARITY_Q &&
                      obs.depth > 0.0f && obs.depth <= MAX_DEPTH_Q &&
                      obs.fb_error <= MAX_FB_ERROR_Q &&
                      obs.stereo_lk_error <= MAX_STEREO_LK_ERROR_Q &&
                      obs.temporal_lk_error <= MAX_TEMPORAL_LK_ERROR_Q;
    if (!USE_QUALITY_FILTERING || quality_ok) {
      indices.push_back(static_cast<int>(i));
    }
  }

  if (USE_WEIGHTED_PNP && !indices.empty()) {
    std::sort(indices.begin(), indices.end(),
              [&](int lhs, int rhs) {
                return (*observations)[lhs].weight > (*observations)[rhs].weight;
              });
    int keep_count = std::max(4, static_cast<int>(std::ceil(indices.size() * WEIGHT_TOP_RATIO)));
    keep_count = std::min<int>(keep_count, indices.size());
    indices.resize(keep_count);
    std::sort(indices.begin(), indices.end());
  }

  vector<cv::Point3f> filtered_3d;
  vector<cv::Point2f> filtered_2d;
  vector<FeatureObservation> filtered_obs;
  filtered_3d.reserve(indices.size());
  filtered_2d.reserve(indices.size());
  filtered_obs.reserve(indices.size());

  for (int idx : indices) {
    filtered_3d.push_back(pts_3d[idx]);
    filtered_2d.push_back(pts_2d[idx]);
    filtered_obs.push_back((*observations)[idx]);
  }

  pts_3d.swap(filtered_3d);
  pts_2d.swap(filtered_2d);
  observations->swap(filtered_obs);

  if (metrics) {
    metrics->pnp_input_count = static_cast<int>(pts_3d.size());
    metrics->mean_weight =
        meanObservationValue(*observations,
                             [](const FeatureObservation& obs) {
                               return static_cast<double>(obs.weight);
                             });
  }
}

void Estimator::updateFrameConfidence(FrameQualityMetrics& metrics, int pnp_candidate_count) {
  double inlier_score = clamp01(metrics.pnp_inlier_ratio);
  double reproj_score = metrics.reprojection_rmse <= 1e-6 ? 1.0 : clamp01(5.0 / metrics.reprojection_rmse);
  double feature_score = clamp01(static_cast<double>(metrics.temporal_track_count) /
                                 std::max(1.0, FEATURE_THRESHOLD));
  double disparity_score = clamp01(metrics.mean_disparity / std::max(1e-6, MIN_DISPARITY_Q * 5.0));
  double imu_score = metrics.imu_visual_consistency;

  if (pnp_candidate_count < MIN_CNT) {
    feature_score *= 0.5;
  }

  metrics.vo_confidence = clamp01(0.30 * inlier_score +
                                  0.25 * reproj_score +
                                  0.20 * feature_score +
                                  0.15 * disparity_score +
                                  0.10 * imu_score);

  if (!metrics.visual_ok && metrics.imu_used) {
    metrics.vo_confidence = std::max(metrics.vo_confidence, 0.25);
  }
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1) {
  if (fail_cnt > 20) {
    reset();
  }
  std::cout << "receive new image===========================" << std::endl;

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img = _img;
  cur_frame.metrics = FrameQualityMetrics();

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
  vector<cv::Point3f> key_pts_3d;

  c_R_k.setIdentity();
  c_t_k.setZero();

  bool track_ok = false;
  bool imu_prediction_ok = false;
  Matrix3d imu_delta_R = Matrix3d::Identity();
  if (prev_frame.frame_time.toSec() > 0) {
    imu_prediction_ok = predictIMURotation(prev_frame.frame_time, time_stamp, imu_delta_R);
  }

  // 已初始化时，先跟踪关键帧 3D 点到当前帧 2D 点，再用 PnP 估计相对位姿。
  if (init_finish) {
    vector<cv::Point2f> tracked_left;
    // Keep tracked observations aligned with key_pts_3d/tracked_left for logging only.
    vector<FeatureObservation> tracked_observations;
    track_ok = trackFeatureBetweenFrames(key_frame, _img, key_pts_3d, tracked_left,
                                         &tracked_observations);
    cur_frame.metrics.temporal_track_count = static_cast<int>(tracked_left.size());
    cur_frame.metrics.mean_temporal_lk_error =
        meanObservationValue(tracked_observations,
                             [](const FeatureObservation& obs) {
                               return static_cast<double>(obs.temporal_lk_error);
                             });

    if (track_ok && !tracked_left.empty()) {
      vector<cv::Point2f> undist_left = undistortedPts(tracked_left, m_camera[0]);
      computeObservationWeights(tracked_observations);
      if (!estimateTBetweenFrames(key_pts_3d, undist_left, c_R_k, c_t_k,
                                  &cur_frame.metrics, &tracked_observations)) {
        track_ok = false;
      }
    }

    if (track_ok && USE_IMU && imu_prediction_ok) {
      Matrix3d rotation_error = imu_delta_R.transpose() * c_R_k;
      double rotation_error_angle = rotationAngle(rotation_error);
      cur_frame.metrics.imu_used = true;
      cur_frame.metrics.imu_visual_rotation_error = rotation_error_angle;
      cur_frame.metrics.imu_visual_consistency = errorScore(rotation_error_angle, 0.35);
      if (rotation_error_angle > 1.0) {
        cur_frame.metrics.failure_reason = "imu_visual_rotation_inconsistent";
        ROS_WARN("IMU/visual rotation inconsistent: %.3f rad", rotation_error_angle);
        // IMU 仅作为一致性诊断，不直接拒绝视觉 PnP 结果，避免轻量 IMU 假设过强。
        cur_frame.metrics.imu_visual_consistency *= 0.5;
      }
    }

    if (track_ok) {
      fail_cnt = 0;
      cur_frame.metrics.visual_ok = true;

      Matrix3d w_R_cur_obs = key_frame.w_R_c * c_R_k;
      Vector3d w_t_cur_obs = key_frame.w_R_c * c_t_k + key_frame.w_t_c;

      double dt = 0.0;
      if (prev_frame.frame_time.toSec() > 0) {
        dt = (time_stamp - prev_frame.frame_time).toSec();
      }

      if (dt > 0 && dt < 0.2) {
        Vector3d w_t_cur_pred;
        if (velocity_initialized) {
          w_t_cur_pred = smoothed_position + estimated_velocity * dt;
        } else {
          w_t_cur_pred = w_t_cur_obs;
        }

        double pred_error = (w_t_cur_obs - w_t_cur_pred).norm();
        if (pred_error > 0.5) {
          ROS_WARN("Outlier detected (err: %.2f m). Using prediction.", pred_error);
          cur_frame.w_t_c = w_t_cur_pred;
          cur_frame.w_R_c = w_R_cur_obs;
        } else {
          const double alpha = 0.4;
          cur_frame.w_t_c = alpha * w_t_cur_obs + (1.0 - alpha) * w_t_cur_pred;
          cur_frame.w_R_c = w_R_cur_obs;

          Vector3d current_vel = (cur_frame.w_t_c - smoothed_position) / dt;
          estimated_velocity = 0.3 * current_vel + 0.7 * estimated_velocity;
          velocity_initialized = true;
        }
      } else {
        cur_frame.w_t_c = w_t_cur_obs;
        cur_frame.w_R_c = w_R_cur_obs;
        estimated_velocity.setZero();
        velocity_initialized = false;
      }

      smoothed_position = cur_frame.w_t_c;
    } else {
      fail_cnt++;
      cur_frame.metrics.visual_ok = false;

      // 视觉短时失败时优先使用 IMU gyro 姿态预测；平移仍保持保守速度模型。
      if (USE_IMU && imu_prediction_ok && fail_cnt < 8) {
        double dt = (time_stamp - prev_frame.frame_time).toSec();
        cur_frame.w_R_c = prev_frame.w_R_c * imu_delta_R;
        if (velocity_initialized && dt > 0 && dt < IMU_PREDICTION_MAX_TIME) {
          cur_frame.w_t_c = smoothed_position + estimated_velocity * dt;
        } else {
          cur_frame.w_t_c = prev_frame.w_t_c;
        }
        smoothed_position = cur_frame.w_t_c;
        cur_frame.metrics.imu_used = true;
        cur_frame.metrics.failure_reason = "visual_failed_imu_prediction";
        ROS_WARN("Tracking lost, using IMU prediction... (cnt: %d)", fail_cnt);
      } else if (velocity_initialized && fail_cnt < 8) {
        double dt = (time_stamp - prev_frame.frame_time).toSec();
        if (dt > 0) {
          cur_frame.w_t_c = smoothed_position + estimated_velocity * dt;
          cur_frame.w_R_c = prev_frame.w_R_c;
          smoothed_position = cur_frame.w_t_c;
          cur_frame.metrics.coasting_used = true;
          cur_frame.metrics.failure_reason = "visual_failed_coasting";
          ROS_WARN("Tracking lost, coasting... (cnt: %d)", fail_cnt);
        } else {
          cur_frame.w_R_c = prev_frame.w_R_c;
          cur_frame.w_t_c = prev_frame.w_t_c;
          cur_frame.metrics.failure_reason = "visual_failed_hold_pose";
        }
      } else {
        cur_frame.w_R_c = prev_frame.w_R_c;
        cur_frame.w_t_c = prev_frame.w_t_c;
        cur_frame.metrics.failure_reason = "visual_failed_hold_pose";
      }
    }
  } else {
    cur_frame.w_R_c = prev_frame.w_R_c;
    cur_frame.w_t_c = prev_frame.w_t_c;
    smoothed_position = prev_frame.w_t_c;
  }

  // 提取当前左目新特征，并通过左右目匹配生成当前帧 3D 点。
  extractNewFeatures(cur_frame.img, left_pts_2d);

  cur_frame.metrics.detected_count = static_cast<int>(left_pts_2d.size());

  if (!_img1.empty() && !left_pts_2d.empty()) {
    vector<cv::Point2f> stereo_left = left_pts_2d;
    // Keep stereo observations aligned with stereo_left/right_pts_2d for logging only.
    vector<FeatureObservation> stereo_observations;
    bool lr_ok = trackFeatureLeftRight(_img, _img1, stereo_left, right_pts_2d,
                                       &stereo_observations);
    cur_frame.metrics.stereo_match_count = static_cast<int>(stereo_left.size());
    cur_frame.metrics.mean_stereo_lk_error =
        meanObservationValue(stereo_observations,
                             [](const FeatureObservation& obs) {
                               return static_cast<double>(obs.stereo_lk_error);
                             });
    cur_frame.metrics.mean_fb_error =
        meanObservationValue(stereo_observations,
                             [](const FeatureObservation& obs) {
                               return static_cast<double>(obs.fb_error);
                             });

    if (lr_ok && !stereo_left.empty() && stereo_left.size() == right_pts_2d.size()) {
      vector<cv::Point2f> left_un = undistortedPts(stereo_left, m_camera[0]);
      vector<cv::Point2f> right_un = undistortedPts(right_pts_2d, m_camera[1]);

      vector<cv::Point3f> cur_pts_3d;
      vector<cv::Point2f> cur_pts_2d = stereo_left;
      generate3dPoints(left_un, right_un, cur_pts_3d, cur_pts_2d,
                       &stereo_observations);

      cur_frame.xyz = cur_pts_3d;
      cur_frame.uv = cur_pts_2d;
      cur_frame.observations = stereo_observations;
      computeObservationWeights(cur_frame.observations);
      cur_frame.metrics.triangulated_count = static_cast<int>(cur_pts_3d.size());
      cur_frame.metrics.mean_disparity =
          meanObservationValue(cur_frame.observations,
                               [](const FeatureObservation& obs) {
                                 return static_cast<double>(obs.disparity);
                               });
      cur_frame.metrics.mean_depth =
          meanObservationValue(cur_frame.observations,
                               [](const FeatureObservation& obs) {
                                 return static_cast<double>(obs.depth);
                               });
      cur_frame.metrics.mean_weight =
          meanObservationValue(cur_frame.observations,
                               [](const FeatureObservation& obs) {
                                 return static_cast<double>(obs.weight);
                               });
    }
  }

  bool change_keyframe = c_t_k.norm() > TRANSLATION_THRESHOLD ||
                         acos(Quaterniond(c_R_k).w()) * 2.0 > ROTATION_THRESHOLD ||
                         key_pts_3d.size() < FEATURE_THRESHOLD ||
                         !init_finish;
  cur_frame.metrics.keyframe_changed = change_keyframe;
  if (change_keyframe) {
    key_frame = cur_frame;
    rel_key_time = key_frame.frame_time;
    ROS_INFO("Change key frame to current frame.");
  }

  prev_frame = cur_frame;
  updateLatestStates(cur_frame);
  updateFrameConfidence(cur_frame.metrics, static_cast<int>(key_pts_3d.size()));
  latest_metrics = cur_frame.metrics;
  logFrameQuality(cur_frame.metrics);
  if (cur_frame.metrics.vo_confidence < CONFIDENCE_WARN_THRESHOLD) {
    ROS_WARN_THROTTLE(1.0, "[VO_QUALITY] low confidence: %s",
                      latestQualityText().c_str());
  }

  init_finish = true;
  return true;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame& keyframe, const cv::Mat& cur_img,
                                          vector<cv::Point3f>& key_pts_3d,
                                          vector<cv::Point2f>& cur_pts_2d,
                                          vector<FeatureObservation>* tracked_observations) {
  key_pts_3d.clear();
  cur_pts_2d.clear();
  if (tracked_observations) {
    tracked_observations->clear();
  }

  if (keyframe.img.empty() || cur_img.empty()) {
    return false;
  }
  if (keyframe.uv.empty() || keyframe.xyz.empty()) {
    return false;
  }

  vector<cv::Point2f> key_pts_2d = keyframe.uv;
  vector<cv::Point2f> cur_pts_tmp;
  vector<uchar> status;
  vector<float> err;

  cv::calcOpticalFlowPyrLK(keyframe.img, cur_img, key_pts_2d, cur_pts_tmp, status, err,
                           cv::Size(21, 21), 3,
                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                           0, 1e-4);

  const int rows = cur_img.rows;
  const int cols = cur_img.cols;
  vector<uchar> status_filtered(status.size(), 0);
  key_pts_3d = keyframe.xyz;
  cur_pts_2d = cur_pts_tmp;
  vector<FeatureObservation> obs_tracked;
  if (tracked_observations) {
    if (keyframe.observations.size() == keyframe.uv.size()) {
      obs_tracked = keyframe.observations;
    } else {
      obs_tracked.resize(keyframe.uv.size());
      for (size_t i = 0; i < keyframe.uv.size() && i < keyframe.xyz.size(); ++i) {
        obs_tracked[i].left_px = keyframe.uv[i];
        obs_tracked[i].point_3d = keyframe.xyz[i];
        obs_tracked[i].depth = keyframe.xyz[i].z;
      }
    }
  }

  for (size_t i = 0; i < cur_pts_tmp.size(); ++i) {
    if (!status[i]) {
      continue;
    }
    if (!inBorder(cur_pts_tmp[i], rows, cols)) {
      status_filtered[i] = 0;
      continue;
    }
    status_filtered[i] = 1;
    if (tracked_observations && i < obs_tracked.size()) {
      obs_tracked[i].cur_px = cur_pts_tmp[i];
      obs_tracked[i].temporal_lk_error = i < err.size() ? err[i] : 0.0f;
    }
  }

  reduceVector<cv::Point3f>(key_pts_3d, status_filtered);
  reduceVector<cv::Point2f>(cur_pts_2d, status_filtered);
  if (tracked_observations) {
    reduceVector<FeatureObservation>(obs_tracked, status_filtered);
    *tracked_observations = obs_tracked;
  }

  bool success = key_pts_3d.size() >= static_cast<size_t>(MIN_CNT);
  return success;
}

bool Estimator::estimateTBetweenFrames(vector<cv::Point3f>& key_pts_3d,
                                       vector<cv::Point2f>& cur_pts_2d,
                                       Matrix3d& R, Vector3d& t,
                                       FrameQualityMetrics* metrics,
                                       vector<FeatureObservation>* observations) {
  if (key_pts_3d.size() < 4 || cur_pts_2d.size() < 4 || key_pts_3d.size() != cur_pts_2d.size()) {
    if (metrics) {
      metrics->failure_reason = "not_enough_points";
      metrics->pnp_input_count = static_cast<int>(key_pts_3d.size());
    }
    return false;
  }

  if (USE_QUALITY_AWARE && observations) {
    filterByQuality(key_pts_3d, cur_pts_2d, observations, metrics);
    if (key_pts_3d.size() < 4) {
      if (metrics) {
        metrics->failure_reason = "not_enough_points";
        metrics->pnp_input_count = static_cast<int>(key_pts_3d.size());
      }
      return false;
    }
  }

  vector<cv::Point3f> obj_pts = key_pts_3d;
  vector<cv::Point2f> img_pts = cur_pts_2d;
  vector<FeatureObservation> obs_pts;
  if (observations && observations->size() == obj_pts.size()) {
    obs_pts = *observations;
  }
  if (metrics) {
    metrics->pnp_input_count = static_cast<int>(obj_pts.size());
  }

  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec, tvec;
  vector<int> inliers;

  bool success = cv::solvePnPRansac(obj_pts, img_pts, cameraMatrix, distCoeffs,
                                    rvec, tvec, false, 100, 2.0, 0.99, inliers,
                                    cv::SOLVEPNP_ITERATIVE);
  if (!success || inliers.size() < 4) {
    if (metrics) {
      metrics->pnp_inlier_count = static_cast<int>(inliers.size());
      metrics->pnp_inlier_ratio =
          obj_pts.empty() ? 0.0 : static_cast<double>(inliers.size()) / static_cast<double>(obj_pts.size());
      metrics->failure_reason = success ? "too_few_inliers" : "pnp_failed";
    }
    return false;
  }
  if (metrics) {
    metrics->pnp_inlier_count = static_cast<int>(inliers.size());
    metrics->pnp_inlier_ratio =
        obj_pts.empty() ? 0.0 : static_cast<double>(inliers.size()) / static_cast<double>(obj_pts.size());
  }

  vector<cv::Point3f> obj_inliers;
  vector<cv::Point2f> img_inliers;
  vector<int> original_inlier_indices;
  obj_inliers.reserve(inliers.size());
  img_inliers.reserve(inliers.size());
  original_inlier_indices.reserve(inliers.size());
  for (size_t i = 0; i < inliers.size(); ++i) {
    int idx = inliers[i];
    if (idx >= 0 && idx < static_cast<int>(obj_pts.size())) {
      obj_inliers.push_back(obj_pts[idx]);
      img_inliers.push_back(img_pts[idx]);
      original_inlier_indices.push_back(idx);
    }
  }

  if (obj_inliers.size() >= 4) {
    cv::solvePnP(obj_inliers, img_inliers, cameraMatrix, distCoeffs, rvec, tvec,
                 true, cv::SOLVEPNP_ITERATIVE);
  }

  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  Eigen::Matrix3d R_pnp;
  Eigen::Vector3d t_pnp;
  cv::cv2eigen(R_cv, R_pnp);
  cv::Mat t_cv = tvec;
  t_pnp = Vector3d(t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2));

  // OpenCV PnP 输出 key -> current 的投影变换，这里转换成关键帧到当前帧的相机运动。
  R = R_pnp.transpose();
  t = -R_pnp.transpose() * t_pnp;

  double total_error = 0.0;
  int valid_count = 0;
  for (size_t i = 0; i < obj_inliers.size(); ++i) {
    Vector3d pt_3d(obj_inliers[i].x, obj_inliers[i].y, obj_inliers[i].z);
    Vector2d pt_2d(img_inliers[i].x, img_inliers[i].y);

    Vector3d pt_proj = R_pnp * pt_3d + t_pnp;
    if (pt_proj(2) <= 0) {
      continue;
    }

    pt_proj = pt_proj / pt_proj(2);
    Vector2d pt_proj_2d(pt_proj(0), pt_proj(1));

    double error = (pt_2d - pt_proj_2d).norm();
    total_error += error * error;
    valid_count++;
    if (!obs_pts.empty() && i < original_inlier_indices.size()) {
      int obs_idx = original_inlier_indices[i];
      if (obs_idx >= 0 && obs_idx < static_cast<int>(obs_pts.size())) {
        obs_pts[obs_idx].pnp_inlier = true;
        obs_pts[obs_idx].reprojection_error = static_cast<float>(error);
      }
    }
  }

  if (valid_count < 4) {
    if (metrics) {
      metrics->failure_reason = "pnp_valid_projection_too_few";
    }
    return false;
  }

  double rmse = sqrt(total_error / valid_count);
  if (metrics) {
    metrics->reprojection_rmse = rmse;
  }
  if (rmse > 5.0) {
    ROS_WARN("Reprojection error too large: %.6f, rejecting pose", rmse);
    if (metrics) {
      metrics->failure_reason = "reprojection_rmse_too_large";
    }
    return false;
  }

  double trans_norm = t.norm();
  double rot_angle = rotationAngle(R);
  if (trans_norm > 2.0 || rot_angle > M_PI / 2.0) {
    ROS_WARN("Relative pose change too large: trans=%.3f, rot=%.3f, rejecting", trans_norm, rot_angle);
    if (metrics) {
      metrics->failure_reason = "pose_jump_too_large";
    }
    return false;
  }

  if (metrics) {
    metrics->failure_reason.clear();
  }
  if (observations && obs_pts.size() == observations->size()) {
    *observations = obs_pts;
  }
  return true;
}

void Estimator::extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv) {
  uv.clear();
  if (img.empty()) {
    return;
  }

  cv::Mat gray;
  if (img.type() == CV_8UC1) {
    gray = img;
  } else {
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  }

  vector<cv::Point2f> pts;
  cv::goodFeaturesToTrack(gray, pts, MAX_CNT, 0.01, MIN_DIST);

  const int rows = gray.rows;
  const int cols = gray.cols;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (inBorder(pts[i], rows, cols)) {
      uv.push_back(pts[i]);
    }
  }
}

bool Estimator::trackFeatureLeftRight(const cv::Mat& _img, const cv::Mat& _img1,
                                      vector<cv::Point2f>& left_pts,
                                      vector<cv::Point2f>& right_pts,
                                      vector<FeatureObservation>* observations) {
  right_pts.clear();
  if (observations) {
    observations->clear();
  }
  if (_img.empty() || _img1.empty() || left_pts.empty()) {
    return false;
  }

  vector<cv::Point2f> right_pred;
  vector<uchar> status;
  vector<float> err;

  cv::calcOpticalFlowPyrLK(_img, _img1, left_pts, right_pred, status, err,
                           cv::Size(21, 21), 3,
                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                           0, 1e-4);

  vector<float> fb_errors(left_pts.size(), 0.0f);
  if (FLOW_BACK) {
    vector<cv::Point2f> left_back;
    vector<uchar> status_back;
    vector<float> err_back;
    cv::calcOpticalFlowPyrLK(_img1, _img, right_pred, left_back, status_back, err_back,
                             cv::Size(21, 21), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             0, 1e-4);
    for (size_t i = 0; i < status.size(); ++i) {
      if (!status[i]) {
        continue;
      }
      fb_errors[i] = status_back[i] ? static_cast<float>(distance(left_pts[i], left_back[i])) : 999.0f;
      if (!status_back[i] || fb_errors[i] > MAX_FB_ERROR_Q) {
        status[i] = 0;
      }
    }
  }

  const int rows = _img1.rows;
  const int cols = _img1.cols;
  for (size_t i = 0; i < right_pred.size(); ++i) {
    if (!status[i]) {
      continue;
    }
    if (!inBorder(right_pred[i], rows, cols)) {
      status[i] = 0;
    }
  }

  vector<FeatureObservation> obs_all;
  if (observations) {
    obs_all.resize(left_pts.size());
    for (size_t i = 0; i < left_pts.size(); ++i) {
      obs_all[i].left_px = left_pts[i];
      obs_all[i].right_px = i < right_pred.size() ? right_pred[i] : cv::Point2f();
      obs_all[i].stereo_lk_error = i < err.size() ? err[i] : 0.0f;
      obs_all[i].fb_error = i < fb_errors.size() ? fb_errors[i] : 0.0f;
      obs_all[i].valid = i < status.size() ? static_cast<bool>(status[i]) : false;
    }
  }

  right_pts = right_pred;
  reduceVector<cv::Point2f>(left_pts, status);
  reduceVector<cv::Point2f>(right_pts, status);
  if (observations) {
    reduceVector<FeatureObservation>(obs_all, status);
    *observations = obs_all;
  }

  bool success = left_pts.size() >= static_cast<size_t>(MIN_CNT);
  return success;
}

void Estimator::generate3dPoints(const vector<cv::Point2f>& left_pts,
                                 const vector<cv::Point2f>& right_pts,
                                 vector<cv::Point3f>& cur_pts_3d,
                                 vector<cv::Point2f>& cur_pts_2d,
                                 vector<FeatureObservation>* observations) {
  Eigen::Matrix<double, 3, 4> P1, P2;

  P1 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;
  P2.block(0, 0, 3, 3) = Tlr.block(0, 0, 3, 3).transpose();
  P2.block(0, 3, 3, 1) = -P2.block(0, 0, 3, 3) * Tlr.block(0, 3, 3, 1);

  vector<uchar> status;

  for (unsigned int i = 0; i < left_pts.size(); ++i) {
    Vector2d pl(left_pts[i].x, left_pts[i].y);
    Vector2d pr(right_pts[i].x, right_pts[i].y);
    Vector3d pt3;
    triangulatePoint(P1, P2, pl, pr, pt3);

    if (pt3[2] > 0) {
      cur_pts_3d.push_back(cv::Point3f(pt3[0], pt3[1], pt3[2]));
      if (observations && i < observations->size()) {
        (*observations)[i].point_3d = cv::Point3f(pt3[0], pt3[1], pt3[2]);
        (*observations)[i].depth = static_cast<float>(pt3[2]);
        (*observations)[i].disparity =
            std::abs((*observations)[i].left_px.x - (*observations)[i].right_px.x);
      }
      status.push_back(1);
    } else {
      if (observations && i < observations->size()) {
        (*observations)[i].valid = false;
      }
      status.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(cur_pts_2d, status);
  if (observations) {
    reduceVector<FeatureObservation>(*observations, status);
  }
}

bool Estimator::inBorder(const cv::Point2f& pt, const int& row, const int& col) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2) {
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

template <typename Derived>
void Estimator::reduceVector(vector<Derived>& v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++) {
    if (status[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

void Estimator::updateLatestStates(frame& latest_frame) {
  latest_time = latest_frame.frame_time;

  Matrix3d w_R_c = latest_frame.w_R_c;
  Vector3d w_t_c = latest_frame.w_t_c;

  Matrix3d R_bc = ric[0];
  Vector3d t_bc = tic[0];

  Matrix3d w_R_b = w_R_c * R_bc.transpose();
  Vector3d w_t_b = w_R_c * (-R_bc.transpose() * t_bc) + w_t_c;

  latest_P = w_t_b;
  latest_Q = Quaterniond(w_R_b);
  latest_Q.normalize();

  Matrix3d w_R_ck = key_frame.w_R_c;
  Vector3d w_t_ck = key_frame.w_t_c;
  Matrix3d w_R_bk = w_R_ck * R_bc.transpose();
  Vector3d w_t_bk = w_R_ck * (-R_bc.transpose() * t_bc) + w_t_ck;

  Matrix3d R_kb = w_R_bk.transpose() * w_R_b;
  Vector3d t_kb = w_R_bk.transpose() * (w_t_b - w_t_bk);

  latest_rel_P = t_kb;
  latest_rel_Q = Quaterniond(R_kb);
  latest_rel_Q.normalize();

  latest_pointcloud = latest_frame.xyz;
}

std::string Estimator::latestQualityText() const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << "confidence=" << latest_metrics.vo_confidence
      << ", detected_count=" << latest_metrics.detected_count
      << ", stereo_match_count=" << latest_metrics.stereo_match_count
      << ", triangulated_count=" << latest_metrics.triangulated_count
      << ", temporal_track_count=" << latest_metrics.temporal_track_count
      << ", pnp_inlier_ratio=" << latest_metrics.pnp_inlier_ratio
      << ", reprojection_rmse=" << latest_metrics.reprojection_rmse
      << ", mean_disparity=" << latest_metrics.mean_disparity
      << ", mean_depth=" << latest_metrics.mean_depth
      << ", imu_used=" << (latest_metrics.imu_used ? 1 : 0)
      << ", coasting_used=" << (latest_metrics.coasting_used ? 1 : 0)
      << ", failure_reason="
      << (latest_metrics.failure_reason.empty() ? "ok" : latest_metrics.failure_reason);
  return oss.str();
}

void Estimator::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0,
                                 Eigen::Matrix<double, 3, 4>& Pose1,
                                 Eigen::Vector2d& point0,
                                 Eigen::Vector2d& point1,
                                 Eigen::Vector3d& point_3d) {
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);

  Eigen::Vector4d triangulated_point;
  triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

double Estimator::reprojectionError(Matrix3d& R, Vector3d& t,
                                    cv::Point3f& key_pts_3d,
                                    cv::Point2f& cur_pts_2d) {
  Vector3d pt1(key_pts_3d.x, key_pts_3d.y, key_pts_3d.z);
  Vector3d pt2 = R * pt1 + t;
  pt2 = pt2 / pt2[2];
  return sqrt(pow(pt2[0] - cur_pts_2d.x, 2) + pow(pt2[1] - cur_pts_2d.y, 2));
}

vector<cv::Point2f> Estimator::undistortedPts(vector<cv::Point2f>& pts, camodocal::CameraPtr cam) {
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++) {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}

void Estimator::filterTranslation(Eigen::Vector3d& T) {
  translation_samples.push_back(T);
  if (translation_samples.size() > FILTER_WINDOW_SIZE) {
    translation_samples.erase(translation_samples.begin());
  }

  if (translation_samples.size() < FILTER_WINDOW_SIZE) {
    return;
  }

  std::vector<double> x_vals, y_vals, z_vals;
  for (const auto& v : translation_samples) {
    x_vals.push_back(v(0));
    y_vals.push_back(v(1));
    z_vals.push_back(v(2));
  }

  std::sort(x_vals.begin(), x_vals.end());
  std::sort(y_vals.begin(), y_vals.end());
  std::sort(z_vals.begin(), z_vals.end());

  T(0) = x_vals[FILTER_WINDOW_SIZE / 2];
  T(1) = y_vals[FILTER_WINDOW_SIZE / 2];
  T(2) = z_vals[FILTER_WINDOW_SIZE / 2];
}
