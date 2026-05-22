#include "estimator.h"

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

  // 初始化最新状态
  latest_time = ros::Time(0.0);
  latest_P.setZero();
  latest_Q = Eigen::Quaterniond::Identity();
  latest_rel_P.setZero();
  latest_rel_Q = Eigen::Quaterniond::Identity();

  // [新增] 初始化平滑变量
  estimated_velocity.setZero();
  smoothed_position.setZero();
  velocity_initialized = false;
}

void Estimator::reset() {
  ROS_ERROR("Lost, reset!");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
  // 重置速度
  estimated_velocity.setZero();
  velocity_initialized = false;
}

void Estimator::setParameter() {
  for (int i = 0; i < 2; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " exitrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  readIntrinsicParameter(CAM_NAMES);

  // transform between left and right camera
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
    ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera.push_back(camera);
  }
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1) {

  if (fail_cnt > 20) {
    reset();
  }
  std::cout << "receive new image===========================" << std::endl;

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img        = _img;

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
  vector<cv::Point3f> key_pts_3d;

  c_R_k.setIdentity();
  c_t_k.setZero();

  bool track_ok = false;

  // 1. 若已初始化：关键帧-当前帧 2D-3D 跟踪 + PnP 估计相对位姿
  if (init_finish) {
    vector<cv::Point2f> tracked_left;
    track_ok = trackFeatureBetweenFrames(key_frame, _img, key_pts_3d, tracked_left);

    if (track_ok && !tracked_left.empty()) {
      vector<cv::Point2f> undist_left = undistortedPts(tracked_left, m_camera[0]);
      if (!estimateTBetweenFrames(key_pts_3d, undist_left, c_R_k, c_t_k)) {
        track_ok = false;
      }
    }

    if (track_ok) {
      fail_cnt = 0;

      // =========================================================
      // [改进] 轨迹平滑与预测逻辑
      // =========================================================
      
      // A. 计算观测值 (Observation from PnP)
      // w_T_cur = w_T_key * key_T_cur
      Matrix3d w_R_cur_obs = key_frame.w_R_c * c_R_k;
      Vector3d w_t_cur_obs = key_frame.w_R_c * c_t_k + key_frame.w_t_c;

      double dt = 0.0;
      if (prev_frame.frame_time.toSec() > 0) {
          dt = (time_stamp - prev_frame.frame_time).toSec();
      }

      if (dt > 0 && dt < 0.2) { // 时间间隔正常
          
          // B. 预测值 (Prediction from Motion Model)
          Vector3d w_t_cur_pred;
          if (velocity_initialized) {
              w_t_cur_pred = smoothed_position + estimated_velocity * dt;
          } else {
              w_t_cur_pred = w_t_cur_obs;
          }

          // C. 异常检测 (Outlier Rejection)
          // 如果观测值和预测值偏差太大(例如 > 0.5m)，说明PnP可能算飞了，完全信任预测
          double pred_error = (w_t_cur_obs - w_t_cur_pred).norm();
          
          if (pred_error > 0.5) {
               ROS_WARN("Outlier detected (err: %.2f m). Using prediction.", pred_error);
               cur_frame.w_t_c = w_t_cur_pred;
               // 旋转通常比平移更稳，可以用观测值，或者沿用上一帧
               cur_frame.w_R_c = w_R_cur_obs; 
          } else {
               // D. 融合更新 (Fusion / Smoothing)
               // alpha 越小越平滑，越大响应越快。推荐 0.4
               double alpha = 0.4;
               cur_frame.w_t_c = alpha * w_t_cur_obs + (1.0 - alpha) * w_t_cur_pred;
               cur_frame.w_R_c = w_R_cur_obs;

               // E. 更新速度 (Update Velocity)
               // V = (P_new - P_old) / dt
               Vector3d current_vel = (cur_frame.w_t_c - smoothed_position) / dt;
               
               // 速度低通滤波: vel = 0.3 * curr + 0.7 * old
               estimated_velocity = 0.3 * current_vel + 0.7 * estimated_velocity;
               velocity_initialized = true;
          }

      } else {
          // 第一帧或时间间隔过大，重置
          cur_frame.w_t_c = w_t_cur_obs;
          cur_frame.w_R_c = w_R_cur_obs;
          estimated_velocity.setZero();
          velocity_initialized = false;
      }
      
      // 更新平滑记录
      smoothed_position = cur_frame.w_t_c;

    } else {
      // 跟踪失败
      fail_cnt++;
      
      // [改进] 丢失跟踪时的惯性预测 (Coast)
      // 如果之前有速度，且丢失不久，允许按惯性飞一小段，避免画面卡死
      if (velocity_initialized && fail_cnt < 8) {
          double dt = (time_stamp - prev_frame.frame_time).toSec();
          if (dt > 0) {
              cur_frame.w_t_c = smoothed_position + estimated_velocity * dt;
              cur_frame.w_R_c = prev_frame.w_R_c;
              smoothed_position = cur_frame.w_t_c; // 更新平滑位置以便下一帧继续预测
              ROS_WARN("Tracking lost, coasting... (cnt: %d)", fail_cnt);
          } else {
              cur_frame.w_R_c = prev_frame.w_R_c;
              cur_frame.w_t_c = prev_frame.w_t_c;
          }
      } else {
          cur_frame.w_R_c = prev_frame.w_R_c;
          cur_frame.w_t_c = prev_frame.w_t_c;
      }
    }
  } else {
    // 首帧：直接采用上一帧（初始化设定）的相机位姿
    cur_frame.w_R_c = prev_frame.w_R_c;
    cur_frame.w_t_c = prev_frame.w_t_c;
    smoothed_position = prev_frame.w_t_c; // 初始化平滑位置
  }

  // 2. 为当前帧提取新的左目特征点
  extractNewFeatures(cur_frame.img, left_pts_2d);

  // 3. 双目跟踪 + 三角化，生成当前帧 3D 点
  if (!_img1.empty() && !left_pts_2d.empty()) {
    vector<cv::Point2f> stereo_left = left_pts_2d;
    bool lr_ok = trackFeatureLeftRight(_img, _img1, stereo_left, right_pts_2d);

    if (lr_ok && !stereo_left.empty() && stereo_left.size() == right_pts_2d.size()) {
      // 去畸变左右 2D 点到归一化平面
      vector<cv::Point2f> left_un  = undistortedPts(stereo_left,  m_camera[0]);
      vector<cv::Point2f> right_un = undistortedPts(right_pts_2d, m_camera[1]);

      vector<cv::Point3f> cur_pts_3d;
      vector<cv::Point2f> cur_pts_2d = stereo_left;  // 像素坐标
      generate3dPoints(left_un, right_un, cur_pts_3d, cur_pts_2d);
      
      cur_frame.xyz = cur_pts_3d;  // 当前相机坐标系下的 3D 点
      cur_frame.uv  = cur_pts_2d;  // 左目像素坐标，用于帧间 LK 跟踪
    }
  }

  // 4. 判断是否切换关键帧
  if (c_t_k.norm() > TRANSLATION_THRESHOLD ||
      acos(Quaterniond(c_R_k).w()) * 2.0 > ROTATION_THRESHOLD ||
      key_pts_3d.size() < FEATURE_THRESHOLD ||
      !init_finish) {
    key_frame = cur_frame;
    rel_key_time = key_frame.frame_time;
    ROS_INFO("Change key frame to current frame.");
  }

  // 5. 更新 prev_frame 与 latest_* 状态
  prev_frame = cur_frame;
  updateLatestStates(cur_frame);

  init_finish = true;

  return true;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame& keyframe, const cv::Mat& cur_img,
                                             vector<cv::Point3f>& key_pts_3d,
                                             vector<cv::Point2f>& cur_pts_2d) {

  key_pts_3d.clear();
  cur_pts_2d.clear();

  if (keyframe.img.empty() || cur_img.empty())
    return false;

  if (keyframe.uv.empty() || keyframe.xyz.empty())
    return false;

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

  // 按边界和状态过滤，并同步 3D 点和 2D 点
  vector<uchar> status_filtered(status.size(), 0);
  key_pts_3d = keyframe.xyz;  // 先完整拷贝，再通过 status 过滤
  cur_pts_2d = cur_pts_tmp;

  for (size_t i = 0; i < cur_pts_tmp.size(); ++i) {
    if (!status[i])
      continue;
    if (!inBorder(cur_pts_tmp[i], rows, cols)) {
      status_filtered[i] = 0;
      continue;
    }
    status_filtered[i] = 1;
  }

  reduceVector<cv::Point3f>(key_pts_3d, status_filtered);
  reduceVector<cv::Point2f>(cur_pts_2d, status_filtered);

  bool success = key_pts_3d.size() >= (size_t)MIN_CNT;
  if (!success)
    fail_cnt++;
  else
    fail_cnt = 0;

  return success;
}

bool Estimator::estimateTBetweenFrames(vector<cv::Point3f>& key_pts_3d,
                                          vector<cv::Point2f>& cur_pts_2d, Matrix3d& R, Vector3d& t) {

  if (key_pts_3d.size() < 4 || cur_pts_2d.size() < 4 || key_pts_3d.size() != cur_pts_2d.size())
    return false;

  vector<cv::Point3f> obj_pts = key_pts_3d;
  vector<cv::Point2f> img_pts = cur_pts_2d;

  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec, tvec;
  vector<int> inliers;

  bool success = cv::solvePnPRansac(obj_pts, img_pts, cameraMatrix, distCoeffs,
                                    rvec, tvec, false, 100, 2.0, 0.99, inliers,
                                    cv::SOLVEPNP_ITERATIVE);
  if (!success || inliers.size() < 4) {
    return false;
  }

  // 使用内点再次精细化
  vector<cv::Point3f> obj_inliers;
  vector<cv::Point2f> img_inliers;
  obj_inliers.reserve(inliers.size());
  img_inliers.reserve(inliers.size());
  for (size_t i = 0; i < inliers.size(); ++i) {
    int idx = inliers[i];
    if (idx >= 0 && idx < (int)obj_pts.size()) {
      obj_inliers.push_back(obj_pts[idx]);
      img_inliers.push_back(img_pts[idx]);
    }
  }
  if (obj_inliers.size() >= 4) {
    cv::solvePnP(obj_inliers, img_inliers, cameraMatrix, distCoeffs, rvec, tvec,
                 true, cv::SOLVEPNP_ITERATIVE);
  }

  // rvec 转旋转矩阵
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  // Eigen 形式的 PnP 结果
  Eigen::Matrix3d R_pnp;
  Eigen::Vector3d t_pnp;
  cv::cv2eigen(R_cv, R_pnp);
  cv::Mat t_cv = tvec;
  t_pnp = Vector3d(t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2));

  // =======================================================================
  // [修正] PnP 结果求逆：T_{C_{cur}}^{C_{key}} -> T_{C_{key}}^{C_{cur}}
  // =======================================================================
  R = R_pnp.transpose();
  t = -R_pnp.transpose() * t_pnp;

  // 重投影误差检查，使用原始 PnP 结果验证
  double total_error = 0.0;
  int valid_count = 0;
  for (size_t i = 0; i < obj_inliers.size(); ++i) {
    Vector3d pt_3d(obj_inliers[i].x, obj_inliers[i].y, obj_inliers[i].z);
    Vector2d pt_2d(img_inliers[i].x, img_inliers[i].y);
    
    // 投影到当前帧：p_cur = R_pnp * p_key + t_pnp
    Vector3d pt_proj = R_pnp * pt_3d + t_pnp; 
    if (pt_proj(2) <= 0) continue; // 无效深度
    
    // 归一化到归一化平面
    pt_proj = pt_proj / pt_proj(2);
    Vector2d pt_proj_2d(pt_proj(0), pt_proj(1));
    
    double error = (pt_2d - pt_proj_2d).norm();
    total_error += error * error;
    valid_count++;
  }
  
  if (valid_count < 4) {
    return false;
  }
  
  double rmse = sqrt(total_error / valid_count);
  if (rmse > 5.0) {  
    ROS_WARN("Reprojection error too large: %.6f, rejecting pose", rmse);
    return false;
  }

  // 检查位姿变化的合理性
  double trans_norm = t.norm(); 
  double rot_angle = acos(std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0))); 
  
  if (trans_norm > 2.0 || rot_angle > M_PI / 2.0) { 
    ROS_WARN("Relative pose change too large: trans=%.3f, rot=%.3f, rejecting", trans_norm, rot_angle);
    return false;
  }

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv) {
  uv.clear();
  if (img.empty())
    return;

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
                                         vector<cv::Point2f>& left_pts, vector<cv::Point2f>& right_pts) {

  right_pts.clear();
  if (_img.empty() || _img1.empty() || left_pts.empty())
    return false;

  vector<cv::Point2f> right_pred;
  vector<uchar> status;
  vector<float> err;

  cv::calcOpticalFlowPyrLK(_img, _img1, left_pts, right_pred, status, err,
                           cv::Size(21, 21), 3,
                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                           0, 1e-4);

  // 反向检查
  if (FLOW_BACK) {
    vector<cv::Point2f> left_back;
    vector<uchar> status_back;
    vector<float> err_back;
    cv::calcOpticalFlowPyrLK(_img1, _img, right_pred, left_back, status_back, err_back,
                             cv::Size(21, 21), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             0, 1e-4);
    for (size_t i = 0; i < status.size(); ++i) {
      if (!status[i])
        continue;
      if (!status_back[i] || distance(left_pts[i], left_back[i]) > 1.0) {
        status[i] = 0;
      }
    }
  }

  const int rows = _img1.rows;
  const int cols = _img1.cols;
  for (size_t i = 0; i < right_pred.size(); ++i) {
    if (!status[i])
      continue;
    if (!inBorder(right_pred[i], rows, cols)) {
      status[i] = 0;
    }
  }

  right_pts = right_pred;
  reduceVector<cv::Point2f>(left_pts, status);
  reduceVector<cv::Point2f>(right_pts, status);

  bool success = left_pts.size() >= (size_t)MIN_CNT;
  if (!success)
    fail_cnt++;
  else
    fail_cnt = 0;

  return success;
}

void Estimator::generate3dPoints(const vector<cv::Point2f>& left_pts,
                                 const vector<cv::Point2f>& right_pts, 
                                 vector<cv::Point3f>& cur_pts_3d,
                                 vector<cv::Point2f>& cur_pts_2d) {

  Eigen::Matrix<double, 3, 4> P1, P2;

  P1 << 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0;
  P2.block(0,0,3,3) = (Tlr.block(0,0,3,3).transpose());
  P2.block(0,3,3,1) = -P2.block(0,0,3,3) * Tlr.block(0,3,3,1);

  vector<uchar> status;

  for (unsigned int i = 0; i < left_pts.size(); ++i) {
    Vector2d pl(left_pts[i].x, left_pts[i].y);
    Vector2d pr(right_pts[i].x, right_pts[i].y);
    Vector3d pt3;
    triangulatePoint(P1, P2, pl, pr, pt3);

    if (pt3[2] > 0) {
      cur_pts_3d.push_back(cv::Point3f(pt3[0], pt3[1], pt3[2]));
      status.push_back(1);
    } else {
      status.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(cur_pts_2d, status);
}

bool Estimator::inBorder(const cv::Point2f& pt, const int& row, const int& col) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y &&
      img_y < row - BORDER_SIZE;
}

double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2) {
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

template <typename Derived>
void Estimator::reduceVector(vector<Derived>& v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

void Estimator::updateLatestStates(frame &latest_frame) {
  latest_time = latest_frame.frame_time;

  Matrix3d w_R_c = latest_frame.w_R_c;
  Vector3d w_t_c = latest_frame.w_t_c;

  Matrix3d R_bc = ric[0];
  Vector3d t_bc = tic[0];
  
  // World_T_Body = World_T_Camera * Camera_T_Body
  Matrix3d w_R_b = w_R_c * R_bc.transpose();
  Vector3d w_t_b = w_R_c * (-R_bc.transpose() * t_bc) + w_t_c;

  latest_P = w_t_b;
  latest_Q = Quaterniond(w_R_b);
  latest_Q.normalize();

  // 关键帧 IMU 位姿
  Matrix3d w_R_ck = key_frame.w_R_c;
  Vector3d w_t_ck = key_frame.w_t_c;

  Matrix3d w_R_bk = w_R_ck * R_bc.transpose();
  Vector3d w_t_bk = w_R_ck * (-R_bc.transpose() * t_bc) + w_t_ck;

  // 相对位姿
  Matrix3d R_kb = w_R_bk.transpose() * w_R_b;
  Vector3d t_kb = w_R_bk.transpose() * (w_t_b - w_t_bk);

  latest_rel_P = t_kb;
  latest_rel_Q = Quaterniond(R_kb);
  latest_rel_Q.normalize();

  latest_pointcloud = latest_frame.xyz;
}

void Estimator::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
                                 Eigen::Vector2d& point0, Eigen::Vector2d& point1,
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

double Estimator::reprojectionError(Matrix3d &R, Vector3d &t, cv::Point3f &key_pts_3d, cv::Point2f &cur_pts_2d){
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
  // 原有逻辑保留，虽然已被新的平滑逻辑取代
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