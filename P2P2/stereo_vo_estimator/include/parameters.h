#pragma once

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern int ROW, COL;
extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string IMU_TOPIC;
extern std::vector<std::string> CAM_NAMES;
extern double MAX_FREQ;
extern int MAX_CNT;
extern int MIN_CNT;
extern int MIN_DIST;
extern double TRANSLATION_THRESHOLD;
extern double ROTATION_THRESHOLD;
extern double FEATURE_THRESHOLD;
extern int SHOW_FEATURE;
extern int FLOW_BACK;

extern int USE_QUALITY_AWARE;
extern int USE_QUALITY_FILTERING;
extern int USE_WEIGHTED_PNP;
extern int USE_IMU;
extern double MIN_DISPARITY_Q;
extern double MAX_DEPTH_Q;
extern double MAX_FB_ERROR_Q;
extern double MAX_STEREO_LK_ERROR_Q;
extern double MAX_TEMPORAL_LK_ERROR_Q;
extern double MIN_QUALITY_WEIGHT;
extern double WEIGHT_DISP_SIGMA;
extern double WEIGHT_STEREO_SIGMA;
extern double WEIGHT_FB_SIGMA;
extern double WEIGHT_TEMP_SIGMA;
extern double WEIGHT_TOP_RATIO;
extern double IMU_MAX_INTERVAL;
extern double GYRO_WEIGHT;
extern double ACC_WEIGHT;
extern double IMU_PREDICTION_MAX_TIME;
extern double CONFIDENCE_WARN_THRESHOLD;

void readParameters(std::string config_file);
