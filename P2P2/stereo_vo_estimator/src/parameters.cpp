#include "parameters.h"

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
int ROW, COL;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string IMU_TOPIC = "/djiros/imu";
std::vector<std::string> CAM_NAMES;
int MIN_CNT;
int MAX_CNT;
int MIN_DIST;
double TRANSLATION_THRESHOLD;
double ROTATION_THRESHOLD;
double FEATURE_THRESHOLD;
int FLOW_BACK;
int SHOW_FEATURE;
double MAX_FREQ;

int USE_QUALITY_AWARE = 0;
int USE_QUALITY_FILTERING = 0;
int USE_WEIGHTED_PNP = 0;
int USE_IMU = 0;
double MIN_DISPARITY_Q = 1.0;
double MAX_DEPTH_Q = 20.0;
double MAX_FB_ERROR_Q = 2.0;
double MAX_STEREO_LK_ERROR_Q = 20.0;
double MAX_TEMPORAL_LK_ERROR_Q = 20.0;
double MIN_QUALITY_WEIGHT = 0.1;
double WEIGHT_DISP_SIGMA = 2.0;
double WEIGHT_STEREO_SIGMA = 10.0;
double WEIGHT_FB_SIGMA = 1.0;
double WEIGHT_TEMP_SIGMA = 10.0;
double WEIGHT_TOP_RATIO = 0.7;
double IMU_MAX_INTERVAL = 0.01;
double GYRO_WEIGHT = 1.0;
double ACC_WEIGHT = 0.0;
double IMU_PREDICTION_MAX_TIME = 0.2;
double CONFIDENCE_WARN_THRESHOLD = 0.35;


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_FREQ = fsSettings["max_freq"];
    MAX_CNT = fsSettings["max_cnt"];
    MIN_CNT = fsSettings["min_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    TRANSLATION_THRESHOLD = fsSettings["translation_threshold"];
    ROTATION_THRESHOLD = fsSettings["rotation_threshold"];
    FEATURE_THRESHOLD = fsSettings["feature_threshold"];
    SHOW_FEATURE = fsSettings["show_feature"];
    FLOW_BACK = fsSettings["flow_back"];

    // 质量感知 VO 第一阶段参数均为可选项。yaml 中缺省时保持 baseline 行为。
    if (!fsSettings["use_quality_aware"].empty())
        USE_QUALITY_AWARE = (int)fsSettings["use_quality_aware"];
    if (!fsSettings["use_quality_filtering"].empty())
        USE_QUALITY_FILTERING = (int)fsSettings["use_quality_filtering"];
    if (!fsSettings["use_weighted_pnp"].empty())
        USE_WEIGHTED_PNP = (int)fsSettings["use_weighted_pnp"];
    if (!fsSettings["use_imu"].empty())
        USE_IMU = (int)fsSettings["use_imu"];
    if (!fsSettings["imu_topic"].empty())
        fsSettings["imu_topic"] >> IMU_TOPIC;
    if (!fsSettings["min_disparity_q"].empty())
        MIN_DISPARITY_Q = (double)fsSettings["min_disparity_q"];
    if (!fsSettings["max_depth_q"].empty())
        MAX_DEPTH_Q = (double)fsSettings["max_depth_q"];
    if (!fsSettings["max_fb_error_q"].empty())
        MAX_FB_ERROR_Q = (double)fsSettings["max_fb_error_q"];
    if (!fsSettings["max_stereo_lk_error_q"].empty())
        MAX_STEREO_LK_ERROR_Q = (double)fsSettings["max_stereo_lk_error_q"];
    if (!fsSettings["max_temporal_lk_error_q"].empty())
        MAX_TEMPORAL_LK_ERROR_Q = (double)fsSettings["max_temporal_lk_error_q"];
    if (!fsSettings["min_quality_weight"].empty())
        MIN_QUALITY_WEIGHT = (double)fsSettings["min_quality_weight"];
    if (!fsSettings["weight_disp_sigma"].empty())
        WEIGHT_DISP_SIGMA = (double)fsSettings["weight_disp_sigma"];
    if (!fsSettings["weight_stereo_sigma"].empty())
        WEIGHT_STEREO_SIGMA = (double)fsSettings["weight_stereo_sigma"];
    if (!fsSettings["weight_fb_sigma"].empty())
        WEIGHT_FB_SIGMA = (double)fsSettings["weight_fb_sigma"];
    if (!fsSettings["weight_temp_sigma"].empty())
        WEIGHT_TEMP_SIGMA = (double)fsSettings["weight_temp_sigma"];
    if (!fsSettings["weight_top_ratio"].empty())
        WEIGHT_TOP_RATIO = (double)fsSettings["weight_top_ratio"];
    if (!fsSettings["imu_max_interval"].empty())
        IMU_MAX_INTERVAL = (double)fsSettings["imu_max_interval"];
    if (!fsSettings["gyro_weight"].empty())
        GYRO_WEIGHT = (double)fsSettings["gyro_weight"];
    if (!fsSettings["acc_weight"].empty())
        ACC_WEIGHT = (double)fsSettings["acc_weight"];
    if (!fsSettings["imu_prediction_max_time"].empty())
        IMU_PREDICTION_MAX_TIME = (double)fsSettings["imu_prediction_max_time"];
    if (!fsSettings["confidence_warn_threshold"].empty())
        CONFIDENCE_WARN_THRESHOLD = (double)fsSettings["confidence_warn_threshold"];


    printf("MAX CNT: %d\n", MAX_CNT);
    printf("MIN CNT: %d\n", MIN_CNT);
    printf("MIN DIST: %d\n", MIN_DIST);
    printf("TRANSLATION THRESHOLD: %f\n", TRANSLATION_THRESHOLD);
    printf("ROTATION THRESHOLD: %f\n", ROTATION_THRESHOLD);
    printf("FLOW BACK: %d\n", FLOW_BACK);
    printf("SHOW FEATURE: %d\n", SHOW_FEATURE);
    printf("USE QUALITY AWARE: %d\n", USE_QUALITY_AWARE);
    printf("USE QUALITY FILTERING: %d\n", USE_QUALITY_FILTERING);
    printf("USE WEIGHTED PNP: %d\n", USE_WEIGHTED_PNP);
    printf("USE IMU: %d\n", USE_IMU);
    printf("IMU TOPIC: %s\n", IMU_TOPIC.c_str());


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    

    cv::Mat cv_T;
    Eigen::Matrix4d T;

    fsSettings["body_T_cam0"] >> cv_T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    fsSettings["body_T_cam1"] >> cv_T;
    cv::cv2eigen(cv_T, T);
    RIC.push_back(T.block<3, 3>(0, 0));
    TIC.push_back(T.block<3, 1>(0, 3));
    
    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::string cam1Path = configPath + "/" + cam1Calib; 
    CAM_NAMES.push_back(cam1Path);


    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    fsSettings.release();
}
