/**
* This file is part of GAC-Mapping.
*
* Copyright (C) 2020-2022 JinHao He, Yilin Zhu / RAPID Lab, Sun Yat-Sen University 
* 
* For more information see <https://github.com/SYSU-RoboticsLab/GAC-Mapping>
*
* GAC-Mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* GAC-Mapping is distributed to support research and development of
* Ground-Aerial heterogeneous multi-agent system, but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GAC-Mapping. If not, see <http://www.gnu.org/licenses/>.
*/

#include "parameters.h"

std::string CAM_NAME;
// std::string CAM_NAME_MAP;

// camera parameters
int ROW;
int COL;
int FOCAL_LENGTH;
int DOWN_SAMPLE_NEED;
Eigen::Matrix3d RCL;
Eigen::Vector3d TCL;
Eigen::Matrix4d T_LC;
Eigen::Matrix4d T_CL;

// LSD detector parameters
int MAX_CNT_LSD;
int NUM_OCTAVES_LSD;
float SCALE_LSD;
// LBD descriptor parameters
int WIDTH_OF_BAND_LBD;

// corner feature parameters
int MAX_CNT_CORNER;
int MIN_DIST_CORNER;

int WINDOW_SIZE;

double F_THRESHOLD;

// laser scan registrators parameters
int N_SCANS;
int HORIZON_SCANS;
float MINIMUM_RANGE;
float ANGLE_RES_X;
float ANGLE_RES_Y;
float ANGLE_BOTTOM;
float CROP_NEAR;
float CROP_FAR;

float EDGE_THRESHOLD;
float SURF_THRESHOLD;
float NEAREST_FEATURE_DIST;
float OCCLUDE_THRESHOLD;

// laser odom parameters
float SCAN_PERIOD;
float DISTANCE_SQ_THRESHOLD;
float NEARBY_SCAN;
int DISTORTION;
float VISUAL_WEIGHT_MAX;

// submap parameters
int SUBMAP_LENGTH;
float KEEP_SQURE_DISTANCE;
// thumbnail para
float NDT_RESOLUTION_THUMBNAIL;
float NDT_RESOLUTION_MATCH;
int NEED_CHECK_DIRECTION;

// output para
int NEED_PUB_ODOM;
std::string GT_PATH;
double GT_START_TIME;
std::string OUT_ODOM_PATH;
std::string OUT_GT_PATH;
int AIR_GPS;

// cache para
std::string CACHE_PATH;

// config para
std::string CONFIG_PATHS[5];
std::string DATA_PATHS[5];
int CONFIG_ID = 0;

// debug print
bool DEBUG;

// display frame selection
bool display_frame_cam;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        if(DEBUG) ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name << ": " << ans);

        // 谁tm写的shutdown，没有就没有啊，要不然会导致程序崩溃
        // n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    n.param<bool>("/gacm/debug", DEBUG, false);
    n.param<bool>("/gacm/display_frame_cam", display_frame_cam, false);

    CONFIG_PATHS[0] = readParam<std::string>(n, "config_file");
    CONFIG_PATHS[1] = readParam<std::string>(n, "config_file1");
    CONFIG_PATHS[2] = readParam<std::string>(n, "config_file2");
    CONFIG_PATHS[3] = readParam<std::string>(n, "config_file3");
    CONFIG_PATHS[4] = readParam<std::string>(n, "config_file4");

    std::string config_file;
    if (CONFIG_ID > 4 ) {
        CONFIG_ID = 4;
    }
    config_file = CONFIG_PATHS[CONFIG_ID];
    if(DEBUG) ROS_INFO_STREAM("Get param from" << config_file);
    // config_file = readParam<std::string>(n, "config_file");
    // std::string config_file_map;
    // config_file_map = readParam<std::string>(n, "config_file_map");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    
    // cv::FileNode cvNode = fsSettings["gacm"];
    // display_frame_cam = (bool)static_cast<int>(cvNode["display_frame_cam"]);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FOCAL_LENGTH = fsSettings["focal_length"];
    CAM_NAME = config_file;
    // CAM_NAME_MAP = config_file_map;
    DOWN_SAMPLE_NEED = fsSettings["down_sample"];

    cv::Mat cv_R, cv_T, cv_R2;
    fsSettings["extrinsicRotation"] >> cv_R;
    fsSettings["extrinsicTranslation"] >> cv_T;
    // fsSettings["extrinsicRotation2"] >> cv_R2;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    Eigen::Matrix3d eigen_R2;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);
    // cv::cv2eigen(cv_R2, eigen_R2);
    Eigen::Quaterniond Q(eigen_R);
    eigen_R = Q.normalized();
    RCL = eigen_R;
    // Q = eigen_R2;
    // eigen_R2 = Q.normalized();
    // RCL2 = eigen_R2;
    TCL = eigen_T;
    if(DEBUG) ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RCL);
    if(DEBUG) ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TCL.transpose());
    // if(DEBUG) ROS_INFO_STREAM("Extrinsic_R2 : " << std::endl << RCL2);
    if(CONFIG_ID == 0)
    {
        T_CL = Eigen::Matrix4d::Identity();
        T_CL.block<3,3>(0,0) = RCL;
        T_CL.block<3,1>(0,3) = TCL;
        T_LC = T_CL.inverse().eval();
    }

    MAX_CNT_LSD = fsSettings["max_keyline_count"];
    NUM_OCTAVES_LSD = fsSettings["keyline_detector_octaves"];
    SCALE_LSD = fsSettings["keyline_detector_scale"];

    WIDTH_OF_BAND_LBD = fsSettings["lbd_descriptor_width_of_band"];

    MAX_CNT_CORNER = fsSettings["max_keypoint_count"];
    MIN_DIST_CORNER = fsSettings["min_keypoint_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];

    WINDOW_SIZE = fsSettings["sliding_window_size"];

    N_SCANS = fsSettings["laser_scan_number"];
    if(DEBUG) ROS_INFO_STREAM("NSCANS " << N_SCANS);
    HORIZON_SCANS = fsSettings["horizon_scans"];
    if(DEBUG) ROS_INFO_STREAM("HORIZON_SCANS " << HORIZON_SCANS);
    MINIMUM_RANGE = fsSettings["minimum_range"];
    ANGLE_RES_X = fsSettings["angle_res_x"];
    ANGLE_RES_Y = fsSettings["angle_res_y"];
    ANGLE_BOTTOM = fsSettings["angle_bottom"];
    CROP_NEAR = fsSettings["crop_near"];
    CROP_FAR = fsSettings["crop_far"];

    EDGE_THRESHOLD = fsSettings["edge_threshold"];
    SURF_THRESHOLD = fsSettings["surf_threshold"];
    NEAREST_FEATURE_DIST = fsSettings["nearest_feature_search_dist"];
    OCCLUDE_THRESHOLD = fsSettings["occlude_threshold"];
    
    SCAN_PERIOD = fsSettings["scan_period"];
    DISTANCE_SQ_THRESHOLD = fsSettings["distance_sq_threshold"];
    NEARBY_SCAN = fsSettings["nearby_scan"];
    DISTORTION = fsSettings["distortion"];
    VISUAL_WEIGHT_MAX = fsSettings["visual_weight_max"];

    SUBMAP_LENGTH = fsSettings["submap_length"];
    KEEP_SQURE_DISTANCE = fsSettings["keep_squre_distance"];
    NDT_RESOLUTION_THUMBNAIL = fsSettings["ndt_resolution_thumbnail"];
    NDT_RESOLUTION_MATCH = fsSettings["ndt_resolution_match"];
    NEED_CHECK_DIRECTION = fsSettings["need_check_direction"];

    NEED_PUB_ODOM = fsSettings["need_pub_odom"];
    fsSettings["gt_path"] >>GT_PATH;
    GT_START_TIME = fsSettings["gt_start_time"];
    fsSettings["out_odom_path"] >> OUT_ODOM_PATH;
    erase_comment(OUT_ODOM_PATH);
    fsSettings["out_gt_path"] >> OUT_GT_PATH;
    AIR_GPS = fsSettings["air_gps"];

    fsSettings["cache_path"]>>CACHE_PATH;
    erase_comment(CACHE_PATH);
    fsSettings.release();

}

void erase_comment(std::string& str)
{
    std::string::size_type comment_pos = str.find_first_of("#");
    if(comment_pos != std::string::npos)
        str.erase(comment_pos);

    // remove tab and space
    std::string::size_type last_pos = str.find_last_of("//");
    if(last_pos != str.size()-1)
        str.erase(last_pos+1);
    else if (last_pos == std::string::npos)
    {
        std::cout << "\033[1;31mThe path format is invalid! path: " + str + "\n\033[0m";
        exit(-1);
    }
}

// 显示进度，和python里的tqdm效果差不多
void status(int length, float percent)
{
    // std::cout << "\x1B[2k";     // Erase entire current line
    std::cout << "\x1B[0E";     // Move to the beginning of the current line
    std::string progress = "Progress: [";
    for(int i = 0; i < length; ++i)
    {
        if(i < length * percent)
            progress += "=";
        else
            progress += " ";
    }
    std::cout << progress + "] " << std::fixed << std::setprecision(2) << percent * 100.0f << "%";
    std::cout.flush();
}

void printCopyright()
{
    std::cout << "\nGAC-Mapping  Copyright (C) Copyright (C) 2020-2022 JinHao He, Yilin Zhu\n"
            << "This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.\n"
            << "This is free software, and you are welcome to redistribute it\n"
            << "under certain conditions; type `show c' for details.\n\n";
}