
#ifndef ST_LIDAR_SLAM_UTILS_H__
#define ST_LIDAR_SLAM_UTILS_H__

#include <time.h>

#define ST_LIDAR_SLAM_EXPORTS 1

#ifdef _MSC_VER
#ifdef __cplusplus
#ifdef ST_LIDAR_SLAM_EXPORTS
#define ST_SDK_API  extern "C" __declspec(dllexport)
#else
#define ST_SDK_API extern "C" __declspec(dllimport)
#endif
#else
#ifdef ST_LIDAR_SLAM_EXPORTS
#define ST_SDK_API __declspec(dllexport)
#else
#define ST_SDK_API __declspec(dllimport)
#endif
#endif
#else /* _MSC_VER */
#ifdef __cplusplus
#ifdef ST_LIDAR_SLAM_EXPORTS
#define ST_SDK_API  extern "C" __attribute__((visibility ("default")))
#else
#define ST_SDK_API extern "C"
#endif
#else
#ifdef ST_LIDAR_SLAM_EXPORTS
#define ST_SDK_API __attribute__((visibility ("default")))
#else
#define ST_SDK_API
#endif
#endif
#endif

#define ST_RET_OK                           0
#define ST_RET_E_INVALIDARG                 -1  ///< 无效参数
#define ST_RET_E_HANDLE                     -2  ///< 句柄错误
#define ST_RET_E_OUTOFMEMORY                -3  ///< 内存不足
#define ST_RET_E_FAIL                       -4  ///< 内部错误
#define ST_RET_E_DELNOTFOUND                -5  ///< 定义缺失
#define ST_RET_E_FILE_NOT_FOUND             -7  ///< 文件不存在
#define ST_RET_E_INVALID_FILE_FORMAT        -8  ///< 文件格式不正确导致加载失败
#define ST_RET_E_FILE_EXPIRE                -9  ///< 文件过期

typedef int st_result_t;
typedef void* st_handle_t;

struct st_point3d_t {
    float x;
    float y;
    float z;
    float intensity;
};

struct st_point2d_t {
    float x;
    float y;
    float intensity;
};

/// @brief 3D位姿
struct st_pose3d_t {
    double mat[16];
};

struct st_imu_data_t {
    double ax;  // acceleration in x, i.e. in direction of vehicle front (m/s^2)
    double ay;  // acceleration in y, i.e. in direction of vehicle left (m/s^2)
    double az;  // acceleration in z, i.e. in direction of vehicle top (m/s^2)
    double wx;  // angular rate around x (rad/s)
    double wy;  // angular rate around y (rad/s)
    double wz;  // angular rate around z (rad/s)
};

/// @brief LiDAR-SLAM 输入
struct st_sensors_data_t {
    st_point3d_t* points;
    int points_num;
    int frame_id;
    st_pose3d_t gnss_pose;
    st_imu_data_t imu_data;
    struct timeval time_stamp;
    
    bool has_gnss_pose;
    bool has_imu_data;
    bool has_time_stamp;
};

/// @brief LiDAR-SLAM 单帧输出
struct st_lidar_slam_output_t {
    st_pose3d_t pose;
};

/// @brief 地图信息，包括位姿态和序号
struct st_lidar_slam_map_info_t {
    st_pose3d_t pose;
    int frame_num;
    int index;
};

/// @brief 单帧信息，包括估计位姿和帧号
struct st_lidar_slam_frame_info_t {
    st_pose3d_t pose;
    int frame_id;
};

/// @brief 局部地图数据结构
struct st_lidar_slam_local_map_t {
    st_point3d_t* points;
    int points_count;
    st_lidar_slam_map_info_t info;
};

/// @brief 获取局部地图更新回调函数
/// @param[in] map, 局部地图信息
/// @param[in] args, 其他参数
typedef int (RECV_LOCAL_MAP_CALLBAK_FUNC)(
    const st_lidar_slam_local_map_t* map, 
    void* args
);

#endif  // ST_LIDAR_SLAM_UTILS_H_

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
