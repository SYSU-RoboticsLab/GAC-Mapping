
#ifndef LIDAR_SLAM_MAP_VIEWER_H_
#define LIDAR_SLAM_MAP_VIEWER_H_ 

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

#include <complex>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pthread.h>
#include <vector>
#include <map>

#include "st_lidar_slam_utils.h"

#if defined(__cplusplus)
extern "C" {
#endif
#include <GL/freeglut.h>
#if defined(__cplusplus)
}
#endif

class Lock {
public:
    Lock(pthread_mutex_t& mutex_lock) :
        _mutex_lock(mutex_lock) {
        pthread_mutex_lock(&_mutex_lock);     
    }
    
    ~Lock() {
        pthread_mutex_unlock(&_mutex_lock);
    }

private:
    pthread_mutex_t& _mutex_lock;
};

// hd map point structure
struct map_point3d_t {
    float intensity;
    float x, y, z;
    unsigned char r, g, b;
    int label;  // semantic label
};

class MapViewer {
public:
    MapViewer();
    ~MapViewer();

    // @brief initialization function
    void Init(int win_width, int win_height);

    // @brief update liadr point cloud
    void UpdateCurLidarPoints(const std::vector<map_point3d_t>& ldr_points, 
            const Eigen::Matrix4f& pose,
            const std::vector<std::pair<int, int> >* corres = nullptr);

    // @brief update active submap
    void UpdateActiveLocalMap(const std::vector<map_point3d_t>& dynamic_pts, 
            const Eigen::Matrix4f& pose);

    // @brief add a new submap
    void AddSubmap(const st_lidar_slam_local_map_t& map);

    // @brief update submap poses
    void UpdateSubmapPoses(const st_lidar_slam_map_info_t* infos, int map_num);

    // @brief create a video file and save the screen-shot picture into video
    bool CreateScreenShotVideo(const std::string& video_name); 

    // @brief take a screen-shot and save into the video
    bool ScreenShotAndSaveIntoVideo();

    // @brief close the screen-shot video
    bool CloseScreenShotVideo();

    void UpdateGnssPose(const Eigen::Matrix4f& pose);
    void AddLidarPose(const Eigen::Matrix4f& pose);

    void AddEdge(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
    void AddFixPoints(const std::vector<map_point3d_t>& pts);
    
    void Stop();

    void ClearDynamicPoints();
    void Release();

private:
    struct submap_t {
        int index;
        Eigen::Matrix4f pose;
        std::vector<st_point3d_t> points;
    };

    // create a display thread
    int CreateDisplayThread();

    // read a point cloud from file(ply)
    bool ReadPointCloud(const std::string& file_name, std::vector<map_point3d_t>& pts);

    // display function
    static void Display();

    // display map point cloud
    static void DisplayPointCloud();

    // display lidar point cloud
    static void DisplayCurLidarPoints();
    static void DisplayCar();

    // display coordinate axis
    static void DisplayAxis();
    static void DrawPoseAxis(Eigen::Matrix4f pose);

    static void DisplayGnssTrace();
    static void DisplayLidarTrace();

    static void DisplayEdges();

    // take screen-shot and save it into file
    static void process_screen_shot();
  
    // glut callback function  
    static void IdleFunc();
    static void MouseFirstClick(int button, int state, int x, int y);
    static void MouseMotion(int x, int y);
    
    static void* DisplayThreadFunc(void* arg);
    void DisplayFunc();

    // compute the bounding box of all points
    static void ComputePointBBox(float bbox[6]);

    pthread_t _show_pid;  // display thread pid
    int _window_pos_x;
    int _window_pos_y;
    static int _window_height;
    static int _window_width;
    std::string _window_name;

    static pthread_mutex_t _s_lock;  // data lock
    static pthread_mutex_t _s_cur_lidar_data_lock;
    static pthread_mutex_t _s_fix_point_lock;
    static pthread_mutex_t _s_new_fix_point_lock;
    
    // image data 
    static unsigned char* _s_image_data;
    static int _s_image_data_size;
    static int _s_image_width;
    static int _s_image_height;

    static Eigen::Matrix4f _cur_gnss_pose;
    static std::vector<Eigen::Matrix4f> _gnss_poses;
    static std::vector<Eigen::Matrix4f> _lidar_poses;

    // dynamic points, which will be updated
    static std::vector<map_point3d_t> _dynamic_points;
    static std::vector<GLuint> _fix_points_display_lists;
    static std::vector< std::pair<Eigen::Vector3f, Eigen::Vector3f> > _edges; 

    // fixed points, which will not be updated
    static std::vector<map_point3d_t> _new_fix_points;
    static std::vector<map_point3d_t> _fix_points;
    static std::map<int, submap_t> _submaps;

    static Eigen::Matrix4f _cur_lidar_pose;  // current LiDAR pose
    static std::vector<map_point3d_t> _cur_lidar_points;  // current LiDAR map points

    static bool _s_take_screen_shot;
    static cv::VideoWriter* _s_video_writer;
    static std::string _s_video_writer_name;

    static bool _s_left_button, _s_right_button;
    static GLdouble _s_angle_x;
    static GLdouble _s_angle_y;
    static GLdouble _s_angle_z;
    static const float _s_angle_step;
    static int _s_mouse_left_old_x, _s_mouse_left_old_y;
    static float _s_scale;
    static const float _s_scale_up;
    static const float _s_scale_down;
    static int _mouse_right_old_x, _s_mouse_right_old_y;
    static float _s_translate_x, _s_translate_y;
    static float _s_translate_step;
    static float _s_roll, _s_yaw, _s_pitch;
    static float _s_point_scale;

    static std::vector<std::pair<int, int> > _corres_idxs; 
};

struct map_viewer_t {
    map_viewer_t() {
        is_on = true;
    }

    MapViewer viewer;
    bool is_on;
}; 

#endif  // LIDAR_SLAM_INCLUDE_MAP_VIEWER_H_

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
