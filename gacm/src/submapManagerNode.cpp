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

// #include "map_manager.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "mapping/sub_map.h"
#include "netvlad_tf_test/CompactImg.h"
#include "parameters.h"

#include <fstream>
#include <stdlib.h>
#include <dirent.h>

ros::NodeHandle* nh_ref; 

int cur_robot_id = 0;
// store map for different robots
std::vector<std::shared_ptr<SubMap>> MapDatabase[5];//[100];
bool is_drone[6];
int cur_map_id = 0;
int cur_thumbnail_id = 0;
int cur_loop_test_id = 1;

std::mutex mBuf;
std::mutex mProcess;
std::mutex m_loop;
std::mutex mTProcess;
std::mutex mOptimize;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometryLast = 0;
double timeLaserOdometry = 0;

// float score[60][60];
int empty_loop = 0;
bool lastnode_created = false;
bool need_init_coord = false;
// bool pause = 0;
bool updating_pg = false;
bool updating_map = false;

Eigen::Matrix<float, 100,100> diff_matrix;

class LoopScore {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		float loop_score;
		float angle_rel;
		int robot_id;
		int submap_id;
		Eigen::Matrix4f guess;
		Eigen::Matrix4f try_guess;

		LoopScore(float loop_score_, float angle_rel_, int robot_id_, int submap_id_, const Eigen::Matrix4f& guess_, const Eigen::Matrix4f& try_guess_) :
		 loop_score(loop_score_), angle_rel(angle_rel_), robot_id(robot_id_), submap_id(submap_id_), guess(guess_), try_guess(try_guess_){}
};
bool cmp(const LoopScore& lhs, const LoopScore& rhs){
    return lhs.loop_score < rhs.loop_score || 
		(lhs.loop_score == rhs.loop_score && lhs.angle_rel < rhs.angle_rel);
}

void std2EigenVector(const std::vector<float>& in, Eigen::VectorXf& out) {
	out.resize(in.size());
	for (uint i = 0; i < in.size(); i++) {
		out(i) = in[i];
	}
}

std::vector<LoopScore, Eigen::aligned_allocator<LoopScore>> loop_scores;

class ManagerIndex {
	public:
	size_t robot_id;
	size_t submap_id;
};

std::vector<MeasurementEdge, Eigen::aligned_allocator<MeasurementEdge>> loopEdgeBuf;
std::vector<MeasurementEdge, Eigen::aligned_allocator<MeasurementEdge>> dropEdgeBuf;



ros::Publisher pubPathOpt[6];
ros::Publisher pubRelEdge;
ros::Publisher pubOptimizedMap[6];
ros::Publisher pubNewSessionSignal;
ros::Publisher pubQuery, pubDetectL;
ros::ServiceClient netvlad_client;


pcl::KdTreeFLANN<PointType>::Ptr kdtree;
pcl::PointCloud<PointType>::Ptr submap_base_map[6];

void performOptimization();
void update_submapbase_cloud();
void publish_opt_posegraph(bool pubmap);
void publish_opt_map(bool pubmap, bool pubmerge);
void globalOptimization();

// 得到了数据，这个时候算法才会开始本节点的其他操作
bool is_first = true;

void cornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
	
	mBuf.lock();
	cornerLastBuf.push(laserCloudCornerLast2);
	mBuf.unlock();
	is_first = false;
}

void surfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
	
	mBuf.lock();
	surfLastBuf.push(laserCloudSurfLast2);
	mBuf.unlock();
}

void fullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
	
	mBuf.lock();
	fullResBuf.push(laserCloudFullRes2);
	mBuf.unlock();
}

//receive odomtry
void odometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(laserOdometry);
	mBuf.unlock();
}


void loopClosureInfoHandler(/*todo*/) {
	//XXX.pushback
}

void export_gt() {
	std::fstream fs;
	std::stringstream ss;
	ss << 0;
	std::string gt_path = GT_PATH;
	// std::cout << "\nPlease enter gt path\n";
	// std::cin >> gt_path;

	ros::Time t_begin;
	double temp_t = GT_START_TIME;
	std::cout << "\ngt start time\n";
	// std::cin >> temp_t;
	std::cout << temp_t;
	t_begin.fromSec(temp_t);
	std::fstream gs;
	gs.open(gt_path, ios::in);
	// fs.open("/home/ncslab_slam/Pictures/odomout/gt" + ss.str()+ ".txt", ios::out);
	fs.open(OUT_GT_PATH + ss.str()+ ".txt", ios::out);
	std::string line;
	if (gs.is_open()) {

		// skip header
		std::getline(gs, line);
		std::getline(gs, line);
		line.clear();

		const float T=1.0f/10;
		ros::Duration d(T);
		std::cout << "GPS mode " << AIR_GPS << "\n";
		while (!gs.eof()) {
			std::string line_;
			std::getline(gs, line_);
			std::stringstream line_s;
			line_s << line_;
			// std::cout << "\n" << line << "\n";
			std::string s_time;
			line_s >> s_time;
			// std::cout << s_time << std::endl;
			double x,y,z, roll, pitch, yaw;
			line_s >> x >> y>> z >> roll >> pitch >> yaw;
			double north, east, height, utc_time;
			line_s >> north >> east >> height >> utc_time;
			Eigen::Quaterniond q;
			// rotation order yaw pitch roll[zxy]
			Eigen::AngleAxisd Rz , Ry, Rx;
			if (AIR_GPS == 0) { // negative yaw
				Rz = Eigen::AngleAxisd(-1* yaw*M_PI/180.0, Eigen::Vector3d::UnitZ());
				Rx = Eigen::AngleAxisd(pitch*M_PI/180.0, Eigen::Vector3d::UnitX());
				Ry = Eigen::AngleAxisd(roll*M_PI/180.0, Eigen::Vector3d::UnitY());
				
				// std::cout << "\nz\n" <<  Eigen::AngleAxisd(-1* yaw*M_PI/180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix()
				//           << "\nx\n" << Eigen::AngleAxisd(pitch*M_PI/180.0, Eigen::Vector3d::UnitX()).toRotationMatrix()
				// 		  << "\ny\n" << Eigen::AngleAxisd(roll*M_PI/180.0, Eigen::Vector3d::UnitY()).toRotationMatrix();// ground yaw取负 air 全取负
			} else if (AIR_GPS == 1) { // nagative all
				Rz = Eigen::AngleAxisd(-1* yaw*M_PI/180.0, Eigen::Vector3d::UnitZ());
				Rx = Eigen::AngleAxisd(-1* pitch*M_PI/180.0, Eigen::Vector3d::UnitX());
				Ry = Eigen::AngleAxisd(-1* roll*M_PI/180.0, Eigen::Vector3d::UnitY());// ground yaw取负 air 全取负
			} else if (AIR_GPS == 2) { // normal
				Rz = Eigen::AngleAxisd(yaw*M_PI/180.0, Eigen::Vector3d::UnitZ());
				Rx = Eigen::AngleAxisd(pitch*M_PI/180.0, Eigen::Vector3d::UnitX());
				Ry = Eigen::AngleAxisd(roll*M_PI/180.0, Eigen::Vector3d::UnitY()); // ground yaw取负 air 全取负
			}
			q = Rz*Rx*Ry;
			t_begin.fromSec(utc_time);
			fs << t_begin.toNSec() << " "
					<< x << " " << y << " " << z << " "
					<< q.x() << " " << q.y() << " " << q.z() << " "  << q.w() << "\n";
		}
		fs.close();
		gs.close();
	} else {
		std:: cout << "gt file open fail\n";
	}
	if(DEBUG) std::cout << "export gt done\n";
}

void export_odom() {
	if (NEED_PUB_ODOM == 0) {
		return;
	}
	// std::fstream fs;
	for (int r= 0; r <=cur_robot_id; r++) {
		std::fstream fs;
		std::stringstream ss;
		ss << r;
		fs.open(OUT_ODOM_PATH + ss.str()+ ".txt", ios::out);
		int database_size = MapDatabase[r].size();


		for (int i = 0; i < database_size-1; i++) { // the last submap is isolated
			int max_size = MapDatabase[r][i]->pose_graph_local.size();
			for (int j = 1; j < max_size; j++) { // start from 1, skip first node
				
				Eigen::Quaterniond q = MapDatabase[r][i]->pose_graph_local[j].q;
				Eigen::Vector3d t = MapDatabase[r][i]->pose_graph_local[j].t;

				t = MapDatabase[r][i]->t_world_base + MapDatabase[r][i]->q_world_base*t;
				q = MapDatabase[r][i]->q_world_base * q;

				// 输出tum格式数据
				fs << std::fixed << std::setprecision(6) << MapDatabase[r][i]->pose_graph_local[j].stamp << " "
				   	<< std::setprecision(7) << t.x() << " " << t.y() << " " << t.z() << " "
					<< q.x() << " " << q.y() << " " << q.z() << " "  << q.w() << "\n";
				
			}
		}

		fs.close();

		
		
		if(DEBUG) std::cout << "Done export\n";
		
	}
}



// 强制保存文件
void save_all() {
	// int save;
	printf("\033[1;32m\nBegin save map\n\033[0m");
	if(DEBUG) std::cout << "Current Edges " << loopEdgeBuf.size() << " : " << dropEdgeBuf.size() <<  "\n";

	int database_size = MapDatabase[cur_robot_id].size();
	for (int i = 0; i < database_size; i++) {
		MapDatabase[cur_robot_id][i]->saveMapToFile(std::string(std::getenv("HOME")) + "/gacm_output/data/testSavemap/");
	}
	std::fstream fs;
	// 打开并清空文件
	fs.open( std::string(std::getenv("HOME")) + "/gacm_output/data/testSavemap/robot" + std::to_string(cur_robot_id) + "_loop.cfg", std::ios::ate|std::ios::out);
	fs << cur_robot_id << "\n";
	fs << is_drone[cur_robot_id] << "\n";
	fs << database_size << "\n";
	fs << loopEdgeBuf.size() << "\n";
	for (auto edge:loopEdgeBuf) {
		fs << std::fixed << edge.stamp_from << " " << edge.stamp_to << " "  
                    << edge.robot_from << " " << edge.robot_to << " "
                    << edge.submap_from << " " << edge.submap_to << " "
                    << edge.index_from << " " << edge.index_to << " "
                    << edge.q.w() << " " << edge.q.x() << " " << edge.q.y() << " "<< edge.q.z() << " " 
                    << edge.t.x() << " " << edge.t.y() << " " << edge.t.z() << "\n";
	}
	
	fs << dropEdgeBuf.size() << "\n";
	for (auto edge:dropEdgeBuf) {
		fs << std::fixed << edge.stamp_from << " " << edge.stamp_to << " "  
                    << edge.robot_from << " " << edge.robot_to << " "
                    << edge.submap_from << " " << edge.submap_to << " "
                    << edge.index_from << " " << edge.index_to << " "
                    << edge.q.w() << " " << edge.q.x() << " " << edge.q.y() << " "<< edge.q.z() << " " 
                    << edge.t.x() << " " << edge.t.y() << " " << edge.t.z() << "\n";
	}
	fs.close();
	std::cout << "\033[1;32m\nSave data Done!\n\033[0m";
}

// 该函数只在main函数开始时进行
void load_all(std::string basepath = std::string(std::getenv("HOME")) + "/gacm_output/data/testSavemap/") {
	// 只有当前机器人为第一个的时候才会进行询问
	if(cur_robot_id == 0)
	{
		int load = 0;
		std:: cout << "\033[1;36m\nNeed Load old session (\"1\" for yes and \"0\" for no)?\n\033[0m";
		std::cin >> load;
		if (load != 1) {
			cout << "\033[1;32m\nload all robot done! Ready to receive robot[" << cur_robot_id << "] data!\n\033[0m";
			return;
		}
	} 

	std:: cout << "\033[1;32m\nStart to Load " << basepath + "robot" + std::to_string(cur_robot_id) + "_loop.cfg\n\033[0m" ;
	std::fstream fs;

	// 尝试抓取loop文件里的数据，loop文件里存放了因子图中回环部分的数据
	try
	{
		fs.open(basepath+"robot"+std::to_string(cur_robot_id)+"_loop.cfg", std::ios::in);
		// 文件里机器人的id应该要和当前算法运行时的id一致，否则强制退出
		int f_robot_id = -100;
		fs >> f_robot_id;
		if(cur_robot_id != f_robot_id)
		{	
			fs.close();
			throw std::string("wrong robot id!\n");
		}
		cur_robot_id = f_robot_id;
	}
	catch(std::string& error)
	{
		cout << "\033[1;34m\nNo file find, return! error is: " << error << "\n\033[0m";

		// 这里的写法按main函数开有那样写，这是为了保证当前机器人初始化正常
		is_drone[cur_robot_id] = (bool)NEED_CHECK_DIRECTION;
		MapDatabase[cur_robot_id].clear();  // 这是必须要，相当于当前机器人所有子图全部清空
		MapDatabase[cur_robot_id].push_back(std::shared_ptr<SubMap>(new SubMap(*nh_ref))); // first submap
		MapDatabase[cur_robot_id][cur_map_id]->initParameters(cur_robot_id,0,NEED_CHECK_DIRECTION,0); //double check? // first node
		kdtree.reset(new pcl::KdTreeFLANN<PointType>());
		submap_base_map[cur_robot_id].reset(new pcl::PointCloud<PointType>());

		// 如果没有文件且当前机器人编号不为0，说明文件获取完毕，则进行依次优化
		if(cur_robot_id != 0)
		{
			performOptimization();
			update_submapbase_cloud();
			globalOptimization();
			export_odom();

			// 这里还需要初始化这些东西，这时的处理与mapping process里的new session过程一致
			cur_map_id = 0;
			cur_thumbnail_id = 0;
			cur_loop_test_id = 0; // loop test start from first node
			need_init_coord = true;
		}

		cout << "\033[1;32m\nload all robot done! Ready to receive robot[" << cur_robot_id << "] data!\n\033[0m";
		return;
	}

	int is_drone_;
	fs >> is_drone_;
	is_drone[cur_robot_id] = is_drone_ == 1? true:false;
	MapDatabase[cur_robot_id].clear();
	int database_size;
	fs >> database_size;
	std::cout << "Loading session " << cur_robot_id << " with " << database_size << " submaps \n";
	for (int i = 0; i < database_size; i++) {
		// std::string map_path = ;
		MapDatabase[cur_robot_id].push_back(std::shared_ptr<SubMap>(new SubMap(basepath, cur_robot_id, i, is_drone[cur_robot_id], *nh_ref)));
		if (MapDatabase[cur_robot_id][i]->thumbnailGenerated) {
			netvlad_tf_test::CompactImg img_srv;
			img_srv.request.req_img_name = MapDatabase[cur_robot_id][i]->thumbnails_db[0].second;
			int id_tn = MapDatabase[cur_robot_id][i]->thumbnails_db[0].first;
			if (netvlad_client.call(img_srv)) {
				if(DEBUG) ROS_ERROR("Succeed to call service");
				// ROS_INFO_STREAM("Descriptor len " << ((std::vector<float>)img_srv.response.res_des).size());
				Eigen::VectorXf desc;
				std2EigenVector(img_srv.response.res_des, desc);
				MapDatabase[cur_robot_id][i]->descriptor_db.push_back(std::make_pair(id_tn, desc));
			} else {
				ROS_ERROR("Failed to call service");
			}
		}
		if(DEBUG) std::cout << i+1 << " of " << database_size << "\n";
	}
	if(DEBUG) std::cout << "Session loaded\n"; 
	int loop_edges_size;
	fs >> loop_edges_size;
	int acc_edge_cnt = 0;
	int adj_edge_cnt = 0;
	int rej_edge_cnt = 0;
	for (int i = 0; i < loop_edges_size; i++) {
		MeasurementEdge edge;
		fs >> edge.stamp_from >> edge.stamp_to 
                    >> edge.robot_from >> edge.robot_to
                    >> edge.submap_from >> edge.submap_to
                    >> edge.index_from >> edge.index_to
                    >> edge.q.w() >> edge.q.x() >> edge.q.y() >>edge.q.z()
                    >> edge.t.x() >>edge.t.y() >> edge.t.z();
		loopEdgeBuf.push_back(edge);
		if (edge.robot_from != edge.robot_to || edge.submap_from != edge.submap_to+1) {
			acc_edge_cnt++;
		} else {
			adj_edge_cnt++;
		}
	}

	int drop_edges_size;
	fs >> drop_edges_size;
	for (int i = 0; i < drop_edges_size; i++) {
		MeasurementEdge edge;
		fs >> edge.stamp_from >> edge.stamp_to 
                    >> edge.robot_from >> edge.robot_to
                    >> edge.submap_from >> edge.submap_to
                    >> edge.index_from >> edge.index_to
                    >> edge.q.w() >> edge.q.x() >> edge.q.y() >>edge.q.z()
                    >> edge.t.x() >>edge.t.y() >> edge.t.z();
		dropEdgeBuf.push_back(edge);
		if (edge.robot_from != edge.robot_to || edge.submap_from != edge.submap_to+1) {
			rej_edge_cnt++;
		}
	}
	fs.close();
	if(DEBUG) std::cout << "\nEdges loaded, Edges : " << acc_edge_cnt << ": " << adj_edge_cnt << ": " << rej_edge_cnt << "\n";
	publish_opt_posegraph(true);

	CONFIG_ID++;
	std_msgs::Empty empty;
	pubNewSessionSignal.publish(empty);

	cur_robot_id++;
	CONFIG_ID = cur_robot_id;
	if(DEBUG) ROS_ERROR_STREAM("test config id " << CONFIG_ID);
	readParameters(*nh_ref);
	load_all( basepath );

}


void pubLoopImage(cv::Mat& query_tn, cv::Mat& response_tn, bool is_accept) {
	cv_bridge::CvImage bridge;
	if (is_accept) {
		cv::copyMakeBorder(query_tn,query_tn,5,5,5,5,cv::BORDER_CONSTANT, cv::Scalar(0,255,0));
		cv::copyMakeBorder(response_tn,response_tn,5,5,5,5,cv::BORDER_CONSTANT, cv::Scalar(0,255,0));
	} else {
		cv::copyMakeBorder(query_tn,query_tn,5,5,5,5,cv::BORDER_CONSTANT, cv::Scalar(0,0,255));
		cv::copyMakeBorder(response_tn,response_tn,5,5,5,5,cv::BORDER_CONSTANT, cv::Scalar(0,0,255));
	}
	bridge.image = query_tn;
	bridge.encoding = "bgr8";
	sensor_msgs::Image::Ptr query_image_ptr = bridge.toImageMsg();
	pubQuery.publish(query_image_ptr);
	bridge.image = response_tn;
	sensor_msgs::Image::Ptr response_image_ptr = bridge.toImageMsg();
	pubDetectL.publish(response_image_ptr);
}




void mapping_process()
{
	// main loop
	while (true) {

		// check topics sync
		
		while (!cornerLastBuf.empty() && !surfLastBuf.empty() && !fullResBuf.empty() && !odometryBuf.empty()) {
			mProcess.lock();

			mBuf.lock();
			while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				odometryBuf.pop();
			if (odometryBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				surfLastBuf.pop();
			if (surfLastBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				fullResBuf.pop();
			if (fullResBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// queue topic check

			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
			// timestamp double check
			// ROS_ERROR_STREAM("Begin time check");
			if (timeLaserCloudCornerLast != timeLaserOdometry ||
				timeLaserCloudSurfLast != timeLaserOdometry ||
				timeLaserCloudFullRes != timeLaserOdometry)
			{
				if(DEBUG) ROS_ERROR("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
				ROS_ERROR("unsync messeage!");
				mBuf.unlock();
				break;
			}

			mBuf.unlock();
			// now topic already sync

			// check if new session
			// 本节点里重新启动算法的部分
			if (lastnode_created) {
				//then create new submap
				ROS_INFO_STREAM("\033[1;32mCreate new session\033[0m");
				cur_robot_id++;
				CONFIG_ID = cur_robot_id;
				if(DEBUG) ROS_INFO_STREAM("\033[1;32mtest config id " << CONFIG_ID << "\033[0m");
				// std_msgs::Empty empty;
				// pubNewSessionSignal.publish(empty);
				readParameters(*nh_ref);
				is_drone[cur_robot_id] = (bool)NEED_CHECK_DIRECTION;
				MapDatabase[cur_robot_id].clear();
				MapDatabase[cur_robot_id].push_back(std::shared_ptr<SubMap>(new SubMap(*nh_ref))); // first submap
				kdtree.reset(new pcl::KdTreeFLANN<PointType>());
				submap_base_map[cur_robot_id].reset(new pcl::PointCloud<PointType>());
				cur_map_id = 0;
				cur_thumbnail_id = 0;
				cur_loop_test_id = 0; // loop test start from first node
				MapDatabase[cur_robot_id][cur_map_id]->initParameters(cur_robot_id,0,NEED_CHECK_DIRECTION,0); //double check? // first node
				need_init_coord = true;
			}

			m_loop.lock();

			MapDatabase[cur_robot_id][cur_map_id]->process(cornerLastBuf.front(), surfLastBuf.front(), fullResBuf.front(), odometryBuf.front());

			m_loop.unlock();

			empty_loop = 0; //stop thumnail generation from counting
			lastnode_created = false;
			
			mBuf.lock();
			cornerLastBuf.pop();
			surfLastBuf.pop();
			fullResBuf.pop();
			odometryBuf.pop();
			mBuf.unlock();
			
			if (MapDatabase[cur_robot_id][cur_map_id]->checkNeedNewSubMap()) {
				m_loop.lock();
				MapDatabase[cur_robot_id].push_back(MapDatabase[cur_robot_id][cur_map_id]->createNewSubMap()); // check, is parameter enough?
				cur_map_id++;
				m_loop.unlock();
			}

			mProcess.unlock();
		} // end topic empty check


		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	} // end mainloop

}




void testBaseScore(pcl::PointCloud<PointType>::Ptr all_submap_bases, std::vector<int>& knn_idx, std::vector<float>& knn_dist) {
	for (int knnid = 0; knnid < knn_idx.size(); knnid++) {
		int ri = 0;
		int i = all_submap_bases->points[knn_idx[knnid]].intensity;
		ri = i/100; // robot id
		i = i%100; // submap_id
		
		if (ri == cur_robot_id && i >= cur_loop_test_id - 5) {
			if(DEBUG) ROS_ERROR_STREAM("Skip test " << i << " (homo) too close to " << cur_loop_test_id);
			continue; // future submap && neighbour submap
		}

		// BUG，补充检查描述子是否为空
		if(MapDatabase[ri][i]->descriptor_db.size() == 0)
			continue;

		// descriptor distance
		float desc_score = (MapDatabase[ri][i]->descriptor_db[0].second - MapDatabase[cur_robot_id][cur_loop_test_id]->descriptor_db[0].second).norm();
		if(DEBUG) ROS_ERROR_STREAM("Desc test " << ri << ":"  << i << " and " << cur_robot_id << ":" << cur_loop_test_id << " dist " << desc_score);
		// diff_matrix(cur_loop_test_id, i) = desc_score;
		// base transform
		Eigen::Quaterniond q_rel = MapDatabase[cur_robot_id][cur_loop_test_id]->q_world_base.inverse()*MapDatabase[ri][i]->q_world_base;
		Eigen::Vector3d t_rel = MapDatabase[cur_robot_id][cur_loop_test_id]->q_world_base.inverse()*(MapDatabase[ri][i]->t_world_base - MapDatabase[cur_robot_id][cur_loop_test_id]->t_world_base);
		double angle_rel = q_rel.angularDistance(Eigen::Quaterniond::Identity());
		Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f try_guess = Eigen::Matrix4f::Identity();
		try_guess.block(0,0,3,3) = q_rel.normalized().toRotationMatrix().cast<float>();
		try_guess.block(0,3,3,1) = t_rel.cast<float>();
		LoopScore loop_score(desc_score, (float)angle_rel, ri, i, init_guess/*update_guess*/, try_guess);
		loop_scores.push_back(loop_score);
	}
	// ROS_WARN_STREAM("Diff Mat \n" << diff_matrix.row(cur_loop_test_id));
}

void testMatchCloud(pcl::PointCloud<PointType>::Ptr cloud_src, pcl::PointCloud<PointType>::Ptr cloud_target, int ri_test, int si_test, const Eigen::Matrix4f& try_guess, float& final_score, Eigen::Matrix4f& final_guess) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src_d1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_d1(new pcl::PointCloud<pcl::PointXYZI>);
	// downsample
	pcl::ApproximateVoxelGrid<PointType> avg;
	avg.setLeafSize(0.8f, 0.8f, 0.8f); // trade off
	// avg.setLeafSize(0.5f, 0.5f, 0.5f); // trade off
	avg.setInputCloud(cloud_src);
	avg.filter(*cloud_src);
	avg.setInputCloud(cloud_target);
	avg.filter(*cloud_target);

	avg.setLeafSize(1.8f, 1.8f, 1.8f); // trade off
	avg.setInputCloud(cloud_src);
	avg.filter(*cloud_src_d1);
	avg.setInputCloud(cloud_target);
	avg.filter(*cloud_target_d1);

	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setMeanK (5);
	sor.setStddevMulThresh (1.0);
	sor.setInputCloud (cloud_src_d1);
	sor.filter (*cloud_src_d1);
	sor.setInputCloud (cloud_target_d1);
	sor.filter (*cloud_target_d1);

	
	Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
	

	// compute init guess using plane norm
	Eigen::Vector4f norm_src = MapDatabase[ri_test][si_test]->coeffs;
	Eigen::Vector4f norm_src_tmp = norm_src;
	norm_src_tmp(3) = 0;
	norm_src_tmp.normalize();
	norm_src.head(3) = norm_src_tmp.head(3);
	Eigen::Vector4f norm_target = MapDatabase[cur_robot_id][cur_loop_test_id]->coeffs;
	Eigen::Vector4f norm_target_tmp = norm_target;
	norm_target_tmp(3) = 0;
	norm_target_tmp.normalize();
	norm_target.head(3) = norm_target_tmp.head(3);
	// angle between two ground plane normal
	Eigen::Quaternionf q_temp;
	q_temp.setFromTwoVectors(norm_src.head(3), norm_target.head(3));

	// if you equipment is installed 45 degree down facing, this will help to find the real norm direction.
	if (is_drone[cur_robot_id] == true && is_drone[ri_test] == false) {
		Eigen::Quaternionf q_check;
		q_check.setFromTwoVectors(norm_target.head(3), Eigen::Vector3f(0,0,1));
		if(q_check.angularDistance(Eigen::Quaternionf::Identity())*57.3 < 90) {
			q_temp.setFromTwoVectors(norm_src.head(3), -1*norm_target.head(3));
		}
	} else if (is_drone[cur_robot_id] == true && is_drone[ri_test] == true) {
		Eigen::Quaternionf q_check_src, q_check_tar;
		q_check_src.setFromTwoVectors(norm_src.head(3), Eigen::Vector3f(0,0,1));
		q_check_tar.setFromTwoVectors(norm_target.head(3), Eigen::Vector3f(0,0,1));
		int filp_src = q_check_src.angularDistance(Eigen::Quaternionf::Identity())*57.3 < 90 ? -1:1;
		int filp_tar = q_check_tar.angularDistance(Eigen::Quaternionf::Identity())*57.3 < 90 ? -1:1;
		q_temp.setFromTwoVectors(filp_src*norm_src.head(3), filp_tar*norm_target.head(3));
	}

	//ROS_WARN_STREAM("Q src tar " << norm_src.transpose() << " " << norm_target.transpose());

	// translation part using pc centroid
	Eigen::Matrix<float,4,1> centroid_src;
	pcl::compute3DCentroid(*cloud_src, centroid_src);
	centroid_src = centroid_src - norm_src_tmp*centroid_src.dot(norm_src); // project centroid on plane
	centroid_src.head(3) = q_temp*centroid_src.head(3);    

	Eigen::Matrix<float,4,1> centroid_target;
	pcl::compute3DCentroid(*cloud_target, centroid_target);
	centroid_target = centroid_target - norm_target_tmp*centroid_target.dot(norm_target);

	init_guess.topLeftCorner(3,3) = q_temp.toRotationMatrix();
	init_guess.topRightCorner(3,1) = (centroid_target-centroid_src).head(3);
	// ROS_WARN_STREAM("init guess \n" << init_guess);



	// try geometry guess
	pcl::NormalDistributionsTransform<PointType, PointType> ndt1;
	ndt1.setResolution (NDT_RESOLUTION_MATCH);
	// ndt1.setMaximumIterations(20);
	ndt1.setInputSource (cloud_src_d1);
	ndt1.setInputTarget (cloud_target_d1);
	pcl::PointCloud<PointType>::Ptr cloud_final_ndt(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_final_ndt1(new pcl::PointCloud<PointType>);
	ndt1.align(*cloud_final_ndt, init_guess);
	Eigen::Matrix4f cur_guess = ndt1.getFinalTransformation();
	float cur_score = ndt1.getFitnessScore();


	// try world frame guess
	// 如果还没有对齐的时候，使用单位阵进行ndt结果很差，可以不用这个猜测进行对齐
	float cur_score_w = 1000;
	Eigen::Matrix4f cur_guess_w = Eigen::Matrix4f::Identity();
	if (!need_init_coord) {
		ndt1.align(*cloud_final_ndt1, try_guess);
		cur_guess_w = ndt1.getFinalTransformation();
		cur_score_w = ndt1.getFitnessScore();
	}

	final_score = cur_score < cur_score_w? cur_score : cur_score_w;
	final_guess = cur_score < cur_score_w? cur_guess : cur_guess_w;

	pcl::PointCloud<PointType>::Ptr cloud_final_icp(new pcl::PointCloud<PointType>);
	pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp2;
	icp2.setInputSource(cloud_src_d1);
	icp2.setInputTarget(cloud_target_d1);
	icp2.setMaximumIterations(50);
	if(!need_init_coord || !is_drone[cur_robot_id]) icp2.setMaxCorrespondenceDistance(5);

	icp2.align(*cloud_final_icp, final_guess);

	if (icp2.hasConverged()&&icp2.getFitnessScore() < final_score) {
		final_guess = icp2.getFinalTransformation();
		final_score = icp2.getFitnessScore();
	}

	if(DEBUG) ROS_INFO_STREAM("init guess ndt score: " << cur_score << ", try guess score: " << cur_score_w << ", icp score: " << icp2.getFitnessScore());
	
}

void thumbnail_process () {
	while(true) {
		mTProcess.lock();
		if (cur_thumbnail_id < cur_map_id) {
			if (!lastnode_created) {
				empty_loop = 0;
			}
			// mTProcess.lock();
			clock_t start,end;
			start = clock();
			MapDatabase[cur_robot_id][cur_thumbnail_id]->generateThumbnail(true);

			// BUG，补充检查是否包含缩略图
			if(MapDatabase[cur_robot_id][cur_thumbnail_id]->thumbnails_db[0].first == -1 )
			{
				if(DEBUG) ROS_WARN_STREAM("no thumbnail get!");
				continue;
			}

			end = clock();
			ofstream outfile;
			outfile.open(std::string(std::getenv("HOME")) + "/gacm_output/timecost/thumbnail_time.txt", ios::app);
            outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
            outfile.close();

			// call netvlad service
			netvlad_tf_test::CompactImg img_srv;
			img_srv.request.req_img_name = MapDatabase[cur_robot_id][cur_thumbnail_id]->thumbnails_db[0].second;
			int id_tn = MapDatabase[cur_robot_id][cur_thumbnail_id]->thumbnails_db[0].first;
			start = clock();
			if (netvlad_client.call(img_srv)) {
				if(DEBUG) ROS_ERROR("Succeed to call service");
				Eigen::VectorXf desc;
				std2EigenVector(img_srv.response.res_des, desc);

				MapDatabase[cur_robot_id][cur_thumbnail_id]->descriptor_db.push_back(std::make_pair(id_tn, desc));
			} else {
				ROS_ERROR("Failed to call service");
			}
			end = clock();
			outfile.open(std::string(std::getenv("HOME")) + "/gacm_output/timecost/descriptor_time.txt", ios::app);
            outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
            outfile.close();
			
			// add submap base for search
			PointType temp_base;
			temp_base.x = (float)(MapDatabase[cur_robot_id][cur_thumbnail_id]->t_world_base.x());
			temp_base.y = (float)(MapDatabase[cur_robot_id][cur_thumbnail_id]->t_world_base.y());
			temp_base.z = (float)(MapDatabase[cur_robot_id][cur_thumbnail_id]->t_world_base.z());
			temp_base.intensity = (float)(100*cur_robot_id+cur_thumbnail_id);
			submap_base_map[cur_robot_id]->points.push_back(temp_base);

			if(DEBUG) ROS_ERROR_STREAM("\n\n\n\n\n\n++++++ thumbnail " << cur_thumbnail_id << "\n is published \nsubmap num " << submap_base_map[cur_robot_id]->points.size()<<"\n\n\n\n");
			cur_thumbnail_id++;
			
			// mTProcess.unlock();
		} else {
			
			// continue;
			// ROS_ERROR_STREAM("process loop ok");
			if (cur_loop_test_id < cur_thumbnail_id) {
				if (!lastnode_created) {
					empty_loop = 0;
				}

				int big_loop_num = 0; // homo + hetero
				//search for submaps to detect
				update_submapbase_cloud();
				pcl::PointCloud<PointType>::Ptr all_submap_bases(new pcl::PointCloud<PointType> ());

				clock_t start,end;
				start = clock();

				// first deal with homogeneous loop detection
				std::vector<int> knn_idx;
				std::vector<float> knn_dist;
				PointType temp_base;
				temp_base.x = (float)(MapDatabase[cur_robot_id][cur_loop_test_id]->t_world_base.x());
				temp_base.y = (float)(MapDatabase[cur_robot_id][cur_loop_test_id]->t_world_base.y());
				temp_base.z = (float)(MapDatabase[cur_robot_id][cur_loop_test_id]->t_world_base.z());
				temp_base.intensity = (float)(cur_loop_test_id);

				// int k = need_init_coord ? 10 : 5;
				if(DEBUG) ROS_INFO_STREAM("start homo test");
				*all_submap_bases += *submap_base_map[cur_robot_id];
				int k = 5; // for homogeneous
				if (all_submap_bases->size() > 0) {
					kdtree->setInputCloud(all_submap_bases);
					kdtree->nearestKSearch(temp_base,k,knn_idx,knn_dist);
				}
				loop_scores.clear();
				testBaseScore(all_submap_bases, knn_idx, knn_dist);


				// try find possible loop (NDT+GICP)
				std::sort(loop_scores.begin(), loop_scores.end(), cmp); // increase order
				// test best match, once loop found, early stop
				bool find_homo_loop = false;
				
				for (int i = 0; i < 1 && i < loop_scores.size(); i++) {
					//todo decide loop score threshold
					// 飞机可以放弃inter-loop？感觉不是很可靠
					if (find_homo_loop == false && loop_scores[i].loop_score < 0.9 && loop_scores[i].angle_rel*57.3 < 60) {
						

						if (loop_scores.size() >= 3) {
							if (loop_scores[i].loop_score/loop_scores[loop_scores.size()-1].loop_score > 0.9) {
								break;
							}
						}
						// accept


						int ri_test = loop_scores[i].robot_id;
						int si_test = loop_scores[i].submap_id;
						if(DEBUG) ROS_ERROR_STREAM("Homo Matching "<< ri_test << ":" << si_test <<" ; " << cur_loop_test_id<<" with score " << loop_scores[i].loop_score <<" with angle " << loop_scores[i].angle_rel*57.3);
						//visualization
						cv::Mat query_tn = cv::imread(MapDatabase[cur_robot_id][cur_loop_test_id]->thumbnails_db[0].second);
						cv::Mat response_tn = cv::imread(MapDatabase[ri_test][si_test]->thumbnails_db[0].second);
						
						pcl::PointCloud<PointType>::Ptr cloud_src = MapDatabase[ri_test][si_test]->getMapCloud();
						pcl::PointCloud<PointType>::Ptr cloud_target = MapDatabase[cur_robot_id][cur_loop_test_id]->getMapCloud();


						clock_t start_icp, end_icp;
						start_icp = clock();
						
						float final_score;
						Eigen::Matrix4f final_guess;
						testMatchCloud(cloud_src, cloud_target, ri_test, si_test, loop_scores[i].try_guess, final_score, final_guess);

						pcl::PointCloud<PointType>::Ptr cloud_final_icp(new pcl::PointCloud<PointType>);
						pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp2;
						icp2.setInputSource(cloud_src);
						icp2.setInputTarget(cloud_target);
						icp2.setMaximumIterations(50);
						icp2.setMaxCorrespondenceDistance(5);
						icp2.align(*cloud_final_icp, final_guess);
						
						end_icp = clock();
						ofstream outfile1;
						outfile1.open(std::string(std::getenv("HOME")) + "/gacm_output/timecost/single_match_homo.txt", ios::app);
						if ((double)(end_icp-start_icp)/CLOCKS_PER_SEC > 0.5) {
							outfile1 << (double)(end_icp-start_icp)/CLOCKS_PER_SEC << "\n";
						}
						outfile1.close();

						if (icp2.hasConverged()) {
							final_score = icp2.getFitnessScore(5);
							if(DEBUG) cout << "\nFitness score final" << final_score << std::endl;
							// save loop closure edge
							Eigen::Matrix3d R_rel = icp2.getFinalTransformation().topLeftCorner(3,3).cast<double>();
							Eigen::Quaterniond q_rel;
							q_rel = R_rel;
							Eigen::Vector3d t_rel = icp2.getFinalTransformation().topRightCorner(3,1).cast<double>();

							MeasurementEdge measurement_edge(MapDatabase[cur_robot_id][cur_loop_test_id]->pose_graph_local[0].stamp,
								MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
								cur_robot_id, ri_test, 
								cur_loop_test_id, si_test,
								0, 0, // base node to base node
								q_rel, t_rel);

							// 这里无人机可以严格一点，因为好像inter-loop不是很可靠
							if (final_score < (is_drone[cur_robot_id]? 0.3:0.5)) {
								m_loop.lock();

								loopEdgeBuf.push_back(measurement_edge);

								m_loop.unlock();

								find_homo_loop = true;
								big_loop_num ++;
								if (cur_robot_id != ri_test) {
									need_init_coord = false;
								}
								cout << "\033[1;32m\nDone add edge homo! robot[" << ri_test << ":" << si_test << "] => robot[" << cur_robot_id << ":" << cur_loop_test_id << "]\n\033[0m\n";

								pubLoopImage(query_tn, response_tn, true);

								
							} else { // reject by icp score
								dropEdgeBuf.push_back(measurement_edge);
								if(DEBUG) ROS_WARN_STREAM("drop edge score");
								pubLoopImage(query_tn, response_tn, false);
							}
						} else { //reject by icp convergence
							if(DEBUG) ROS_WARN_STREAM("Not converge");
							MeasurementEdge measurement_edge(MapDatabase[cur_robot_id][cur_loop_test_id]->pose_graph_local[0].stamp,
									MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
									cur_robot_id, ri_test, 
									cur_loop_test_id, si_test,
									0, 0, // base node to base node
									Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
							dropEdgeBuf.push_back(measurement_edge);
							if(DEBUG) ROS_WARN_STREAM("drop edge converge(" << loop_scores[i].angle_rel * 57.3 << ")");
							pubLoopImage(query_tn, response_tn, false);
						}

					} else { //reject by geometry
						if (loop_scores[i].loop_score < 0.9 ) {

							int ri_test = loop_scores[i].robot_id;
							int si_test = loop_scores[i].submap_id;
							MeasurementEdge measurement_edge(MapDatabase[cur_robot_id][cur_loop_test_id]->pose_graph_local[0].stamp,
									MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
									cur_robot_id, ri_test, 
									cur_loop_test_id, si_test,
									0, 0, // base node to base node
									Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
							dropEdgeBuf.push_back(measurement_edge);
							if(DEBUG) ROS_WARN_STREAM("drop edge orientation");
							cv::Mat query_tn = cv::imread(MapDatabase[cur_robot_id][cur_loop_test_id]->thumbnails_db[0].second);
							cv::Mat response_tn = cv::imread(MapDatabase[ri_test][si_test]->thumbnails_db[0].second);
							pubLoopImage(query_tn,response_tn, false);
						}
					} // end if candidate accept 

				}
				// end homo loopscores

				if(DEBUG) ROS_INFO_STREAM("start hetro test");
				// for heterogeneous
				all_submap_bases->clear();
				for (int r = 0; r < cur_robot_id; r++) {
					// if (is_drone[r] != is_drone[cur_robot_id]) {
						*all_submap_bases += *submap_base_map[r];
					// }
				}
				knn_idx.clear();
				knn_dist.clear();

				k = 20;
				if (need_init_coord) {
					k = 200;
				}
				if (all_submap_bases->size() > 0) {
					kdtree->setInputCloud(all_submap_bases);
					kdtree->nearestKSearch(temp_base,k,knn_idx,knn_dist);
				}
				loop_scores.clear();
				testBaseScore(all_submap_bases, knn_idx, knn_dist);

				// try find possible loop (NDT+GICP)
				std::sort(loop_scores.begin(), loop_scores.end(), cmp); // increase order
				// test best match, once loop found, early stop
				bool find_hetero_loop = false;
				for (int i = 0; i < 1 && i < loop_scores.size(); i++) {
					//todo decide loop score threshold
					// 飞机可以在角度上放宽要求
					if (find_hetero_loop == false && loop_scores[i].loop_score < 0.9 && (need_init_coord || is_drone[cur_robot_id] || loop_scores[i].angle_rel*57.3 < 60 )) {
						// accept

						if (loop_scores.size() >= 3) {
							if (loop_scores[i].loop_score/loop_scores[loop_scores.size()-1].loop_score > 0.9) {
								break;
							}
						}

						int ri_test = loop_scores[i].robot_id;
						int si_test = loop_scores[i].submap_id;
						if(DEBUG) ROS_ERROR_STREAM("Hetero Matching "<< ri_test << " : " << si_test <<";" << cur_loop_test_id<<" with score " << loop_scores[i].loop_score <<" with angle " << loop_scores[i].angle_rel*57.3);
						//visualization
						cv::Mat query_tn = cv::imread(MapDatabase[cur_robot_id][cur_loop_test_id]->thumbnails_db[0].second);
						cv::Mat response_tn = cv::imread(MapDatabase[ri_test][si_test]->thumbnails_db[0].second);
						
						// debug icp
						pcl::PointCloud<PointType>::Ptr cloud_src = MapDatabase[ri_test][si_test]->getMapCloud();
						pcl::PointCloud<PointType>::Ptr cloud_target = MapDatabase[cur_robot_id][cur_loop_test_id]->getMapCloud();

						
						clock_t start_icp, end_icp;
						start_icp = clock();

						float final_score;
						Eigen::Matrix4f final_guess;
						testMatchCloud(cloud_src, cloud_target, ri_test, si_test, loop_scores[i].try_guess, final_score, final_guess);

						pcl::PointCloud<PointType>::Ptr cloud_final_icp(new pcl::PointCloud<PointType>);
						pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp3;
						if (need_init_coord == false || !is_drone[cur_robot_id]) {
							icp3.setMaximumIterations(60);
							icp3.setMaxCorrespondenceDistance(3);  // hetero test shouldn't set distance threshold
							icp3.setRANSACIterations(10);
						}
						icp3.setInputSource(cloud_src);
						icp3.setInputTarget(cloud_target);
						icp3.align(*cloud_final_icp, final_guess);


						end_icp = clock();
						ofstream outfile1;
						outfile1.open(std::string(std::getenv("HOME")) + "/gacm_output/timecost/single_match_hetero.txt", ios::app);
						if ((double)(end_icp-start_icp)/CLOCKS_PER_SEC > 0.5) {
							outfile1 << (double)(end_icp-start_icp)/CLOCKS_PER_SEC << "\n";
						}
						outfile1.close();

						if (icp3.hasConverged()) {
							final_score = icp3.getFitnessScore(5);
							if(DEBUG) ROS_INFO_STREAM("robot[" << ri_test << ":" << si_test << "] => robot[" << cur_robot_id << ":" << cur_loop_test_id << "] Fitness score final: " << final_score << "\n");
							// save loop closure edge
							Eigen::Matrix3d R_rel = icp3.getFinalTransformation().topLeftCorner(3,3).cast<double>();
							Eigen::Quaterniond q_rel;
							q_rel = R_rel;
							Eigen::Vector3d t_rel = icp3.getFinalTransformation().topRightCorner(3,1).cast<double>();

							MeasurementEdge measurement_edge(MapDatabase[cur_robot_id][cur_loop_test_id]->pose_graph_local[0].stamp,
								MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
								cur_robot_id, ri_test, 
								cur_loop_test_id, si_test,
								0, 0, // base node to base node
								q_rel, t_rel);
							
							// 无人机初始化的时候可以放松要求
							// 除了初始化，如果构成异构回环的都是无人机，那么还需要更严格的要求
							if (final_score < (is_drone[cur_robot_id] && need_init_coord ? (is_drone[ri_test]? 0.8:0.9) : (is_drone[cur_robot_id] && is_drone[ri_test]? 0.65:0.75))) {

								m_loop.lock();
								// int accept;
								// std::cin >> accept;
								// if (accept == 1) {
									loopEdgeBuf.push_back(measurement_edge);
								// } 
								m_loop.unlock();
								// performOptimization();
								find_hetero_loop = true;
								big_loop_num++;
								std::cout << "\033[1;32m\nDone add edge hetero! robot[" << ri_test << ":" << si_test << "] => robot[" << cur_robot_id << ":" << cur_loop_test_id << "]\n\033[0m\n";
								need_init_coord = false;

								std::string file_pefix = std::getenv("HOME");
								file_pefix += "/gacm_output/cache/r" + std::to_string(cur_robot_id) + "s" + std::to_string(cur_loop_test_id) + "-r" + std::to_string(ri_test) + "s" + std::to_string(si_test) + "-";
								pcl::io::savePCDFileASCII(file_pefix + "target.pcd", *cloud_target);
								pcl::io::savePCDFileASCII(file_pefix + "source.pcd", *cloud_src);
								pcl::io::savePCDFileASCII(file_pefix + "final.pcd", *cloud_final_icp);

								pubLoopImage(query_tn, response_tn, true);

								
							}
							else { // reject by icp score
								dropEdgeBuf.push_back(measurement_edge);
								if(DEBUG) ROS_WARN_STREAM("(hetero loop)drop edge score");
								pubLoopImage(query_tn,response_tn, false);
							}
						} else { //reject by icp convergence

							// int ri_test = loop_scores[i].robot_id;
							// int si_test = loop_scores[i].submap_id;
							MeasurementEdge measurement_edge(MapDatabase[cur_robot_id][cur_loop_test_id]->pose_graph_local[0].stamp,
									MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
									cur_robot_id, ri_test, 
									cur_loop_test_id, si_test,
									0, 0, // base node to base node
							Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
							if(DEBUG) ROS_WARN_STREAM("(hetero loop)Not converge");

							pubLoopImage(query_tn,response_tn, false);
						}

					} else { //reject by geometry
						if (loop_scores[i].loop_score < 1.0) {
							int ri_test = loop_scores[i].robot_id;
							int si_test = loop_scores[i].submap_id;
							MeasurementEdge measurement_edge(MapDatabase[cur_robot_id][cur_loop_test_id]->pose_graph_local[0].stamp,
									MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
									cur_robot_id, ri_test, 
									cur_loop_test_id, si_test,
									0, 0, // base node to base node
									Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
							dropEdgeBuf.push_back(measurement_edge);
							if(DEBUG) ROS_WARN_STREAM("(hetero loop)drop edge orientation(" << loop_scores[i].angle_rel*57.3 <<")");
							cv::Mat query_tn = cv::imread(MapDatabase[cur_robot_id][cur_loop_test_id]->thumbnails_db[0].second);
							cv::Mat response_tn = cv::imread(MapDatabase[ri_test][si_test]->thumbnails_db[0].second);
							pubLoopImage(query_tn,response_tn, false);
						}
					} // end if candidate accept 

				} 
				// end hetero loopscores


				end = clock();
				ofstream outfile;
				outfile.open(std::string(std::getenv("HOME")) + "/gacm_output/timecost/loop_time.txt", ios::app);
				if ((double)(end-start)/CLOCKS_PER_SEC > 0.5) {
					outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
				}
				outfile.close();

				if(DEBUG) ROS_INFO_STREAM("start adjacent test");
				// for adjacent submap
				if (cur_loop_test_id > 1) {
					clock_t start_icp, end_icp;
					start_icp = clock();

					pcl::PointCloud<PointType>::Ptr cloud_final_icp3(new pcl::PointCloud<PointType>);
					pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp3;
					pcl::PointCloud<PointType>::Ptr cloud_src = MapDatabase[cur_robot_id][cur_loop_test_id-1]->getMapCloud();
					pcl::PointCloud<PointType>::Ptr cloud_target = MapDatabase[cur_robot_id][cur_loop_test_id]->getMapCloud();
					if(DEBUG) ROS_INFO_STREAM("done fetch " << cloud_src->size() << " : " << cloud_target->size());
					pcl::ApproximateVoxelGrid<PointType> avg;
					avg.setLeafSize(0.8f, 0.8f, 0.8f); // trade off
					// avg.setLeafSize(0.5f, 0.5f, 0.5f); // trade off
					avg.setInputCloud(cloud_src);
					avg.filter(*cloud_src);
					avg.setInputCloud(cloud_target);
					avg.filter(*cloud_target);
					icp3.setInputSource(cloud_src);
					icp3.setInputTarget(cloud_target);
					icp3.setMaxCorrespondenceDistance(5);
					int ri_test = cur_robot_id;
					int si_test = cur_loop_test_id-1;
					Eigen::Quaterniond q_rel = MapDatabase[cur_robot_id][cur_loop_test_id]->q_world_base.inverse()*MapDatabase[ri_test][si_test]->q_world_base;
					Eigen::Vector3d t_rel = MapDatabase[cur_robot_id][cur_loop_test_id]->q_world_base.inverse()*(MapDatabase[ri_test][si_test]->t_world_base - MapDatabase[cur_robot_id][cur_loop_test_id]->t_world_base);
					Eigen::Matrix4f try_guess = Eigen::Matrix4f::Identity();
					try_guess.block(0,0,3,3) = q_rel.normalized().toRotationMatrix().cast<float>();
					try_guess.block(0,3,3,1) = t_rel.cast<float>();
					if(DEBUG) ROS_INFO_STREAM("done guess");
					icp3.align(*cloud_final_icp3, try_guess);
					if(DEBUG) ROS_INFO_STREAM("done icp");
					end_icp = clock();
					ofstream outfile1;
					outfile1.open(std::string(std::getenv("HOME")) + "/gacm_output/timecost/single_match_adj.txt", ios::app);
					outfile1 << (double)(end_icp-start_icp)/CLOCKS_PER_SEC << "\n";
					outfile1.close();
					
					if (icp3.hasConverged()) {
							float final_score = icp3.getFitnessScore(5);
							if(DEBUG) cout << "\nFitness score final adj" << final_score << std::endl;
							// save loop closure edge
							Eigen::Matrix3d R_rel = icp3.getFinalTransformation().topLeftCorner(3,3).cast<double>();
							Eigen::Quaterniond q_rel_;
							q_rel_ = R_rel;
							Eigen::Vector3d t_rel_ = icp3.getFinalTransformation().topRightCorner(3,1).cast<double>();

							MeasurementEdge measurement_edge(MapDatabase[cur_robot_id][cur_loop_test_id]->pose_graph_local[0].stamp,
								MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
								cur_robot_id, ri_test, 
								cur_loop_test_id, si_test,
								0, 0, // base node to base node
								q_rel_, t_rel_);

							if (final_score < 0.9) {
								m_loop.lock();

								loopEdgeBuf.push_back(measurement_edge);

								m_loop.unlock();
								// performOptimization();
								if(DEBUG) ROS_WARN_STREAM("done add edge adjacent");
							} else { // reject by icp score
								if(DEBUG) ROS_WARN_STREAM("drop edge adj");
							}
						} else { //reject by icp convergence
							if(DEBUG) ROS_WARN_STREAM("Not converge");
						}
				}
				// end adjacent test
				
				std::thread t(publish_opt_posegraph, false);
					t.detach();
					publish_opt_posegraph(false);
				if (big_loop_num > 0 || loopEdgeBuf.size() >= 3 || lastnode_created) {
					m_loop.lock();
					performOptimization(); // check need optimize inside function
					m_loop.unlock();
				} else {
				}
				cur_loop_test_id++;
				if(DEBUG) ROS_ERROR_STREAM("done thumnail loop");
				if (lastnode_created) {
					save_all();
					if (cur_robot_id >= 1) {
						// export_odom();
						if(DEBUG) std::cout << "\033[1;31m\nodom before opt exported, please check\n\033[0m";
						globalOptimization();
					}
					export_odom();

					cout << "\033[1;32m\nSave odom and global optimization Done! \n\033[0m";
					cout << "\033[1;32m\nCurrent robot[" << cur_robot_id << "] end, ready to receive robot[" << cur_robot_id+1 << "] data!\n\033[0m";
				}
			} else {
				if(is_first)
				{
					mTProcess.unlock();
					std::chrono::milliseconds dura(2);
					std::this_thread::sleep_for(dura);
					continue;
				}

				empty_loop++;
				// empty loop, no further operation, should start a new session
				if (empty_loop >= 10000 && !lastnode_created) {
					if(DEBUG) ROS_ERROR_STREAM("LAST node!!!!!!!!!!!!!!!!!!!!!!\n!!!!!!!!!!!!!!!!!!!!!!!\n!!!!!!!!!!!!!!!!!!!!!!!!!!\n!!!!!!!!!!!!!!!!!!!!!!!!\n!!!!!!!!!!!!");
					
					m_loop.lock();
					if(DEBUG) ROS_ERROR_STREAM("loop lock ok");
					MapDatabase[cur_robot_id].push_back(MapDatabase[cur_robot_id][cur_map_id]->createNewSubMap()); // check, is parameter enough?
					cur_map_id++;
					

					CONFIG_ID++;
					if(DEBUG) ROS_ERROR_STREAM("Config id " << CONFIG_ID);
					lastnode_created = true;
					std_msgs::Empty empty;
					pubNewSessionSignal.publish(empty);
					m_loop.unlock();
					// empty_loop = 0; dont't need to prevent pause case

					is_first = true;
				}
			}
		}
		mTProcess.unlock();
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	} // end loop
}









void update_submapbase_cloud() {
	if(DEBUG) std::cout << "\nStart update submapbase\n";
	
	for (int ri = 0; ri <= cur_robot_id; ri++) {
		int temp_submap_size = MapDatabase[ri].size();
		if (ri == cur_robot_id && temp_submap_size > cur_loop_test_id) {
			temp_submap_size = cur_loop_test_id;
		}
		submap_base_map[ri].reset(new pcl::PointCloud<PointType>());
		// submap_base_map[ri]->points.clear();
		for (int si = 0; si < temp_submap_size; si++) {
			// don't use future submap or lastnode for loop detection
			if (MapDatabase[ri][si]->thumbnailGenerated == false) {
				continue;
			}
			PointType temp_base;
			temp_base.x = (float)(MapDatabase[ri][si]->t_world_base.x());
			temp_base.y = (float)(MapDatabase[ri][si]->t_world_base.y());
			temp_base.z = (float)(MapDatabase[ri][si]->t_world_base.z());
			temp_base.intensity = (float)(100*ri+si);
			submap_base_map[ri]->points.push_back(temp_base);
			// std::cout << "add " << si <<"\n";

		}
		// std::cout << "\ndone " << ri << "\n";
	}
	if(DEBUG) std::cout << "\nDone update submapbase\n";
}


/**
 * @brief For visualization
 * 
 * @param pubmap 
 */
void publish_opt_posegraph(bool pubmap = false) {
	if (updating_pg == true) {
		return;
	} else {
		updating_pg = true;
	}
	if(DEBUG) std::cout << "Publish posegraph start \n";
	int edge_acc = 0;
	int edge_adj = 0;
	int edge_rej = 0;
	clock_t start, end;
	start = clock();
	// pose graph and loop edge visualization , see ga_posegrph folder
	for (int r = 0; r <= cur_robot_id; r++) {
		nav_msgs::Path optPath; // path after odom
		optPath.header.frame_id = "camera";
		optPath.header.stamp = ros::Time::now();
		int database_size = MapDatabase[r].size();
		// if(DEBUG) ROS_WARN_STREAM("robot[" << r << "] has " << database_size << " submaps");
		for (int i = 0; i < database_size-1; i++) { // the last submap is isolated
			int max_size = MapDatabase[r][i]->pose_graph_local.size();
			// std::cout << "Max size " << max_size << "\n";
			for (int j = 0; j < max_size; j++) {
				// nav_msgs::Odometry laserOdometry;
				geometry_msgs::PoseStamped pose_opt;
				pose_opt.header.frame_id = "camera";
				pose_opt.header.stamp = ros::Time().fromSec(MapDatabase[r][i]->pose_graph_local[j].stamp);
				Eigen::Quaterniond q = MapDatabase[r][i]->pose_graph_local[j].q;
				Eigen::Vector3d t = MapDatabase[r][i]->pose_graph_local[j].t;

				t = MapDatabase[r][i]->t_world_base + MapDatabase[r][i]->q_world_base*t;
				q = MapDatabase[r][i]->q_world_base * q;

				if(!display_frame_cam)
				{
					Eigen::Affine3d T_Cworld_robot = Eigen::Translation3d(t)*q.toRotationMatrix();
					Eigen::Affine3d T_Lworld_robot = Eigen::Affine3d(T_LC) * T_Cworld_robot;
					q = T_Lworld_robot.rotation();
					t = T_Lworld_robot.translation();
				}

				pose_opt.pose.orientation.w = q.w();
				pose_opt.pose.orientation.x = q.x();
				pose_opt.pose.orientation.y = q.y();
				pose_opt.pose.orientation.z = q.z();
				pose_opt.pose.position.x = t.x();
				pose_opt.pose.position.y = t.y();
				pose_opt.pose.position.z = t.z();
				optPath.poses.push_back(pose_opt);
			}
			

			// publish posegraph edges

			visualization_msgs::Marker edge_line_msg;
			edge_line_msg.header.frame_id = "camera";
			edge_line_msg.header.stamp = ros::Time::now();
			edge_line_msg.id = 0;
			edge_line_msg.type = visualization_msgs::Marker::LINE_LIST;
			edge_line_msg.scale.x = 0.3;
			edge_line_msg.color.r = 1.0;
			edge_line_msg.color.g = 1.0;
			edge_line_msg.color.b = 0.0;
			edge_line_msg.color.a = 1.0;
			visualization_msgs::Marker edge_node_msg;
			edge_node_msg.header.frame_id = "camera";
			edge_node_msg.header.stamp = ros::Time::now();
			edge_node_msg.id =1;
			edge_node_msg.type = visualization_msgs::Marker::SPHERE_LIST;
			edge_node_msg.scale.x = 3;
			edge_node_msg.color.r = 0.0;
			edge_node_msg.color.g = 1.0;
			edge_node_msg.color.b = 0.0;
			edge_node_msg.color.a = 1.0;
			visualization_msgs::Marker edge_line_msg1;
			edge_line_msg1.header.frame_id = "camera";
			edge_line_msg1.header.stamp = ros::Time::now();
			edge_line_msg1.id = 2;
			edge_line_msg1.type = visualization_msgs::Marker::LINE_LIST;
			edge_line_msg1.scale.x = 0.15;
			edge_line_msg1.color.r = 1.0;
			edge_line_msg1.color.g = 0.0;
			edge_line_msg1.color.b = 0.0;
			edge_line_msg1.color.a = 1.0;
			visualization_msgs::Marker edge_node_msg1;
			edge_node_msg1.header.frame_id = "camera";
			edge_node_msg1.header.stamp = ros::Time::now();
			edge_node_msg1.id =3;
			edge_node_msg1.type = visualization_msgs::Marker::SPHERE_LIST;
			edge_node_msg1.scale.x = 1.5;
			edge_node_msg1.color.r = 1.0;
			edge_node_msg1.color.g = 0.2;
			edge_node_msg1.color.b = 0.0;
			edge_node_msg1.color.a = 1.0;
			for (int index = 0; index < loopEdgeBuf.size(); index++) {
				int ri_from = loopEdgeBuf[index].robot_from;
				int ri_to = loopEdgeBuf[index].robot_to;
				int si_from = loopEdgeBuf[index].submap_from;
				int si_to = loopEdgeBuf[index].submap_to;
				if (ri_from != ri_to || si_from != si_to+1) {
					Eigen::Vector3d t_from = MapDatabase[ri_from][si_from]->t_world_base;
					Eigen::Vector3d t_to = MapDatabase[ri_to][si_to]->t_world_base;
					edge_acc++;

					if(!display_frame_cam)
					{
						t_from = (T_LC * Eigen::Vector4d(t_from[0], t_from[1], t_from[2], 1)).head(3);
						t_to = (T_LC * Eigen::Vector4d(t_to[0], t_to[1], t_to[2], 1)).head(3);
					}

					geometry_msgs::Point p;
					p.x = t_from.x();
					p.y = t_from.y();
					p.z = t_from.z();
					edge_line_msg.points.push_back(p);
					edge_node_msg.points.push_back(p);
					p.x = t_to.x();
					p.y = t_to.y();
					p.z = t_to.z();
					edge_line_msg.points.push_back(p);
					edge_node_msg.points.push_back(p);
				} else {
					edge_adj++;
				}
			}

			for (int index = 0; index < dropEdgeBuf.size(); index++) {
				int ri_from = dropEdgeBuf[index].robot_from;
				int ri_to = dropEdgeBuf[index].robot_to;
				int si_from = dropEdgeBuf[index].submap_from;
				int si_to = dropEdgeBuf[index].submap_to;

				
				if (ri_from != ri_to || si_from != si_to+1) {
					Eigen::Vector3d t_from = MapDatabase[ri_from][si_from]->t_world_base;
					Eigen::Vector3d t_to = MapDatabase[ri_to][si_to]->t_world_base;
					edge_rej++;


					if(!display_frame_cam)
					{
						t_from = (T_LC * Eigen::Vector4d(t_from[0], t_from[1], t_from[2], 1)).head(3);
						t_to = (T_LC * Eigen::Vector4d(t_to[0], t_to[1], t_to[2], 1)).head(3);
					}

					geometry_msgs::Point p;
					p.x = t_from.x();
					p.y = t_from.y();
					p.z = t_from.z();
					edge_line_msg1.points.push_back(p);
					edge_node_msg1.points.push_back(p);
					p.x = t_to.x();
					p.y = t_to.y();
					p.z = t_to.z();
					edge_line_msg1.points.push_back(p);
					edge_node_msg1.points.push_back(p);
				}
			}
			
			pubRelEdge.publish(edge_line_msg);
			pubRelEdge.publish(edge_node_msg);
			// pubRelEdge.publish(edge_line_msg1);
			pubRelEdge.publish(edge_node_msg1);
			
			MapDatabase[r][i]->publishMap(0); // don't pub map, update TF only

		}
		pubPathOpt[r].publish(optPath);
		
	}
	end = clock();
	if(DEBUG) std::cout << "Publish posegraph use " << (double)(end-start)/CLOCKS_PER_SEC << "\n";
	updating_pg = false;
	// std::cout << "\nEdges : " << edge_acc << ": " << edge_adj << ": " <<edge_rej << "\n";
}



void publish_opt_map(bool pubmap = false, bool pubmerge = false) {
	if (updating_map == true) {
		return;
	} else {
		updating_map = true;
	}
	if(DEBUG) std::cout << "\nstart visualization\n ";
	pcl::PointCloud<PointType>::Ptr mergeCloudMapG(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr mergeCloudMapA(new pcl::PointCloud<PointType>());
	clock_t start, end;
	start = clock();
	// pose graph and loop edge visualization , see ga_posegrph folder
	for (int r = 0; r <= cur_robot_id; r++) {
		pcl::PointCloud<PointType>::Ptr laserCloudMap(new pcl::PointCloud<PointType>());
		pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>());

		int database_size = MapDatabase[r].size();
		for (int i = 0; i < database_size-1; i++) { // the last submap is isolated
			tempCloud->clear();
			*tempCloud += *(MapDatabase[r][i]->getMapCloud(true));

			if(tempCloud->size() > 0) {
				
				// 这里不要用去除外点，要不然实在太慢了
				if(0)
				{
					pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
					approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05); //downsample map speed up
					approximate_voxel_filter.setInputCloud (tempCloud);
					approximate_voxel_filter.filter (*tempCloud);
				
					pcl::StatisticalOutlierRemoval<PointType> sor; // remove outlier
					sor.setMeanK (5);
					sor.setStddevMulThresh (1.0);
					sor.setInputCloud (tempCloud);
					sor.filter (*tempCloud);
				}

				*laserCloudMap += *tempCloud;
				tempCloud->clear();
			}
			if (pubmap == true) {
				std::string ss = "0";
				ss[0]+=r;
				if(DEBUG) std::cout << "start exporting " << laserCloudMap->size() << std::endl;
			}
			// ROS_WARN_STREAM("pub opt posegraph");
		}
		
		if (pubmerge && laserCloudMap->points.size() > 100) {

			pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1); //downsample map speed up
			approximate_voxel_filter.setInputCloud (laserCloudMap);
			approximate_voxel_filter.filter (*laserCloudMap);
			// std::cout << "Down sampled\n";
			pcl::StatisticalOutlierRemoval<PointType> sor; // remove outlier
			sor.setMeanK (10);
			sor.setStddevMulThresh (1.0);
			sor.setInputCloud (laserCloudMap);
			sor.filter (*laserCloudMap);
			std::string ss = "0";
			ss[0]+=r;
			pcl::io::savePCDFileASCII(std::string(std::getenv("HOME")) + "/gacm_output/data/testSavemap/fullcloud/"+ss+"full.pcd", *laserCloudMap);
			if (r < 3){
				*mergeCloudMapG += *laserCloudMap;
				pcl::io::savePCDFileASCII(std::string(std::getenv("HOME")) + "/gacm_output/data/testSavemap/fullcloud/mergeG.pcd", *mergeCloudMapG);
			} else {
				*mergeCloudMapA += *laserCloudMap;
				pcl::io::savePCDFileASCII(std::string(std::getenv("HOME")) + "/gacm_output/data/testSavemap/fullcloud/mergeA.pcd", *mergeCloudMapA);
			}
			std::cout << "\nSave merge to pcd\n";
		}
		// std::cout << "Sor sampled\n";
		sensor_msgs::PointCloud2 laserCloudMsg;

		if(!display_frame_cam)
		{
			pcl::transformPointCloud(*laserCloudMap, *laserCloudMap, T_LC);
		}

		pcl::toROSMsg(*laserCloudMap, laserCloudMsg);
		laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
		laserCloudMsg.header.frame_id = "/camera";
		pubOptimizedMap[r].publish(laserCloudMsg);
		laserCloudMap->clear();
		if(DEBUG) ROS_INFO_STREAM("pub map [" << r << "]");

	}
	end = clock();
	if(DEBUG) std::cout << "Publish optimized map use " << (double)(end-start)/CLOCKS_PER_SEC << "\n";
	updating_map = false;
}

int last_loop_size = 0;
// fetch posegraph from submaps
void performOptimization() {
	int cur_loop_size = loopEdgeBuf.size();
	if (cur_loop_size-last_loop_size==0) {
		return;
	}

	ceres::Problem::Options problem_options;
	ceres::Problem problem(problem_options);
	ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
	Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6,6>::Identity();
	// add_loop_num = 0;
	for (int ri = 0; ri <= cur_robot_id; ri++) {
		int current_database_size = MapDatabase[ri].size();
		for (int i = 0; i < current_database_size; i++) {
			int current_node_size, current_edge_size;

			// parameter block for base node
			problem.AddParameterBlock(MapDatabase[ri][i]->q_world_base.coeffs().data(),4,q_parameterization);
			problem.AddParameterBlock(MapDatabase[ri][i]->t_world_base.data(),3);
			if (ri == 0 && i == 0) {
				// set fix the first base
				problem.SetParameterBlockConstant(MapDatabase[ri][i]->q_world_base.coeffs().data());
				problem.SetParameterBlockConstant(MapDatabase[ri][i]->t_world_base.data());
			}
			// parameter block for local node
			// fetch node and add parameter block
			current_node_size = MapDatabase[ri][i]->pose_graph_local.size();
			for (int node_id = 0; node_id < current_node_size; node_id++) {
				problem.AddParameterBlock(MapDatabase[ri][i]->pose_graph_local[node_id].q.coeffs().data(),4, q_parameterization);
				problem.AddParameterBlock(MapDatabase[ri][i]->pose_graph_local[node_id].t.data(),3);
				if (node_id==0) {
					// set fix every start node
					problem.SetParameterBlockConstant(MapDatabase[ri][i]->pose_graph_local[node_id].q.coeffs().data());
					problem.SetParameterBlockConstant(MapDatabase[ri][i]->pose_graph_local[node_id].t.data());
				}
			}


			// add residual block for consistancy, the last node of the former submap shuoud be the same as the first node of current submap
			if (i > 0) {
				ceres::CostFunction *cost_function = PoseGraph3dErrorTermWorld::Create(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), cov);
				problem.AddResidualBlock(cost_function, NULL, 
					MapDatabase[ri][i-1]->pose_graph_local.back().q.coeffs().data(),MapDatabase[ri][i-1]->pose_graph_local.back().t.data(),
					MapDatabase[ri][i-1]->q_world_base.coeffs().data(),MapDatabase[ri][i-1]->t_world_base.data(),
					MapDatabase[ri][i]->pose_graph_local.front().q.coeffs().data(),MapDatabase[ri][i]->pose_graph_local.front().t.data(),
					MapDatabase[ri][i]->q_world_base.coeffs().data(),MapDatabase[ri][i]->t_world_base.data());
			}

			// fetch edge and add residual block
			current_edge_size = MapDatabase[ri][i]->edges_local.size();
			for (int edge_id = 0; edge_id < current_edge_size; edge_id++) {
				ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(MapDatabase[ri][i]->edges_local[edge_id].q, MapDatabase[ri][i]->edges_local[edge_id].t, cov);
				problem.AddResidualBlock(cost_function, NULL, 
				MapDatabase[ri][i]->pose_graph_local[MapDatabase[ri][i]->edges_local[edge_id].index_from].q.coeffs().data(),
				MapDatabase[ri][i]->pose_graph_local[MapDatabase[ri][i]->edges_local[edge_id].index_from].t.data(),
				MapDatabase[ri][i]->pose_graph_local[MapDatabase[ri][i]->edges_local[edge_id].index_to].q.coeffs().data(), 
				MapDatabase[ri][i]->pose_graph_local[MapDatabase[ri][i]->edges_local[edge_id].index_to].t.data());
			}

		}

	}

	
	for (int i = 0; i < cur_loop_size; i++) {
		int r_id_f = loopEdgeBuf[i].robot_from;
		int r_id_t = loopEdgeBuf[i].robot_to;
		int s_id_f = loopEdgeBuf[i].submap_from;
		int s_id_t = loopEdgeBuf[i].submap_to;
		ceres::CostFunction *cost_function = 
			PoseGraph3dErrorTermWorld::Create(loopEdgeBuf[i].q, loopEdgeBuf[i].t, cov);
		problem.AddResidualBlock(cost_function, NULL, MapDatabase[r_id_f][s_id_f]->pose_graph_local[0].q.coeffs().data(),
													  MapDatabase[r_id_f][s_id_f]->pose_graph_local[0].t.data(),
													  MapDatabase[r_id_f][s_id_f]->q_world_base.coeffs().data(),
													  MapDatabase[r_id_f][s_id_f]->t_world_base.data(),
													  MapDatabase[r_id_t][s_id_t]->pose_graph_local[0].q.coeffs().data(),
													  MapDatabase[r_id_t][s_id_t]->pose_graph_local[0].t.data(),
													  MapDatabase[r_id_t][s_id_t]->q_world_base.coeffs().data(),
													  MapDatabase[r_id_t][s_id_t]->t_world_base.data());
	}


	if (cur_loop_size >= 1) {
		clock_t start, end;
		start = clock();
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
		options.dynamic_sparsity = true;
		// options.linear_solver_type = ceres::DENSE_QR;
		options.max_num_iterations = 100;
		options.num_threads = 8;
		options.minimizer_progress_to_stdout = false;
		
		if(DEBUG) ROS_WARN_STREAM("Start Optimizing");
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		if(DEBUG) 
		{
			std::cout << summary.BriefReport() << std::endl;
			ROS_WARN_STREAM("Pose graph solved");
		}
		last_loop_size = cur_loop_size;
		end = clock();

		ofstream outfile;
		outfile.open(std::string(std::getenv("HOME")) + "/gacm_output/timecost/pgo_time.txt", ios::app);
		outfile << (double)(end-start)/CLOCKS_PER_SEC << "\n";
		outfile.close();
	}

}


void globalOptimization() {
	if(cur_robot_id < 1) return;

	int cmd;

	std::cout << "\033[1;36mNeed Optimization? (\"0\" for no, other key for yes)\n\033[0m";
	std::cin >> cmd;
	if (cmd == 0) {
		return;
	}

	// 先统计一共要处理多少数据
	int data_length = 0, now_length = 0;
	for(int i = 0; i <= cur_robot_id; i++)
		data_length += (MapDatabase[i].size()-1)*2;

	std::cout << "\033[1;32mBegin global optimization!\033[0m\n";
	int cnt = 0;
	for (int ri = 1; ri <= cur_robot_id; ri++) {
		for (int si = 0; si < MapDatabase[ri].size()-1; si++) {
			for (int is_homo = 0; is_homo <=1; is_homo++) {
				// if (ri_t == ri) {
				// 	continue;
				// }
				// 显示处理进度
				now_length++;
				if(!DEBUG) status(40, (1.0f*now_length) / data_length);

				pcl::PointCloud<PointType>::Ptr all_submap_bases(new pcl::PointCloud<PointType> ());
				all_submap_bases->clear();
				// 拿到所有机器人子图信息
				for (int r = 0; r <= cur_robot_id; r++) {
					if (ri != r && (is_homo ==1? is_drone[ri]==is_drone[r] :is_drone[ri]!=is_drone[r])) {

						*all_submap_bases += *submap_base_map[r];
					}
				}
				if(all_submap_bases->size() == 0) {
					continue;
				}
				// search neighbour
				std::vector<int> knn_idx;
				std::vector<float> knn_dist;
				PointType temp_base;
				temp_base.x = (float)(MapDatabase[ri][si]->t_world_base.x());
				temp_base.y = (float)(MapDatabase[ri][si]->t_world_base.y());
				temp_base.z = (float)(MapDatabase[ri][si]->t_world_base.z());
				temp_base.intensity = (float)(si);
				knn_idx.clear();
				knn_dist.clear();
				int k = 1;
				kdtree->setInputCloud(all_submap_bases);
				kdtree->nearestKSearch(temp_base,k,knn_idx,knn_dist);

				for (int knnid = 0; knnid < knn_idx.size(); knnid++) {
					int ri_test = 0;
					int si_test = all_submap_bases->points[knn_idx[knnid]].intensity;
					ri_test = si_test/100; // robot id
					si_test = si_test%100; // submap_id
					
					pcl::PointCloud<PointType>::Ptr cloud_final_icp3(new pcl::PointCloud<PointType>);
					pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp3;
					pcl::PointCloud<PointType>::Ptr cloud_src = MapDatabase[ri_test][si_test]->getMapCloud();
					pcl::PointCloud<PointType>::Ptr cloud_target = MapDatabase[ri][si]->getMapCloud();
					if(DEBUG) ROS_INFO_STREAM("done fetch " << cloud_src->size() << " : " << cloud_target->size());
					pcl::ApproximateVoxelGrid<PointType> avg;
					avg.setLeafSize(0.8f, 0.8f, 0.8f); // trade off
					avg.setInputCloud(cloud_src);
					avg.filter(*cloud_src);
					avg.setInputCloud(cloud_target);
					avg.filter(*cloud_target);
					icp3.setInputSource(cloud_src);
					icp3.setInputTarget(cloud_target);
					icp3.setMaxCorrespondenceDistance((is_drone[ri]?5:3));	// 这里也需要更严格的限制，除了无人机
					
					Eigen::Quaterniond q_rel = MapDatabase[ri][si]->q_world_base.inverse()*MapDatabase[ri_test][si_test]->q_world_base;
					Eigen::Vector3d t_rel = MapDatabase[ri][si]->q_world_base.inverse()*(MapDatabase[ri_test][si_test]->t_world_base - MapDatabase[ri][si]->t_world_base);
					if (t_rel.norm() > 50) {
						continue;
					}
					Eigen::Matrix4f try_guess = Eigen::Matrix4f::Identity();
					try_guess.block(0,0,3,3) = q_rel.normalized().toRotationMatrix().cast<float>();
					try_guess.block(0,3,3,1) = t_rel.cast<float>();
					if(DEBUG) ROS_INFO_STREAM("done guess");
					icp3.align(*cloud_final_icp3, try_guess);
					if(DEBUG) ROS_INFO_STREAM("done icp");

					if (icp3.hasConverged()) {
						float final_score = icp3.getFitnessScore(5);
						if(DEBUG) cout << "\nFitness score final adj" << final_score << std::endl;
						// save loop closure edge
						Eigen::Matrix3d R_rel = icp3.getFinalTransformation().topLeftCorner(3,3).cast<double>();
						Eigen::Quaterniond q_rel_;
						q_rel_ = R_rel;
						Eigen::Vector3d t_rel_ = icp3.getFinalTransformation().topRightCorner(3,1).cast<double>();

						MeasurementEdge measurement_edge(MapDatabase[ri][si]->pose_graph_local[0].stamp,
							MapDatabase[ri_test][si_test]->pose_graph_local[0].stamp,
							ri, ri_test, 
							si, si_test,
							0, 0, // base node to base node
							q_rel_, t_rel_);
						if (final_score < (is_drone[ri]? (is_drone[ri_test]? 0.65:0.8):0.65)) { // 0.9 // 这里需要更加严格的条件，除了无人机
							// int accept;
							// std::cin >> accept;
							if (std::find(loopEdgeBuf.begin(), loopEdgeBuf.end(), measurement_edge) == loopEdgeBuf.end()) {
								m_loop.lock();
								loopEdgeBuf.push_back(measurement_edge);
								cnt++;

								// if (accept == 1) {
								// } 
								m_loop.unlock();
								// performOptimization();
								if(DEBUG) ROS_WARN_STREAM("done add edge");
								publish_opt_posegraph();
								break;						// 添加回环边后即可退出
							} 
						}
					}
				}
			}
		}
	}
	if(!DEBUG)
		{ status(40, 1); std::cout << std::endl;}
	if(DEBUG) ROS_WARN_STREAM(cnt << " global Edges added");
	performOptimization();
	publish_opt_map(false, true);
}


void checkCacheableMap() {
	if(DEBUG) std::cout << "\n Check cache\n ";
	for (int ri = 0; ri <= cur_robot_id; ri++) {
		int temp_submap_size = MapDatabase[ri].size();
		if (ri == cur_robot_id) {
			temp_submap_size = cur_loop_test_id;
		}
		
		if(DEBUG) cout << "\033[1;32m\nrobot[" << ri << "] submap num: " << MapDatabase[ri].size() << ", cur loop test id: " << cur_loop_test_id << "\n\033[0m";
		for (int si = 0; si < temp_submap_size; si++) {
			if (si < temp_submap_size-2 && MapDatabase[ri][si]->access_status < temp_submap_size-2) {
				MapDatabase[ri][si]->setCache();
			}
			// std::cout << "\n Check memory " << sizeof(*MapDatabase[ri][si]) << "\n";
		}
	}
}

void cache_process()
{
	unsigned int cnt = 0;
	while (1) {
		if (cnt % 3 == 0) {
			checkCacheableMap();
			publish_opt_map(false);
			cnt = 1;
		} else {
			cnt++;
		}

		publish_opt_posegraph(false);
		std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
	}
}


// void pgo_process()
// {	
// 	// opt_mutex.lock() //need also lock the write operation in submap
// 	while(1) {
// 		m_loop.lock();
// 		if (!loopEdgeBuf.empty()) {
// 			// ROS_WARN_STREAM("loop ok");

// 			performOptimization();
			
			
// 		}
// 		publish_opt_posegraph();
// 		m_loop.unlock();

// 		std::chrono::milliseconds dura(2);
//         std::this_thread::sleep_for(dura);
// 	}
// 	// opt_mutex.unlock()
// }



int main(int argc, char** argv) {

	printCopyright();

	ros::init(argc, argv, "Mapping");
	ros::NodeHandle nh("~");
	// CONFIG_ID = 4;
	readParameters(nh);
	nh_ref = &nh;
	is_drone[cur_robot_id] = NEED_CHECK_DIRECTION;
	// export_gt();

	MapDatabase[cur_robot_id].push_back(std::shared_ptr<SubMap>(new SubMap(nh))); // first submap
	MapDatabase[cur_robot_id][cur_map_id]->initParameters(cur_robot_id,0,NEED_CHECK_DIRECTION,0); //double check? // first node
	kdtree.reset(new pcl::KdTreeFLANN<PointType>());
	submap_base_map[cur_robot_id].reset(new pcl::PointCloud<PointType>());

	std::string home_dir = std::getenv("HOME");
	std::string data_dir1 = home_dir + "/gacm_output/data/testSavemap/fullcloud/",
				data_dir2 = home_dir + "/gacm_output/data/air_experiment/",		// deprecate
				timelog_dir = home_dir + "/gacm_output/timecost/",
			    img_dir1 = home_dir + "/gacm_output/pictures/submap_img/",
				img_dir2 = home_dir + "/gacm_output/pictures/eachframe/",
				cache_dir1 = home_dir + "/gacm_output/cache/";

	// 删除相关文件夹
	// std::cout << "\033[1;36m\nneed to rm ${HOME}\\gacm_output folder? ( \"1\" for yes, \"0\" for no) \n\033[0m";
	// int permission;
	// cin >> permission;
	// if(permission == 1)
	// {
	// 	system(("rm -rf " + home_dir + "/gacm_output/data/").c_str());
	// 	system(("rm -rf " + home_dir + "/gacm_output/timecose/").c_str());
	// 	system(("rm -rf " + CACHE_PATH).c_str());
	// 	system(("rm -rf " + home_dir + "/gacm_output/pictures/").c_str());
	// }

	// 检查输出文件夹是否存在
	DIR *dir;
	if((dir=opendir(data_dir1.c_str())) == NULL)
	{
		system(("mkdir -p " + data_dir1).c_str());
		assert(opendir(data_dir1.c_str()) != NULL);
	}
	if((dir=opendir(data_dir2.c_str())) == NULL)
	{
		system(("mkdir -p " + data_dir2).c_str());
	}
	if((dir=opendir(timelog_dir.c_str())) == NULL)
	{
		system(("mkdir -p " + timelog_dir).c_str());
	}
	if((dir=opendir(img_dir1.c_str())) == NULL)
	{
		system(("mkdir -p " + img_dir1).c_str());
	}
	if((dir=opendir(img_dir2.c_str())) == NULL)
	{
		system(("mkdir -p " + img_dir2).c_str());
	}
	if((dir=opendir(cache_dir1.c_str())) == NULL)
	{
		system(("mkdir -p " + cache_dir1).c_str());
	}
	if( (CACHE_PATH != cache_dir1) && (dir=opendir(CACHE_PATH.c_str())) == NULL)
	{
		system(("mkdir -p " + CACHE_PATH).c_str());
	}

	// Subscribers
	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, cornerLastHandler);

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, surfLastHandler);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, odometryHandler);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/laser_full_3", 100, fullResHandler);

	// Publishers
	
	for (int i = 0; i < 5; i++) {
		std::stringstream path_topic;
		path_topic << "/path_opt" << i;
		std::stringstream map_topic;
		map_topic << "/optimized_map" << i;
    	pubPathOpt[i] = nh.advertise<nav_msgs::Path>(path_topic.str(), 100);
		pubOptimizedMap[i] = nh.advertise<sensor_msgs::PointCloud2>(map_topic.str(),100);
	}
    pubRelEdge = nh.advertise<visualization_msgs::Marker>("/relative_observation", 100);
	pubNewSessionSignal = nh.advertise<std_msgs::Empty>("/start_new_session",10);

	pubQuery = nh.advertise<sensor_msgs::Image>("/query_thumb",10);
	pubDetectL = nh.advertise<sensor_msgs::Image>("/database_thumb",10);

	// Services
	netvlad_client = nh.serviceClient<netvlad_tf_test::CompactImg>("/compact_image");

	load_all();

	std::thread mapping_thread{ mapping_process };
	std::thread thumbnail_thread{ thumbnail_process };
	std::thread cache_thread{ cache_process };
	// std::thread pgo_thread{pgo_process};

	ros::spin();
	return 0;
}