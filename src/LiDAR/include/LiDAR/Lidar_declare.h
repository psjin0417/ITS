#pragma once

#ifndef LIDAR_DECLARE
#define LIDAR_DECLARE
#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <limits>
#include <algorithm>
#include <string>
#include <time.h>
#include <signal.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>   //noise filtering
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <LiDAR/SDW_DBSCAN.h>
//
#include <LiDAR/object_msg_arr.h>
#include <LiDAR/objsInfo.h>
#include <LiDAR/totalInfo.h>
// #include <LiDAR/GNSSInfo.h>
#include <LiDAR/ground_msg_arr.h>
#include <LiDAR/RANSAC_points_arr.h>
#include <LiDAR/coeff_arr.h>
#include <LiDAR/raw4cam_arr.h>
#include <LiDAR/lane_arr.h>
#include <LiDAR/tunneloutpoint_arr.h>
//---------------------------------------------------
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//---------------------------------------------------
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
//Threading
#include <pthread.h>
#include <mutex>
#include <queue>
#include <atomic>
#include <condition_variable>
//

using namespace std;
using namespace std::chrono;
using Eigen::MatrixXd;
using Eigen::VectorXd;

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> PCXYZI;
typedef pcl::PointXYZ PXYZ;
typedef pcl::PointXYZI PXYZI;

ros::Publisher pub_ROI;     //ROI
ros::Publisher pub_Clu;
ros::Publisher pub_obj;     //user defined msg
ros::Publisher pub_LC;     //user defined msg
ros::Publisher pub_TS;
ros::Publisher pub_DS;
ros::Publisher pub_RS_RPA;
ros::Publisher pub_coeff;//for tunnel
ros::Publisher marker_pub;
ros::Publisher pub_raw4cam; //for camera
ros::Publisher pub_RS;
ros::Publisher pub_walls;
ros::Publisher pub_rawdata;
ros::Publisher pub_ground;
ros::Publisher pub_ground_to_cam;
ros::Publisher pub_others;
ros::Publisher pub_lane;//for lane
ros::Publisher pub_lane_coeff;
ros::Publisher image_pub;//for visual
ros::Publisher marker_pub_lane;
ros::Publisher all_lane_pub;
ros::Publisher pub_curve;
ros::Publisher pub_top_PCL;//tunnet out point
ros::Publisher pub_top;
ros::Publisher pub_Midlane;
ros::Publisher pub_Midlane_line;


//ros::Publisher OUT_MSG;     //out message
int lidar_state;
double REMOVE_FACTOR, REMOVE_FACTOR_re, REMOVE_FACTOR_1, REMOVE_FACTOR_3, REMOVE_FACTOR_4, REMOVE_FACTOR_8;
float voxel_size_x, voxel_size_y, voxel_size_z;
float ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax,
      ROI_xMax_2 ,ROI_yMin_2, ROI_yMax_2, ROI_zMin_2, ROI_zMax_2,
      ROI_xMax_4, ROI_yMin_4, ROI_yMax_4,
      ROI_xMin_8, ROI_xMax_8, ROI_yMin_8, ROI_yMax_8, ROI_zMax_8, ROI_zMin_8,
      ROI_xMin_re, ROI_xMax_re, ROI_yMin_re, ROI_yMax_re, ROI_zMin_re, ROI_zMax_re;
//
float xlimit, ylimit;
int intensity_min, intensity_max, intensity_min_1, intensity_max_1, intensity_min_2, intensity_max_2; 
//
float GE_Z, GE_Z_4, GE_Z_re, GE_slope, GE_delta_slope;
float Surround_Z_ROI;
float DBscan_eps, DBscan_eps_re, DBscan_eps_1, DBscan_eps_2, DBscan_eps_3, DBscan_eps_4, DBscan_eps_8;
float DBscan_minPts, DBscan_minPts_re, DBscan_minPts_1, DBscan_minPts_2, DBscan_minPts_3, DBscan_minPts_4, DBscan_minPts_8;
int DB_MinClusterSize, DB_MinClusterSize_re, DB_MinClusterSize_1, DB_MinClusterSize_2, DB_MinClusterSize_3, DB_MinClusterSize_4, DB_MinClusterSize_8;
int DB_MaxClusterSize;
float EC_eps;
int EC_MinClusterSize, EC_MaxClusterSize;
double ransac_distanceThreshold; //함수 : double형 parameter
bool switch_jiwon_filter;
bool switch_DY_filter;
bool switch_ROI;
bool switch_DownSampling;
bool switch_Euclid;
bool switch_RanSaC;
bool switch_DBscan;
bool switch_visual;
bool switch_visual_2D;
bool switch_Cone_filter;
bool switch_Surround_filter;
bool switch_Drum_filter;
bool switch_Delievery_filter;
bool switch_interior_points;
bool switch_UDP_communication;
bool Sensor_Hesai;
bool Sensor_V16;
bool switch_lane;
bool switch_lane_tunnel;
bool VLP16;

bool switch_find_Rectangle;
//-----------------------------------
int cnt = 0;
double yaw_start = 0;
double yaw_end = 0;
double dyaw;
double yaw_now;

bool shouldExit = false;
int Port_Num;
int BUFFER_SIZE;
string UDP_IP;
PCXYZI rawdata;
PCXYZI others;
PCXYZI ground;
PCXYZI lane;
//-----------------------------
PCXYZI datum;
double each_azimuth = 0.3;
double azimuth_now;
double azimuth_b4;
int k = 0;
int m = 0;
int p = 0;
static const double PI = acos(-1.0);

const double LASER_ANGLES_V16[] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15,
                            -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};//VLP16

const double Vertical_angle[] = {15, 14, 13, 12, 11, 10,  9,  8,  
                                   7, 6,  5,  4,  3,  2,  1,  0,
                                  -1, -2, -3, -4, -5, -6, -7, -8,
                                  -9, -10, -11, -12, -13, -14, -15, -16};//Hesai32

const double LASER_ANGLES[] = {-30.67, -9.33, -29.33, -8.00, -28.00, -6.66, -26.66, -5.33,
                        -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33, 0.00,
                        -20.00, 1.33, -18.67, 2.67, -17.33, 4.00, -16.00, 5.33,
                        -14.67, 6.67, -13.33, 8.00, -12.00, 9.33, -10.67, 10.67 };//VLP32

const double Vertical_angle_Ouster[] = { 19.96, 18.71, 17.44, 16.16, 14.86, 13.53, 12.2, 10.85,
                                         9.48, 8.109999999999999, 6.72, 5.34, 3.94, 2.53, 1.14, -0.27,
                                         -1.67, -3.08, -4.48, -5.88, -7.25, -8.65, -10.01, -11.37,
                                        -12.71, -14.05, -15.35, -16.65, -17.93, -19.19, -20.41, -21.63 };// 라이다 연결 후 TCP command를 이용해서 입력하기
const double Beam_azimuth_angle_Ouster[] = {-4.27, -4.26, -4.25, -4.25, -4.25, -4.25, -4.24, -4.24,
                                            -4.24, -4.23, -4.23, -4.23, -4.23, -4.23, -4.22, -4.22,
                                            -4.21, -4.21, -4.21, -4.21, -4.21, -4.21, -4.19, -4.19,
                                            -4.18, -4.19, -4.18, -4.19, -4.19, -4.17, -4.18, -4.17};
const double BeamToLidar[] = {
    1, 0, 0, 15.806,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1};

//get_beam_intrinsics
                             
struct Point {
    double x, y, z, dist, azimuth;
    uint8_t reflec;
    int laserID;
    int level;//-1은 지면, 1은 object
    Point() {}
    Point(double _x, double _y, double _z, double _dist, uint8_t _reflec, double _azimuth, int _laserID): x(_x), y(_y), z(_z), dist(_dist), reflec(_reflec), azimuth(_azimuth), laserID(_laserID) {}
    // Point(double _x, double _y, double _z, double _dist, uint8_t _reflec, double _azimuth, int _laserID): x(_x), y(_y), z(_z), dist(_dist), reflec(_reflec), azimuth(_azimuth), laserID(_laserID) {}
};

Point cal_point(double dist, double azi, uint8_t re, int ld) {//VLP32
  double omega = LASER_ANGLES[ld] * PI / 180.0;
  double alpha = azi * PI / 180.0;
  double X = dist * cos(omega) * sin(alpha);
  double Y = dist * cos(omega) * cos(alpha);
  double Z = dist * sin(omega);

  return Point(Y, -X, Z, dist, re, azi, ld);
}
Point cal_point_V16(double dist, double azi, uint8_t re, int ld) {//VLP16
  double omega = LASER_ANGLES_V16[ld] * PI / 180.0;
  double alpha = azi * PI / 180.0;
  double X = dist * cos(omega) * sin(alpha);
  double Y = dist * cos(omega) * cos(alpha);
  double Z = dist * sin(omega);

  return Point(Y, -X, Z, dist, re, azi, ld);
}

Point cal_point_H(double dist, double azi, uint8_t re, int ld) {//for hesai
  double omega = Vertical_angle[ld] * PI / 180.0;
  double alpha = azi * PI / 180.0;
  double X = dist * cos(omega) * sin(alpha);
  double Y = dist * cos(omega) * cos(alpha);
  double Z = dist * sin(omega);

  return Point(-Y, X, Z, dist, re, azi, ld);
}

// Point cal_point_Ouster(double dist, double azi, uint8_t re, int ld) {//for hesai
Point cal_point_Ouster(double dist, uint8_t re, int ld, uint16_t MeasureID) {
  double range_offset = 15.806; // unit mm 
  double ScanWidth = 1024;  //1024-20
  
  double ThetaEncoder = 2.f * M_PI * (1.f - (MeasureID / 1024.f));
  double Azimuth_deg = -(Beam_azimuth_angle_Ouster[ld]);
  double Azimuth_rad = (M_PI /180.0f) * Azimuth_deg + ThetaEncoder + M_PI;
  if (Azimuth_rad >= M_PI * 2)
  {
      Azimuth_rad -= 2 * M_PI;
  }
  else if (Azimuth_rad < 0.f)
  {
      Azimuth_rad += 2 * M_PI;
  }
  Azimuth_deg = (180.f/M_PI) * Azimuth_rad;
  // cout << "Azimuth_deg" << Azimuth_deg <<endl;
  // double ThetaAzimuth = -2 * M_PI * (Beam_azimuth_angle_Ouster[ld] / 360);//deg to rad
  
  double Phi = (M_PI/ 180.f) * Vertical_angle_Ouster[ld];


  double X = (dist - range_offset)*cos(Azimuth_rad)*cos(Phi) + 15.806f * cos(ThetaEncoder);
  double Y = (dist - range_offset)*sin(Azimuth_rad)*cos(Phi) + 15.806f * sin(ThetaEncoder);
  double Z = (dist - range_offset) * sin(Phi);
  // double Azimuth = (ThetaEncoder + ThetaAzimuth) /(2*M_PI);
  X /= 1000.f;// unit : mm -> m
  Y /= 1000.f;
  Z /= 1000.f;
  return Point(X, Y, Z, (dist - range_offset), re, Azimuth_deg, ld);

  // double X = dist * cos(Phi) * sin(ThetaEncoder);
  // double Y = dist * cos(Phi) * cos(ThetaEncoder);
  // double Z = dist * sin(Phi);
  // double Azimuth = ThetaEncoder/(2*M_PI);
  // return Point(-Y, X, Z, dist, re, azi, ld);
  
  // return Point(-Y, -X, Z, (dist - range_offset), re, Azimuth, ld);

}

vector<LiDAR::coeff> ransac_coeff;
vector<LiDAR::lane> lane_coeff;
vector<LiDAR::tunneloutpoint> top;

vector<Point> arr;
vector<Point> arr2;
LiDAR::ground_msg_arr ground_cam;

PCXYZI objs_points;

//cluster information struct using at after clustering
struct objectInfo {
    pcl::PointIndices* objPoints;
    string classes;
    unsigned int idx;
    float x;
    float y;
    float z;
    float xMin;
    float yMin;
    float zMin;
    float xMax;
    float yMax;
    float zMax;
    short intensity;
}; //순서 유지 필수

struct pointInfo{
    float x;
    float y;
    float z;
};

//func
void Lane_Detection(const sensor_msgs::PointCloud2ConstPtr&);
void ROI(const sensor_msgs::PointCloud2ConstPtr&, int);
void makeCropBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
void Clustering (PCXYZI::Ptr, PCXYZI&, bool, bool);
void afterClusteringProcess(PCXYZI::Ptr, PCXYZI&, vector<pcl::PointIndices>&);
void DBScanClustering(PCXYZI::Ptr, PCXYZI&);
void EuclideanClustering(PCXYZI::Ptr, PCXYZI&);
void EXTRACT_TUNNEL(PCXYZI::Ptr);
string send_msg_minmax(float, float, float, float);
// void object_msg_process(const vector<struct objectInfo>&);
void object_msg_process(const vector<struct objectInfo>& , pcl::PointCloud<pcl::PointXYZI>::Ptr );
void visualizeBoundingBoxes(const std::vector<objectInfo>&);
//--------------------------
void ground_extraction();
void Parse_rawdata();
void Parsing_lidar_data(const char*);//velodyne 16, 32
void Parsing_Hesai(const char*);//hesai32
void Parsing_Ouster(const char*); //ouster32
//
void Ground_Extraction(const sensor_msgs::PointCloud2ConstPtr&);//for velodyne32
void Ground_Extraction2(const sensor_msgs::PointCloud2ConstPtr& scan);//VLP32 _RANSAC
void Ground_Extraction_H(const sensor_msgs::PointCloud2ConstPtr&);//for Hesai
void Ground_Extraction_V16(const sensor_msgs::PointCloud2ConstPtr&);//for velodyne16
void GE_OusterRaw(const sensor_msgs::PointCloud2ConstPtr&);//for Ouster-RANSAC
//
void RanSaC_signalhead(PCXYZI::Ptr );

class Filter{
public:

    void jiwon_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag);
    void jiwon_filter(vector<objectInfo>& objs,PCXYZI::Ptr, bool flag);
    void jiwon_filter(vector<objectInfo>& objs, bool flag);
    inline bool check_in(PXYZI a, PXYZI b) { return ((abs(a.x - b.x) <= REMOVE_FACTOR) && (abs(a.y - b.y) <= REMOVE_FACTOR)); }
    void generate_return_PointCloud(PCXYZI::Ptr inputCloud, PCXYZI& returnCloud, vector<objectInfo>& objs);
    
    void Cone_filter(vector<objectInfo>& objs, bool flag);
    void Surround_filter(vector<objectInfo>& objs, bool flag);
    void Drum_filter(vector<objectInfo>& objs, bool flag);
    void Delievery_filter(vector<objectInfo>& objs, bool flag);
    //
    void find_Rectangle(vector<objectInfo>& objs,PCXYZI::Ptr, bool flag);

};
Filter FT;

class Fps{
public:
    Fps();
    void update();
private:
    double prev_clock;
    double cur_clock;
    double interval;
    double m_fps;
    size_t m_count;
};

class RT{
public:
    RT();
    static void start();
    static void end_cal(const char*);
private:
    static double prev_clock;
    static double cur_clock;
    static double interval;
};
double RT::prev_clock;
double RT::cur_clock;
double RT::interval;

inline float cal_dist(float x, float y){ return sqrt(x*x+y*y); }
inline double cal_dist_21(double x2, double x1, double y2, double y1) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));}
inline float cal_dist_21(float x2, float x1, float y2, float y1) { return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));}
inline float MidPt(float a, float b){ return (a + b) / 2; }
inline double cal_dist_3(double x2, double x1, double y2, double y1, double z2, double z1){return ((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));}

template<typename T> //this func is used all code
void pub_process(T& input, sensor_msgs::PointCloud2& output){
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(input, tmp_PCL);                     //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2

    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
}

float Distance2D(float x1, float y1,float x2,float y2)
{
  return pow( pow(x1-x2, 2) + pow(y1-y2, 2), 0.5);
}

std::queue<std::vector<char>> dataQueue;
std::mutex queueMutex;
std::condition_variable queueCondVar;

#endif

