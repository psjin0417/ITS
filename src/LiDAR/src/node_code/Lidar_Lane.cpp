#include <LiDAR/Lidar_declare.h>

void signal_process(const LiDAR::totalInfoConstPtr& signal) {
    // switch_lane = (signal->State == 4 || signal->State == 1 || signal->State == -1) ? true : false;
    // switch_lane_tunnel = (signal->State ==4 || signal->State ==-1) ? true : false;
    if (signal->State == 4 || signal->State == -1 || signal->State == 1 ){
        switch_lane = true;
        if(signal->State == 4 || signal->State == -1 ){
            switch_lane_tunnel = true;
            intensity_min = intensity_min_1;
            intensity_max = intensity_max_1;
        }
        else if(signal->State == 1 ){
            switch_lane_tunnel = false;
            intensity_min = intensity_min_2;
            intensity_max = intensity_max_2;
        }
    }
    else{
        switch_lane = false;
        switch_lane_tunnel =false;
    }


}

void Lane_process(const sensor_msgs::PointCloud2ConstPtr& scan){
    
    if(switch_lane){
        RT::start();
        Lane_Detection(scan);
        RT::end_cal("LaneDetection");
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "LaneDetection");
    ros::NodeHandle nh;

    nh.getParam("/Lane_node/switch_lane", switch_lane);
    nh.getParam("/Lane_node/switch_lane_tunnel", switch_lane_tunnel);
    nh.getParam("/Lane_node/intensity_min", intensity_min);
    nh.getParam("/Lane_node/intensity_max", intensity_max);
    nh.getParam("/Lane_node/intensity_min_1", intensity_min_1);
    nh.getParam("/Lane_node/intensity_max_1", intensity_max_1);
    nh.getParam("/Lane_node/intensity_min_2", intensity_min_2);
    nh.getParam("/Lane_node/intensity_max_2", intensity_max_2);

    ros::Subscriber sub_totalInfo = nh.subscribe<LiDAR::totalInfo>("/Totalcom", 1, signal_process);
    ros::Subscriber sub_ground = nh.subscribe<sensor_msgs::PointCloud2>("/0_2_GE_ground", 1, Lane_process);

    pub_lane = nh.advertise<sensor_msgs::PointCloud2>("/Lane_PCL", 1);
    pub_lane_coeff = nh.advertise<LiDAR::lane_arr>("/Lane_coeff", 1);
    pub_raw4cam = nh.advertise<LiDAR::raw4cam_arr>("/Lane_point", 1);
    // marker_pub_lane = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // pub_curve = nh.advertise<sensor_msgs::PointCloud2>("/Curve_fitting", 1);
    // pub_coeff_lane = nh.advertise<>("/Lane_visual",1);

    ros::spin();
    return 0;
}