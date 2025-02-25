#include <LiDAR/Lidar_declare.h>


void Parsing_GE(const sensor_msgs::PointCloud2ConstPtr& scan){

    RT::start();
    if(Sensor_Hesai == true){
        Ground_Extraction_H(scan);
    }
    else if(Sensor_V16 == true){
        Ground_Extraction_V16(scan);
    }
    else{
        // Ground_Extraction(scan);//VLP32
        Ground_Extraction2(scan);
    }
    // printf("\033[38;2;139;232;229mState\033[0m : \033[1;37m%d\033[0m\n", lidar_state);
    RT::end_cal("GroundExtraction");
}
void state_process(const LiDAR::totalInfoConstPtr& signal) {//for ouster
    lidar_state = signal->State;
}
void ouster_process(const sensor_msgs::PointCloud2ConstPtr& scan){
    RT::start();
    GE_OusterRaw(scan);
    printf("\033[38;2;139;232;229mState\033[0m : \033[1;37m%d\033[0m\n", lidar_state);
    RT::end_cal("GroundExtraction");
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "GE_node");
    ros::NodeHandle nh;
    nh.getParam("/GE_node/Sensor_Hesai", Sensor_Hesai);
    nh.getParam("/GE_node/Sensor_V16", Sensor_V16);
    nh.getParam("/GE_node/GE_Z", GE_Z);
    nh.getParam("/GE_node/GE_Z_re", GE_Z_re);
    nh.getParam("/GE_node/GE_slope", GE_slope);
    nh.getParam("/GE_node/GE_delta_slope", GE_delta_slope);
    ros::Subscriber sub = nh.subscribe<LiDAR::totalInfo>("/Totalcom", 1, state_process);
    // ros::Subscriber sub_ouster = nh.subscribe<sensor_msgs::PointCloud2>("/ouster/points",1,ouster_process);
    ros::Subscriber sub_rawdata = nh.subscribe<sensor_msgs::PointCloud2>("/0_0_UDP_rawdata", 1, Parsing_GE);
    // ros::Subscriber sub_rawdata = nh.subscribe<sensor_msgs::PointCloud2>("/0_UDP_rawdata",1,Parsing_GE);

    pub_others = nh.advertise<sensor_msgs::PointCloud2>("/0_1_GE_others", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/0_2_GE_ground", 1);
    ros::spin();

    return 0;
    
}