#include <LiDAR/Lidar_declare.h>

void signal_process(const LiDAR::totalInfoConstPtr& signal) {
    switch_RanSaC = (signal->State == 4) ? true : false;
}

void ransac_process(const sensor_msgs::PointCloud2ConstPtr& aft_ROI){
    //
    RT::start();
    PCXYZI::Ptr RANSAC_cloud (new PCXYZI);
    pcl::fromROSMsg(*aft_ROI,*RANSAC_cloud);

    if( switch_RanSaC ) {//if switch_Ransac is true, tunnel extract
    
        LiDAR::RANSAC_points_arr output_ransac;
        LiDAR::RANSAC_points tmp;

        EXTRACT_TUNNEL(RANSAC_cloud);//YJ

        for(int i = 0; i < RANSAC_cloud->points.size();i++){
                tmp.Rx = RANSAC_cloud->points[i].x;
                tmp.Ry = RANSAC_cloud->points[i].y;
                tmp.Rz = RANSAC_cloud->points[i].intensity;
                tmp.Ri = RANSAC_cloud->points[i].intensity;
                output_ransac.data.push_back(tmp);
        }
        pub_RS_RPA.publish(output_ransac);
        RT::end_cal("RANSAC");
    }
    else{
        sensor_msgs::PointCloud2 output; 
        pub_process(*RANSAC_cloud, output); 
        pub_RS.publish(output);
        PCXYZI empty_cloud;
        pub_process(empty_cloud, output);
        pub_walls.publish(output);

        LiDAR::RANSAC_points_arr output_ransac;
        LiDAR::RANSAC_points tmp;
        for(int i = 0; i < RANSAC_cloud->points.size(); i++) {
                tmp.Rx = RANSAC_cloud->points[i].x;
                tmp.Ry = RANSAC_cloud->points[i].y;
                tmp.Rz = RANSAC_cloud->points[i].z;
                tmp.Ri = RANSAC_cloud->points[i].intensity;
                output_ransac.data.push_back(tmp);
        }
        pub_RS_RPA.publish(output_ransac);
       printf("\033[33mRANSAC runtime\033[0m : \033[1;35mOFF\033[0m\n");
    }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ransac"); //node name 
	ros::NodeHandle nh;         //nodehandle

    nh.getParam("/ransac_node/switch_RanSaC", switch_RanSaC);
    nh.getParam("/ransac_node/ransac_distanceThreshold", ransac_distanceThreshold);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/1_ROI_PCL2", 1, ransac_process);
    ros::Subscriber sub_totalInfo = nh.subscribe<LiDAR::totalInfo> ("/Totalcom", 1, signal_process);
    pub_RS = nh.advertise<sensor_msgs::PointCloud2> ("/2_1_RANSAC_others_PCL2", 1);
    pub_walls = nh.advertise<sensor_msgs::PointCloud2> ("/2_2_RANSAC_walls_PCL2", 1);
    pub_RS_RPA = nh.advertise<LiDAR::RANSAC_points_arr> ("/2_3_RANSAC_points_RPA", 1);
    pub_coeff = nh.advertise<LiDAR::coeff_arr>("/RANSAC_coeff",1);

    pub_top = nh.advertise<LiDAR::tunneloutpoint_arr>("/TOP",1);
    pub_top_PCL = nh.advertise<sensor_msgs::PointCloud2>("/TOP_PCL",1);
    
	ros::spin();
}
