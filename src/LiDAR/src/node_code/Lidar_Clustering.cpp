#include <LiDAR/Lidar_declare.h>

Fps FPS1;
void Coeff_processing(const LiDAR::coeff_arrConstPtr& msg){

    ransac_coeff = msg->coeff_data;
}
void Coeff_precessing2(const LiDAR::lane_arrConstPtr& msg){

    lane_coeff = msg->lane_data;
}
void TOP_processing(const LiDAR::tunneloutpoint_arrConstPtr& msg){

    top = msg->top_arr;
}
void Lane_processing(const LiDAR::raw4cam_arrConstPtr& msg){
    
    PCXYZI::Ptr LaneCloud(new PCXYZI);
    PCXYZI::Ptr LaneCloud_re(new PCXYZI);
    for(int i=0; i<msg->raw_data.size(); i++){
        pcl::PointXYZI point;
        // msg->raw_data[i]의 필드들을 각각 pcl::PointXYZI로 변환
        point.x = msg->raw_data[i].x; // 적절한 필드 이름으로 대체
        point.y = msg->raw_data[i].y;
        point.z = msg->raw_data[i].z;
        // cout << point << endl;
        LaneCloud->points.push_back(point);
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*LaneCloud, output);
    output.header.frame_id = "map"; // 또는 적절한 프레임 ID로 변경
    pub_Midlane.publish(output);

    float sum_y = 0;
    for(int i=0; i< LaneCloud->points.size();i++){
        sum_y += LaneCloud->points[i].y;
    }
    float mean_y  = sum_y / LaneCloud->points.size();
    for(int i=0; i< LaneCloud->points.size();i++){
        if(abs(LaneCloud->points[i].y - mean_y) < 0.5){
            LaneCloud_re->points.push_back(LaneCloud->points[i]);
        }
    }
    // Least Squares Method to fit a line y = ax + b
    int n = LaneCloud_re->points.size();
    for(int i = 0; i < LaneCloud_re->points.size();i++){
        cout << LaneCloud_re->points[i] << endl;
    }
    if (n > 2) {  // 최소 2개 이상의 점이 필요합니다.
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        for (const auto& point : LaneCloud_re->points) {
            double x = point.x;
            double y = point.y;
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }
        double a = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        double b = (sum_y * sum_xx - sum_x * sum_xy) / (n * sum_xx - sum_x * sum_x);
        std::cout << "Fitted line: y = " << a << " * x + " << b << std::endl;
        LiDAR::lane_arr lane_arr;
        LiDAR::lane lane1;
        lane1.Slope = a;
        lane1.Yintercept = b;
        lane1.Angle = atan2(a, 1.0) * (180.0 / M_PI);
        lane_arr.lane_data.push_back(lane1);
        lane_coeff = lane_arr.lane_data;
        //ransac
            // pcl::SACSegmentation<pcl::PointXYZI> seg;
            // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            // seg.setOptimizeCoefficients (true);
            // seg.setModelType (pcl::SACMODEL_LINE); 
            // seg.setMethodType (pcl::SAC_RANSAC); 
            // seg.setDistanceThreshold (0.1);
            // seg.setMaxIterations (5000);
            // seg.setInputCloud(LaneCloud_re);
            // seg.segment(*inliers, *coefficients);

            // double a = (*coefficients).values[4] / (*coefficients).values[3];  //    dy/dx   = - dx/ dy                         
            // double b = (-a*coefficients->values[0] + coefficients->values[1]);
            // std::cout << "Fitted line: y = " << a << " * x + " << b << std::endl;
            // LiDAR::lane_arr lane_arr;
            // LiDAR::lane lane1;
            // lane1.Slope = a;
            // lane1.Yintercept = b;
            // lane1.Angle = atan2(a, 1.0) * (180.0 / M_PI);
            // lane_arr.lane_data.push_back(lane1);
            // lane_coeff = lane_arr.lane_data;

        //for rviz
        pcl::PointCloud<pcl::PointXYZI> Midlinefitting_cloud;
        for (double x = 0.0; x <= 7.0; x += 0.1) {
            pcl::PointXYZI point;
            pcl::PointXYZI point_parallel;
            point.x = x;
            point.y = a * x + b;
            point.z = 0.0;
            point.intensity = 255;
            Midlinefitting_cloud.push_back(point);
            point_parallel.x = x;
            point_parallel.y = a * x + (b-3.4);
            point_parallel.z = 0;
            point_parallel.intensity = 255;
            Midlinefitting_cloud.push_back(point_parallel);
        }
        sensor_msgs::PointCloud2 output2;
        pcl::toROSMsg(Midlinefitting_cloud, output2);
        output2.header.frame_id = "map";  // 또는 적절한 프레임 ID로 변경
        pub_Midlane_line.publish(output2);
    } 
}

void Clustering_process(const sensor_msgs::PointCloud2ConstPtr& aft_ransac){

    PCXYZI::Ptr clustering_cloud (new PCXYZI);
    pcl::fromROSMsg(*aft_ransac,*clustering_cloud);
    PCXYZI Fin_Cloud;
    Clustering(clustering_cloud, Fin_Cloud, switch_DBscan, switch_Euclid);
    ransac_coeff.clear();
    lane_coeff.clear();
    top.clear();
    // LiDAR::raw4cam_arr RAW;
    // RAW.Num = Fin_Cloud.size();
    // for (int i = 0; i < Fin_Cloud.size(); i++){
    //     LiDAR::raw4cam raw;
    //     raw.x = Fin_Cloud[i].x;
    //     raw.y = Fin_Cloud[i].y;
    //     raw.z = Fin_Cloud[i].z;
    //     RAW.raw_data.push_back(raw);
    // }
    // pub_raw4cam.publish(RAW);
    
    FPS1.update();
    printf("\033[2m────────────────────────────");
    printf("────────────────────────────\033[0m\n");

}

void param_clustering(const LiDAR::totalInfoConstPtr& signal) {

    if(signal->State == 1 || signal->State == 9) {//소형
        //장애물, 유턴(PE_Drum)
        DBscan_eps = DBscan_eps_1;
        DBscan_minPts = DBscan_minPts_1;
        DB_MinClusterSize = DB_MinClusterSize_1;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_1;
        switch_Cone_filter = false;
        switch_Surround_filter = false;
        switch_Drum_filter = false;
        switch_Delievery_filter = false;
    }
    else if(signal->State == 2) {
        //배달 (표지판)
        DBscan_eps = DBscan_eps_2;
        DBscan_minPts = DBscan_minPts_2;
        DB_MinClusterSize = DB_MinClusterSize_2;
        switch_jiwon_filter = false;
        switch_Cone_filter = false;
        switch_Surround_filter = false;
        switch_Drum_filter = false;
        switch_Delievery_filter = true;
        switch_find_Rectangle = true;
    }
    else if(signal->State == 3){//대형
        //장애물(차량)
        DBscan_eps = DBscan_eps_3;
        DBscan_minPts = DBscan_minPts_3;
        DB_MinClusterSize = DB_MinClusterSize_3;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_3;
        switch_Cone_filter = false;
        switch_Surround_filter = false;
        switch_Drum_filter = false;
        switch_Delievery_filter = false;
    }
    else if(signal->State == 4) {
        //터널
        DBscan_eps = DBscan_eps_4;
        DBscan_minPts = DBscan_minPts_4;
        DB_MinClusterSize = DB_MinClusterSize_4;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_4;
        switch_Cone_filter = false;
        switch_Surround_filter = false;
        switch_Drum_filter = false;
        switch_Delievery_filter = false;
    }
    else if(signal->State == 8 || signal->State == -1) {
        //협로, 평행주차(콘)
        DBscan_eps = DBscan_eps_8;
        DBscan_minPts = DBscan_minPts_8;
        DB_MinClusterSize = DB_MinClusterSize_8;
        switch_jiwon_filter = true; //false
        switch_Cone_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_8;
        switch_Surround_filter = false; 
        switch_Drum_filter = false;
        switch_Delievery_filter = false;

    }
    else {
        DBscan_eps = DBscan_eps_re;
        DBscan_minPts = DBscan_minPts_re;
        DB_MinClusterSize = DB_MinClusterSize_re;
        switch_jiwon_filter = true;
        REMOVE_FACTOR = REMOVE_FACTOR_re;
        switch_Cone_filter = false;
        switch_Drum_filter = false;
        switch_Surround_filter = false;
        switch_Delievery_filter = false;
        switch_find_Rectangle = false;
    }
}


int main(int argc, char** argv){
	ros::init(argc, argv, "Clustering");    //node name 
	ros::NodeHandle nh;                     //nodehandle    

    //Euclid
    nh.getParam("/Clustering_node/switch_Euclid", switch_Euclid);
    nh.getParam("/Clustering_node/EC_eps", EC_eps);
    nh.getParam("/Clustering_node/EC_MinClusterSize", EC_MinClusterSize);
    nh.getParam("/Clustering_node/EC_MaxClusterSize", EC_MaxClusterSize);
    //DBSCAN
    nh.getParam("/Clustering_node/switch_DBscan", switch_DBscan);
    nh.getParam("/Clustering_node/DBscan_eps", DBscan_eps);
    nh.getParam("/Clustering_node/DBscan_minPts", DBscan_minPts);
    nh.getParam("/Clustering_node/DB_MinClusterSize", DB_MinClusterSize);
    nh.getParam("/Clustering_node/DB_MaxClusterSize", DB_MaxClusterSize);
    //etc
    nh.getParam("/Clustering_node/switch_jiwon_filter", switch_jiwon_filter);
    nh.getParam("/Clustering_node/switch_Cone_filter", switch_Cone_filter);
    nh.getParam("/Clustering_node/switch_Surround_filter", switch_Surround_filter);
    nh.getParam("/Clustering_node/switch_Drum_filter", switch_Drum_filter);
    nh.getParam("/Clustering_node/switch_Delievery_filter", switch_Delievery_filter);
    nh.getParam("/Clustering_node/Surround_Z_ROI", Surround_Z_ROI);  //DY_filter param
    nh.getParam("/Clustering_node/REMOVE_FACTOR", REMOVE_FACTOR);
    //
    nh.getParam("/Clustering_node/switch_find_Rectangle", switch_find_Rectangle);

    //sinal
    nh.getParam("/Clustering_node/REMOVE_FACTOR_1", REMOVE_FACTOR_1);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_3", REMOVE_FACTOR_3);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_4", REMOVE_FACTOR_4);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_8", REMOVE_FACTOR_8);
    nh.getParam("/Clustering_node/REMOVE_FACTOR_re", REMOVE_FACTOR_re);

    nh.getParam("/Clustering_node/DBscan_eps_1", DBscan_eps_1);
    nh.getParam("/Clustering_node/DBscan_minPts_1", DBscan_minPts_1);
    nh.getParam("/Clustering_node/DB_MinClusterSize_1", DB_MinClusterSize_1);
    nh.getParam("/Clustering_node/DBscan_eps_2", DBscan_eps_2);
    nh.getParam("/Clustering_node/DBscan_minPts_2", DBscan_minPts_2);
    nh.getParam("/Clustering_node/DB_MinClusterSize_2", DB_MinClusterSize_2);
    nh.getParam("/Clustering_node/DBscan_eps_3", DBscan_eps_3);
    nh.getParam("/Clustering_node/DBscan_minPts_3", DBscan_minPts_3);
    nh.getParam("/Clustering_node/DB_MinClusterSize_3", DB_MinClusterSize_3);
    nh.getParam("/Clustering_node/DBscan_eps_4", DBscan_eps_4);
    nh.getParam("/Clustering_node/DBscan_minPts_4", DBscan_minPts_4);
    nh.getParam("/Clustering_node/DB_MinClusterSize_4", DB_MinClusterSize_4);
    nh.getParam("/Clustering_node/DBscan_eps_8", DBscan_eps_8);
    nh.getParam("/Clustering_node/DBscan_minPts_8", DBscan_minPts_8);
    nh.getParam("/Clustering_node/DB_MinClusterSize_8", DB_MinClusterSize_8);
    nh.getParam("/Clustering_node/DBscan_eps_re", DBscan_eps_re);
    nh.getParam("/Clustering_node/DBscan_minPts_re", DBscan_minPts_re);
    nh.getParam("/Clustering_node/DB_MinClusterSize_re", DB_MinClusterSize_re);

    ros::Subscriber sub_coeff = nh.subscribe<LiDAR::coeff_arr>("/RANSAC_coeff",1,Coeff_processing);
    ros::Subscriber sub_coeff2 = nh.subscribe<LiDAR::lane_arr>("/Lane_coeff",1,Coeff_precessing2);
    ros::Subscriber sub_tunneloutpoint = nh.subscribe<LiDAR::tunneloutpoint_arr>("/TOP",1,TOP_processing);
    ros::Subscriber sub_miiddle = nh.subscribe<LiDAR::raw4cam_arr>("/MiddleLineDetecion",1,Lane_processing);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/2_1_RANSAC_others_PCL2",1,Clustering_process);
    ros::Subscriber sub_totalInfo = nh.subscribe<LiDAR::totalInfo> ("/Totalcom", 1, param_clustering);
   
    pub_Clu = nh.advertise<sensor_msgs::PointCloud2> ("/4_Clustering_PCL2", 1);
    pub_obj = nh.advertise<LiDAR::object_msg_arr> ("/4_Clustering_OMA", 1);
    pub_LC = nh.advertise<LiDAR::objsInfo> ("/LiDARcom", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes", 1);
    pub_Midlane = nh.advertise<sensor_msgs::PointCloud2> ("/Final_midlane_pt", 1);
    pub_Midlane_line = nh.advertise<sensor_msgs::PointCloud2>("/Lane_line", 1);
    // pub_raw4cam = nh.advertise<LiDAR::raw4cam_arr>("/raw4cam", 1);

    ros::spin();
    return 0;
}
