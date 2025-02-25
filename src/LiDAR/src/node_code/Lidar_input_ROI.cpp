#include <LiDAR/Lidar_declare.h>

void param_ROI(const LiDAR::totalInfoConstPtr& signal) {
    lidar_state = signal->State;

    if(signal->State == 2) {
        ROI_xMax = ROI_xMax_2;
        ROI_yMin = ROI_yMin_2;
        ROI_yMax = ROI_yMax_2;
        ROI_zMin = ROI_zMin_2;
        ROI_zMax = ROI_zMax_2;
    }
    else if(signal->State == 4) {
        ROI_xMin = ROI_xMin_8;
        ROI_xMax = ROI_xMax_4;
        ROI_yMin = ROI_yMin_4;
        ROI_yMax = ROI_yMax_4;
        ROI_zMin = ROI_zMin_re;      
    }
    else if(signal->State == 8) {
        ROI_xMin = ROI_xMin_8;
        ROI_xMax = ROI_xMax_8;
        ROI_yMin = ROI_yMin_8;
        ROI_yMax = ROI_yMax_8;
        ROI_zMax = ROI_zMax_8;
        ROI_zMin = ROI_zMin_re; 
    }
    else {
        ROI_xMin = ROI_xMin_re;
        ROI_xMax = ROI_xMax_re;
        ROI_yMin = ROI_yMin_re;
        ROI_yMax = ROI_yMax_re;
        ROI_zMin = ROI_zMin_re;
        ROI_zMax = ROI_zMax_re;
    }
}

void ROI_process(const sensor_msgs::PointCloud2ConstPtr& scan) {
    RT::start();
    ROI(scan, lidar_state);
    RT::end_cal("ROI");
}

void TunnelOut(const sensor_msgs::PointCloud2ConstPtr& scan){
    if(lidar_state == 4){
        PCXYZI p;
        PCXYZI Left;
        PCXYZI Right;
        PCXYZI Finalpt;

        pcl::fromROSMsg(*scan, p);
        for (int i = 0; i < p.size(); i++){
            if (p[i].z > 1.5 && p[i].z <= 2.0 && p[i].y > 0.6 && p[i].y < 10.0 && p[i].x > 0.5 && p[i].x < 20.0 ){
                p[i].z = 0;
                Left.push_back(p[i]);
            }
            else if(p[i].z > 1.0 && p[i].z <= 2.0 && p[i].y < -0.6 &&  p[i].y > -10.0 && p[i].x > 0.5 && p[i].x < 20.0){
                p[i].z = 0;
                Right.push_back(p[i]);
            }
        }
        if(Left.size() != 0 && Right.size() != 0){
            pcl::VoxelGrid<PXYZI> sor;
            PCXYZI::Ptr Left_ptr (new PCXYZI());
            *Left_ptr = Left;
            PCXYZI::Ptr Right_ptr (new PCXYZI());
            *Right_ptr = Right;
            sor.setInputCloud(Left_ptr);
            sor.setLeafSize(0.1f, 0.1f, 0.1f); 
            PCXYZI::Ptr NewCloud_L (new PCXYZI());
            sor.filter(*NewCloud_L);

            sor.setInputCloud(Right_ptr);
            sor.setLeafSize(0.1f, 0.1f, 0.1f); 
            PCXYZI::Ptr NewCloud_R (new PCXYZI());
            sor.filter(*NewCloud_R);

            // NewCloud_L과 NewCloud_R을 x 기준으로 정렬
            // std::sort(NewCloud_L->points.begin(), NewCloud_L->points.end(), [](const PXYZI &a, const PXYZI &b) {
            //     return a.x < b.x;
            // });
            std::sort(NewCloud_R->points.begin(), NewCloud_R->points.end(), [](const PXYZI &a, const PXYZI &b) {
                return a.x < b.x;
            });

            // NewCloud_R의 마지막 포인트를 xMax_R로 저장
            PXYZI xMin_R = NewCloud_R->points.front();
            PXYZI xMax_R = NewCloud_R->points.back();
            Finalpt.push_back(xMax_R);
            // xMax_R과 거리가 가장 가까운 NewCloud_L의 포인트를 xMax_L로 저장
            PXYZI xMax_L;

            float min_dist = std::numeric_limits<float>::max();
            if(Finalpt.size() != 0){
                for (const auto& point : NewCloud_L->points) {
                    float dist = sqrt((point.x - xMax_R.x)*(point.x - xMax_R.x) + (point.y - xMax_R.y)*(point.y - xMax_R.y));
                    if (dist < min_dist) {
                        min_dist = dist;
                        xMax_L = point;
                    }
                }
                Finalpt.push_back(xMax_L);

                LiDAR::tunneloutpoint_arr TOP;
                TOP.ptNum = Finalpt.size();

                for (int i = 0; i < Finalpt.size(); i++){
                    LiDAR::tunneloutpoint top;
                    top.x = Finalpt[i].x;
                    top.y = Finalpt[i].y;
                    TOP.top_arr.push_back(top);
                }
                pub_top.publish(TOP);    


                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(Finalpt, output);
                output.header.frame_id = "map"; // 또는 적절한 프레임 ID로 변경
                pub_top_PCL.publish(output);
            }
        }
    }
}

int main(int argc, char** argv){
	ros::init(argc, argv, "input_ROI");         //node name 
	ros::NodeHandle nh;                         //nodehandle
    
    nh.getParam("/ROI_node/switch_ROI", switch_ROI);
    nh.getParam("/ROI_node/ROI_xMin", ROI_xMin);
    nh.getParam("/ROI_node/ROI_xMax", ROI_xMax);
    nh.getParam("/ROI_node/ROI_yMin", ROI_yMin);
    nh.getParam("/ROI_node/ROI_yMax", ROI_yMax);
    nh.getParam("/ROI_node/ROI_zMin", ROI_zMin);
    nh.getParam("/ROI_node/ROI_zMax", ROI_zMax);
    nh.getParam("/ROI_node/ylimit", ylimit);
    nh.getParam("/ROI_node/ROI_zMin_2", ROI_zMin_2);
    nh.getParam("/ROI_node/ROI_zMax_2", ROI_zMax_2);
    nh.getParam("/ROI_node/ROI_xMax_2", ROI_xMax_2);
    nh.getParam("/ROI_node/ROI_yMin_2", ROI_yMin_2);
    nh.getParam("/ROI_node/ROI_yMax_2", ROI_yMax_2);
    nh.getParam("/ROI_node/ROI_xMax_4", ROI_xMax_4);
    nh.getParam("/ROI_node/ROI_yMin_4", ROI_yMin_4);
    nh.getParam("/ROI_node/ROI_yMax_4", ROI_yMax_4);
    nh.getParam("/ROI_node/ROI_xMin_8", ROI_xMin_8);
    nh.getParam("/ROI_node/ROI_xMax_8", ROI_xMax_8);
    nh.getParam("/ROI_node/ROI_yMin_8", ROI_yMin_8);
    nh.getParam("/ROI_node/ROI_yMax_8", ROI_yMax_8);
    nh.getParam("/ROI_node/ROI_zMax_8", ROI_zMax_8);
    nh.getParam("/ROI_node/ROI_xMin_re", ROI_xMin_re);
    nh.getParam("/ROI_node/ROI_xMax_re", ROI_xMax_re);
    nh.getParam("/ROI_node/ROI_yMin_re", ROI_yMin_re);
    nh.getParam("/ROI_node/ROI_yMax_re", ROI_yMax_re);
    nh.getParam("/ROI_node/ROI_zMin_re", ROI_zMin_re);
    nh.getParam("/ROI_node/ROI_zMax_re", ROI_zMax_re);

    ros::Subscriber sub_totalInfo = nh.subscribe<LiDAR::totalInfo>("/Totalcom", 1, param_ROI);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/0_1_GE_others", 1, ROI_process);
    // ros::Subscriber sub_tunnel = nh.subscribe<sensor_msgs::PointCloud2>("/0_1_GE_others", 1, TunnelOut);
     // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/0_UDP_others", 1, ROI_process); 
     
    pub_ROI = nh.advertise<sensor_msgs::PointCloud2>("/1_ROI_PCL2", 1);
    // pub_top = nh.advertise<LiDAR::tunneloutpoint_arr>("/TOP",1);
    // pub_top_PCL = nh.advertise<sensor_msgs::PointCloud2>("/TOP_PCL",1);


    ros::spin();
    return 0;
}
