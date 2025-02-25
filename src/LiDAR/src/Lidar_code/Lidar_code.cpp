#include <LiDAR/Lidar_declare.h>
#include <omp.h>
#include <pcl/common/pca.h>
#include <map>
#include <tuple>

void Lane_Detection(const sensor_msgs::PointCloud2ConstPtr& scan) { //기본
    pcl::PointCloud<pcl::PointXYZI> rawforlane;
    pcl::fromROSMsg(*scan, rawforlane);
    pcl::PointCloud<pcl::PointXYZI> lane;

    for (int i = 0; i < rawforlane.size(); i++) {// 200 <= 중앙선 <= 255  ,  75 <= 흰 차선 < 150
        // if ( ( (rawforlane[i].intensity >= 75 && rawforlane[i].intensity < 150) || ( rawforlane[i].intensity >= 200 && rawforlane[i].intensity < 255 ) ) &&
        //     rawforlane[i].z < -0.7) {   
        if (((rawforlane[i].intensity >= intensity_min && rawforlane[i].intensity < intensity_max)) // 
            && (rawforlane[i].x > 0.0) && (rawforlane[i].x < 5.0)  
            && (rawforlane[i].z < -0.75)
            && (rawforlane[i].y < 3.0) && (rawforlane[i].y > 0.0)
             ){
            // cout << "intensity = " << rawforlane[i].intensity << endl;
            lane.push_back(rawforlane[i]);
        }
    }
    PCXYZI::Ptr lane_ptr (new PCXYZI ());
    *lane_ptr = lane;

    pcl::VoxelGrid<PXYZI> sor;
    sor.setInputCloud(lane_ptr);
    sor.setLeafSize(0.02f, 0.02f, 0.02f); //0.05f
    PCXYZI::Ptr filteredCloud(new PCXYZI);
    PCXYZI::Ptr filteredCloud_re(new PCXYZI);
    sor.filter(*filteredCloud);
    //remain only convex pt


    //send lane point data to camera
    LiDAR::raw4cam_arr RAW;
    RAW.Num = filteredCloud->points.size();
    for(int i =0; i <filteredCloud->points.size();i++){
        LiDAR::raw4cam raw;
        raw.x = filteredCloud->points[i].x;
        raw.y = filteredCloud->points[i].y;
        raw.z = filteredCloud->points[i].z;
        RAW.raw_data.push_back(raw);
    }
    pub_raw4cam.publish(RAW);
    RAW.raw_data.clear();
    //
    if(switch_lane_tunnel){
        float sum_y = 0;
        for(int i=0; i< filteredCloud->points.size();i++){
            sum_y += filteredCloud->points[i].y;
        }
        float mean_y  = sum_y / filteredCloud->points.size();
        for(int i=0; i< filteredCloud->points.size();i++){
            if(abs(filteredCloud->points[i].y - mean_y) < 1.5){
                filteredCloud_re->points.push_back(filteredCloud->points[i]);
            }
        }
        // Least Squares Method to fit a line y = ax + b
        int n = filteredCloud_re->points.size();
        if (n > 2) {  // 최소 2개 이상의 점이 필요합니다.
            // least square
            double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
            for (const auto& point : filteredCloud_re->points) {
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
            LiDAR::lane lane2;
            lane1.Slope = a;
            lane1.Yintercept = b;
            lane1.Angle = atan2(a, 1.0) * (180.0 / M_PI);
            lane_arr.lane_data.push_back(lane1);
            pub_lane_coeff.publish(lane_arr);

            //ransac
            // pcl::SACSegmentation<pcl::PointXYZI> seg;
            // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            // seg.setOptimizeCoefficients (true);
            // seg.setModelType (pcl::SACMODEL_LINE); 
            // seg.setMethodType (pcl::SAC_RANSAC); 
            // seg.setDistanceThreshold (0.5);//모라이에서는 중앙선이 두개 존재 0.5
            // seg.setMaxIterations (1000);
            // seg.setInputCloud(filteredCloud);
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
            // pub_lane_coeff.publish(lane_arr);
            
            //for rviz
            pcl::PointCloud<pcl::PointXYZI> curve_cloud;
            for (double x = 0.0; x <= 7.0; x += 0.1) {
                pcl::PointXYZI point;
                pcl::PointXYZI point_parallel;
                point.x = x;
                point.y = a * x + b;
                point.z = 0.0;
                point.intensity = 255;
                curve_cloud.push_back(point);
                point_parallel.x = x;
                point_parallel.y = a * x + (b-3.4);
                point_parallel.z = 0;
                point_parallel.intensity = 255;
                curve_cloud.push_back(point_parallel);
            }
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(curve_cloud, output);
            output.header.frame_id = "map";  // 또는 적절한 프레임 ID로 변경
            pub_lane.publish(output);
        } 
    }
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*filteredCloud, output);
    // output.header.frame_id = "map";  // 또는 적절한 프레임 ID로 변경
    // pub_lane.publish(output);
}

void ROI(const sensor_msgs::PointCloud2ConstPtr& scan, int state){

    PCXYZI rawData;
    PCXYZI cropedData;
    pcl::fromROSMsg(*scan, rawData);

    if(switch_ROI) {
        makeCropBox(rawData, ROI_xMin, ROI_xMax, ROI_yMin, ROI_yMax, ROI_zMin, ROI_zMax);
            PCXYZI::Ptr rawData_Ptr (new PCXYZI ());
            *rawData_Ptr = rawData;

            PCXYZI::Ptr except_points (new PCXYZI ());
            PCXYZI::Ptr non_except_points (new PCXYZI ());
            PCXYZI::Ptr x_except_points (new PCXYZI ());
            PCXYZI::Ptr x_non_except_points (new PCXYZI ());
            PCXYZI::Ptr xy_non_except_points_neg (new PCXYZI ());
            PCXYZI::Ptr xy_non_except_points_pos (new PCXYZI ());

            pcl::PassThrough<PXYZI> x_filter;
            pcl::PassThrough<PXYZI> y_filter;

            x_filter.setInputCloud (rawData_Ptr);
            x_filter.setFilterFieldName ("x"); //x limit same 
            x_filter.setFilterLimits (-0.5, 0.5);
            x_filter.filter (*x_except_points);
            x_filter.setFilterLimits (0.5, 20);
            x_filter.filter (*x_non_except_points);

            y_filter.setInputCloud (x_except_points);
            y_filter.setFilterFieldName ("y");
            y_filter.setFilterLimits (-ylimit, ylimit); //for ERP (-0.6, 0.6)  for Morai (-1.0, 1.0)
            y_filter.filter (*except_points);
            y_filter.setFilterLimits(-20, -ylimit); //for ERP (-20, -0.6)  for Mora(-20, -1.0 )
            y_filter.filter (*xy_non_except_points_neg);
            y_filter.setFilterLimits(ylimit, 20); //for ERp (0.6, 20) for Morai(1.0, 20)
            y_filter.filter(*xy_non_except_points_pos);

            *non_except_points = (*x_non_except_points) + (*xy_non_except_points_neg) + (*xy_non_except_points_pos);
            rawData = *non_except_points;
    }

    sensor_msgs::PointCloud2 output;                        //to output ROIdata formed PC2
    pub_process(rawData,output);
    pub_ROI.publish(output);
}

void makeCropBox (PCXYZI& Cloud, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    pcl::CropBox<PXYZI> boxfilter;
    boxfilter.setMin(Eigen::Vector4f(xMin, yMin, zMin, std::numeric_limits<float>::lowest()));
    boxfilter.setMax(Eigen::Vector4f(xMax, yMax, zMax, std::numeric_limits<float>::max()));
    boxfilter.setInputCloud(Cloud.makeShared());
    boxfilter.filter(Cloud);
}

void Clustering (PCXYZI::Ptr inputCloud, PCXYZI& retCloud, bool switch_DBscan, bool switch_Euclid){
    // if(inputCloud->points.size() != 0){    
        if( switch_DBscan ) DBScanClustering( inputCloud, retCloud); //prior DBSCAN
        else if( switch_Euclid ) EuclideanClustering( inputCloud, retCloud );
        else retCloud = *inputCloud; //doesn't process clustering
    // }
    sensor_msgs::PointCloud2 output; 
    pub_process(retCloud,output); 
    pub_Clu.publish(output); 
}
void afterClusteringProcess(PCXYZI::Ptr inputCloud, PCXYZI& retCloud, vector<pcl::PointIndices>& cluster_indices) {//extract column
    vector<pair<PXYZI, string>> sorted_OBJ; // minmax 포함하는 struct 생성 필요
    vector<objectInfo> objs;
    // vector<pointInfo> pts;//for camera

    int intensityValue = 0;
    for (vector<pcl::PointIndices>::iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, intensityValue++) {
        pair<float, float> x(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()); //first = min, second = max
        pair<float, float> y(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
        pair<float, float> z(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
        int pt_num =0 ;
        //for camera
        // PXYZI Zmin_pt, Zmax_pt;
        // PXYZI Ymin_pt, Ymax_pt;
        //
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            PXYZI pt = inputCloud->points[*pit];
            pt.intensity = intensityValue % 10;
            //origin
            if (pt.x < x.first) x.first = pt.x;
            if (pt.x > x.second) x.second = pt.x;
            if (pt.y < y.first) y.first = pt.y;
            if (pt.y > y.second) y.second = pt.y;
            if (pt.z < z.first) z.first = pt.z;
            if (pt.z > z.second) z.second = pt.z;

            // modified version for camer sensor fusion
            // if (pt.x < x.first) x.first = pt.x;
            // if (pt.x > x.second) x.second = pt.x;
            // if (pt.y < y.first) {
            //     y.first = pt.y;
            //     Ymin_pt = pt;
            // }
            // if (pt.y > y.second) {
            //     y.second = pt.y;
            //     Ymax_pt = pt;
            // }
            // if (pt.z < z.first) {
            //     z.first = pt.z;
            //     Zmin_pt = pt;
            // }
            // if (pt.z > z.second) {
            //     z.second = pt.z;
            //     Zmax_pt = pt;
            // }   
            // //
            pt_num++;
        }

        PXYZI* tmp = new PXYZI();
        tmp->x = MidPt(x.first, x.second); 
        tmp->y = MidPt(y.first, y.second); 
        tmp->z = MidPt(z.first, z.second);
        pair<PXYZI, string> temp = make_pair(*tmp, send_msg_minmax(x.first, x.second, y.first, y.second));
        sorted_OBJ.push_back(temp);

        objectInfo tmp_obj = { &(*it), "unknown", (unsigned int)intensityValue,
                            MidPt(x.first, x.second), MidPt(y.first, y.second), MidPt(z.first, z.second),
                            x.first, y.first, z.first, x.second, y.second, z.second,
                            (short)(intensityValue % 10) };
        // pointInfo ZminInfo = {"Points at Zmin", Zmin_pt.x, Zmin_pt.y, Zmin_pt.z};
        // pts.push_back(ZminInfo);
        // pointInfo ZmaxInfo = {"Points at Zmax", Zmax_pt.x, Zmax_pt.y, Zmax_pt.z};
        // pts.push_back(ZmaxInfo);
        // pointInfo YminInfo = {"Points at Ymin", Ymin_pt.x, Ymin_pt.y, Ymin_pt.z};
        // pts.push_back(YminInfo);
        // pointInfo YmaxInfo = {"Points at Ymax", Ymax_pt.x, Ymax_pt.y, Ymax_pt.z};
        // pts.push_back(YmaxInfo);

        objs.push_back(tmp_obj);
    }
    //cout << "------------------Filter------------------" << endl;
    // FT.jiwon_filter(sorted_OBJ, switch_jiwon_filter);
    FT.jiwon_filter(objs,inputCloud, switch_jiwon_filter); //indices vector를 수정하는 filter
    // FT.jiwon_filter(objs,pts,inputCloud, switch_jiwon_filter);
    // FT.jiwon_filter(objs, switch_jiwon_filter);
    FT.Cone_filter(objs, switch_Cone_filter);
    FT.Surround_filter(objs,switch_Surround_filter);
    FT.Drum_filter(objs,switch_Drum_filter);
    FT.Delievery_filter(objs, switch_Delievery_filter);
    FT.find_Rectangle(objs,inputCloud,switch_find_Rectangle);
    FT.generate_return_PointCloud(inputCloud, retCloud, objs);
    
    visualizeBoundingBoxes(objs);
    // object_msg_process(objs);
    object_msg_process(objs,inputCloud);
}
// bounding box를 시각화하는 함수
void visualizeBoundingBoxes(const std::vector<objectInfo>& objs) {
    visualization_msgs::MarkerArray marker_array;
        // 기존의 마커를 모두 삭제
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    for (const auto& obj : objs) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "bounding_boxes";
        marker.id = obj.intensity;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.03;  // 선의 두께
        marker.color.a = 0.3;  // 불투명도
        marker.color.r = 0.0;  // 빨간색 요소
        marker.color.g = 1.0;  // 녹색 요소
        marker.color.b = 0.0;  // 파란색 요소
        // 바운딩 박스 꼭짓점
        Eigen::Vector3f bbox_vertices[8] = {
            {obj.xMin, obj.yMin, obj.zMin},
            {obj.xMax, obj.yMin, obj.zMin},
            {obj.xMin, obj.yMax, obj.zMin},
            {obj.xMax, obj.yMax, obj.zMin},
            {obj.xMin, obj.yMin, obj.zMax},
            {obj.xMax, obj.yMin, obj.zMax},
            {obj.xMin, obj.yMax, obj.zMax},
            {obj.xMax, obj.yMax, obj.zMax}
        };

        // 바운딩 박스의 각 모서리를 연결하는 선 추가
        std::vector<std::pair<int, int>> edges = {
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };

        for (const auto& edge : edges) {
            geometry_msgs::Point p1, p2;
            p1.x = bbox_vertices[edge.first].x();
            p1.y = bbox_vertices[edge.first].y();
            p1.z = bbox_vertices[edge.first].z();
            p2.x = bbox_vertices[edge.second].x();
            p2.y = bbox_vertices[edge.second].y();
            p2.z = bbox_vertices[edge.second].z();
            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }

        marker_array.markers.push_back(marker);
    }
    marker_pub.publish(marker_array);
}

void EuclideanClustering(PCXYZI::Ptr inputCloud, PCXYZI& retCloud){
    pcl::search::KdTree<PXYZI>::Ptr tree (new pcl::search::KdTree<PXYZI>);  // Creating the KdTree for searching PC
    tree->setInputCloud(inputCloud);                     // setting the KdTree

    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    pcl::EuclideanClusterExtraction<PXYZI> ec;           // clustering with Euclidean method
    ec.setInputCloud(inputCloud);   	                 // setting ec with inputCloud
    ec.setClusterTolerance(EC_eps); 	                 // dist between points ..  cur : 30cm
    ec.setMinClusterSize(EC_MinClusterSize);		     // minSize the number of point for clustering
    ec.setMaxClusterSize(EC_MaxClusterSize);	         // minSize the number of point for clustering
    ec.setSearchMethod(tree);				             // searching method : tree 
    ec.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    //cout << "Number of clusters is equal to " << cluster_indices.size() << endl;    //return num of clusteringObj
    afterClusteringProcess(inputCloud, retCloud, cluster_indices);
}

void DBScanClustering(PCXYZI::Ptr inputCloud, PCXYZI& retCloud){
    pcl::VoxelGrid<PXYZI> sor;
    sor.setInputCloud(inputCloud);
    sor.setLeafSize(0.02f, 0.02f, 0.02f); 
    //0.01f = 8fps & 18900size 0.02f = 15fps & 12000 size, 0.03f = 20fps & 9500 size,  0.04f = 20fps & 7900 size,  0.05f = 20fps & 6700 size,  
    PCXYZI::Ptr filteredCloud(new PCXYZI);
    sor.filter(*filteredCloud);

    pcl::search::KdTree<PXYZI>::Ptr tree (new pcl::search::KdTree<PXYZI>);  // Creating the KdTree for searching PC
    if(filteredCloud->points.size() != 0) tree->setInputCloud(filteredCloud);                     // setting the KdTree
    vector<pcl::PointIndices> cluster_indices;           // saving place for clustering obj
    
    DBSCAN<PXYZI> db;
    db.setCorePointMinPts(DBscan_minPts);                // minimum points of cluster judge
    db.setClusterTolerance(DBscan_eps);                  // dist between points
    db.setMinClusterSize(DB_MinClusterSize);		     // minSize the number of point for clustering
    db.setMaxClusterSize(DB_MaxClusterSize);	         // maxSize the number of point for clustering
    db.setSearchMethod(tree);				             // searching method : tree
    db.setInputCloud(filteredCloud);   	                 // setting ec with inputCloud
    db.extract(cluster_indices);                         // save clusteringObj to cluster_indices

    afterClusteringProcess(filteredCloud, retCloud, cluster_indices);
}

void EXTRACT_TUNNEL(PCXYZI::Ptr inputcloud){
    PCXYZI::Ptr cloud_new3 (new(PCXYZI)),
                cloud_new2 (new PCXYZI),
                cloud_origin (new PCXYZI(*inputcloud)),
                cloud_new (new PCXYZI()),
                inlierPoints (new PCXYZI),
                inlierPoints_first(new PCXYZI()), 
                inlierPoints_second(new PCXYZI()),
                inlierPoints_third(new PCXYZI()),
                inlierPoints_neg (new PCXYZI),
                finalpoints (new PCXYZI);
    for(int i =0 ; i < inputcloud->points.size(); i++){
        if(inputcloud->points[i].z > 1.0 && inputcloud->points[i].z < 2.0){
            inputcloud->points[i].z = 0;
            cloud_new->points.push_back(inputcloud->points[i]);
        }
    }                            
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); // point x,y,z, vector x,y,z
    pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients ());
    pcl::ModelCoefficients::Ptr coefficients3 (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices ());
    pcl::PointIndices::Ptr inliers3 (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    LiDAR::coeff_arr coeff_arr;
    LiDAR::coeff coeff1;
    LiDAR::coeff coeff2;
    LiDAR::coeff coeff3;

    LiDAR::tunneloutpoint_arr TOP;
    LiDAR::tunneloutpoint top1;
    LiDAR::tunneloutpoint top2;
    PCXYZI Finalpt;
    pcl::VoxelGrid<PXYZI> sor;

    seg.setOptimizeCoefficients (true);       
    seg.setModelType (pcl::SACMODEL_LINE);    
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (5000);               //최대 실행 수
    seg.setDistanceThreshold (ransac_distanceThreshold);   
        
    if(cloud_new->points.size() > 2){    
        seg.setInputCloud (cloud_new);
        seg.segment (*inliers, *coefficients);
        // cout << "Model coefficients: "  << coefficients->values[0] << " " // point x
        //                                 << coefficients->values[1] << " "  //point y
        //                                 << coefficients->values[2] << " " // point z
        //                                 << coefficients->values[3] << " " //vector x
        //                                 << coefficients->values[4] << " "  //vector y
        //                                 << coefficients->values[5] << endl; // vector z
        double slope = (*coefficients).values[4] / (*coefficients).values[3];  //    dy/dx   = - dx/ dy                         
        cout << "y1 = "  << slope << "x + "<<(-slope*coefficients->values[0] + coefficients->values[1]) << endl;
        coeff1.Slope = slope;
        coeff1.Yintercept = (-slope*coefficients->values[0] + coefficients->values[1]);
        pcl::copyPointCloud<pcl::PointXYZI>(*cloud_new, *inliers, *inlierPoints_first);
        extract.setInputCloud(cloud_new);
        extract.setIndices(inliers); 
        extract.setNegative(true);
        extract.filter(*cloud_new2);
        if(cloud_new2->points.size() <= 2){
            coeff2.Slope = coeff1.Slope;
            if(coeff1.Slope < -1.0f || coeff1.Slope > 1.0f){
                coeff2.Yintercept = (coeff1.Slope * 13.4) + coeff1.Yintercept; //모라이 11.4  Kcity 13.4
                coeff_arr.coeff_data.push_back(coeff1);
                coeff_arr.coeff_data.push_back(coeff2);
            }
            else{
                if(coeff1.Yintercept > 0.0f){
                    coeff2.Yintercept = coeff1.Yintercept - 13.4;//13.4
                }
                else{
                    coeff2.Yintercept = coeff1.Yintercept + 13.4;
                }
                coeff_arr.coeff_data.push_back(coeff1);
                coeff_arr.coeff_data.push_back(coeff2);
            }
        }
        else if(cloud_new2->points.size() > 2){
            seg.setInputCloud(cloud_new2);
            
            seg.segment(*inliers2,*coefficients2);
            double slope2 = (*coefficients2).values[4] / (*coefficients2).values[3];  //    dy/dx   = - dx/ dy                   
            cout << "y2 = "  << slope2 << "x + "<<(-slope2*coefficients2->values[0] + coefficients2->values[1]) << endl;
            coeff2.Slope = slope2;
            coeff2.Yintercept = (-slope2*coefficients2->values[0] + coefficients2->values[1]);
            pcl::copyPointCloud<pcl::PointXYZI>(*cloud_new2, *inliers2, *inlierPoints_second);
            // *inlierPoints = (*inlierPoints_first)+(*inlierPoints_second);
            extract.setInputCloud(cloud_new2);
            extract.setIndices(inliers2);
            extract.setNegative(true);
            extract.filter(*cloud_new3);
            if(abs(slope-slope2) < 0.1){
                *inlierPoints = (*inlierPoints_first)+(*inlierPoints_second);

                coeff_arr.coeff_data.push_back(coeff1);
                coeff_arr.coeff_data.push_back(coeff2);

                // PCXYZI Finalpt;
                // pcl::VoxelGrid<PXYZI> sor;
                PCXYZI::Ptr Left_ptr (new PCXYZI());
                PCXYZI::Ptr Right_ptr (new PCXYZI());
                if(coeff1.Yintercept < coeff2.Yintercept){ //inlier first is left 
                    *Right_ptr = *inlierPoints_first;
                    *Left_ptr = * inlierPoints_second;
                }
                else{
                    *Left_ptr = *inlierPoints_first;
                    *Right_ptr = * inlierPoints_second;
                }
                sor.setInputCloud(Left_ptr);
                sor.setLeafSize(0.1f, 0.1f, 0.1f); 
                PCXYZI::Ptr NewCloud_L (new PCXYZI());
                sor.filter(*NewCloud_L);
                sor.setInputCloud(Right_ptr);
                sor.setLeafSize(0.1f, 0.1f, 0.1f); 
                PCXYZI::Ptr NewCloud_R (new PCXYZI());
                sor.filter(*NewCloud_R);
                std::sort(NewCloud_R->points.begin(), NewCloud_R->points.end(), [](const PXYZI &a, const PXYZI &b) {
                    return a.x < b.x;
                });

                // NewCloud_R의 마지막 포인트를 xMax_R로 저장
                PXYZI xMin_R = NewCloud_R->points.front();
                PXYZI xMax_R = NewCloud_R->points.back();
                Finalpt.push_back(xMax_R);

                top1.x = xMax_R.x;
                top1.y = xMax_R.y;
                TOP.top_arr.push_back(top1);
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
                    top2.x = xMax_L.x;
                    top2.y = xMax_L.y;
                    TOP.top_arr.push_back(top2);
                    TOP.ptNum = Finalpt.size();
                }
            }
            else{
                if(cloud_new3->points.size() > 2){
                    seg.setInputCloud(cloud_new3);
                    seg.segment(*inliers3, *coefficients3);
                    pcl::copyPointCloud<pcl::PointXYZI>(*cloud_new3, *inliers3, *inlierPoints_third);
                    double slope3 = (*coefficients3).values[4] / (*coefficients3).values[3]; //    dy/dx   = - dx/ dy
                    cout << "y3 = " << slope3 << "x + " << (-slope3 * coefficients3->values[0] + coefficients3->values[1]) << endl;
                    coeff3.Slope = slope3;
                    coeff3.Yintercept = (-slope3 * coefficients3->values[0] + coefficients3->values[1]);
                    if(abs(slope-slope3) < 0.1f){
                        *inlierPoints = (*inlierPoints_first) + (*inlierPoints_third);

                        coeff_arr.coeff_data.push_back(coeff1);
                        coeff_arr.coeff_data.push_back(coeff3);
                    }
                    else if(abs(slope2-slope3) < 0.1f){
                        *inlierPoints = (*inlierPoints_second) + (*inlierPoints_third);

                        coeff_arr.coeff_data.push_back(coeff2);
                        coeff_arr.coeff_data.push_back(coeff3);
                    }   
                }
            }                
        }
    }
    
    for(int i= 0; i < cloud_new->points.size(); i++){
            cloud_new->points[i].z = cloud_new->points[i].intensity; 
        }
    sort(coeff_arr.coeff_data.begin(), coeff_arr.coeff_data.end(),
          [](const LiDAR::coeff& obj1, const LiDAR::coeff& obj2) {
              return obj1.Yintercept < obj2.Yintercept;   //first yintercept (-) : right, second yintercept (+) : left
          });
    pub_coeff.publish(coeff_arr);
    pub_top.publish(TOP); 
    //x m away from each side line
    //
    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(Finalpt, output2);
    output2.header.frame_id = "map"; // 또는 적절한 프레임 ID로 변경
    pub_top_PCL.publish(output2);
    //
    for(const auto& points : cloud_origin->points){
        if(coeff_arr.coeff_data.size() == 2 ){    
            if(points.y > (coeff_arr.coeff_data[0].Slope * points.x) + coeff_arr.coeff_data[0].Yintercept + 2 &&
               points.y < (coeff_arr.coeff_data[1].Slope * points.x) + coeff_arr.coeff_data[1].Yintercept -2){
                    finalpoints->points.push_back(points);
            }
        }    
    }
    sensor_msgs::PointCloud2 output;                        //to output ROIdata formed PC2
    pub_process(*finalpoints,output);
    pub_RS.publish(output);//터널 라인 제거한 inputcloud
    pub_process(*inlierPoints,output);
    pub_walls.publish(output);//터널라인
    coeff_arr.coeff_data.clear();
    TOP.top_arr.clear();
}

void Parsing_lidar_data(const char* buffer) {//VLP16 or VLP32

    for( int i = 0; i < 1200; i += 100 ){
        uint8_t byte1 = buffer[i + 2];// 2,3번째 데이터 
        uint8_t byte2 = buffer[i + 3];
        uint16_t byte_comb_a = (byte2 << 8) | byte1;
        double Azimuth = byte_comb_a * 0.01;

        int laser_id = 0;
        for( int j = 4; j < 100; j += 3){
            uint8_t byte_dist1 = buffer[i+j]; //distance
            uint8_t byte_dist2 = buffer[i+j+1];
            uint8_t byte_reflec = buffer[i+j+2]; // intensity
            uint16_t byte_comb_d = (byte_dist2 << 8) | byte_dist1;
            double Distance = byte_comb_d * 0.002;
            if(VLP16 == true){
                arr.push_back(cal_point_V16(Distance, Azimuth, byte_reflec, laser_id));
            }
            else{
                arr.push_back(cal_point(Distance, Azimuth, byte_reflec, laser_id));
            }
            laser_id ++;
        }
    }
    Parse_rawdata();
    arr.clear();
    arr2.clear();
}

void Parsing_Hesai(const char* buffer){  //12 for header, 28 for tail

    for(int i = 12; i < 1052; i += 130){  // 130  ->  data double
        uint8_t byte1 = buffer[i];//  첫번째 데이터
        uint8_t byte2 = buffer[i+1]; // 두번째 데이터
        uint16_t byte_comb_a = (byte2 << 8) | byte1;
        double Azimuth = byte_comb_a * 0.01;
        
        int laser_id = 0;
        for(int j = 2; j < 130; j += 4){
            //cout <<"j   ->   "<< j << endl;
            uint8_t byte_dist1 = buffer[i+j];
            uint8_t byte_dist2 = buffer[i+j+1];
            uint8_t byte_reflec = buffer[i+j+2]; //reflectivity = reflectivity * 0.01 
            uint16_t byte_comb_d = (byte_dist2 << 8) | byte_dist1;
            double Distance = byte_comb_d * 0.004;   //Distance unit is 4mm
            //if(Distance > 0.2){
            arr.push_back(cal_point_H(Distance, Azimuth, byte_reflec, laser_id));
            //}
            laser_id ++;
        }
    }
    Parse_rawdata();
    arr.clear();
    arr2.clear();
}
void Parsing_Ouster(const char* buffer){
    // cout <<"Parsing ouster" << endl;
    for(int i = 32; i < 6368; i += 396){  // each block 404bytes -> 16 block exist. + first 32 and last 32 bytes excepted
        // 4바이트의 encoder count를 아지무스로 변환 (360도 범위)
        uint16_t Measure_ID = ((uint8_t)buffer[i+9] << 8) | (uint8_t)buffer[i+8]; //360 / 1024 = 0.35degree
        // cout <<"Measure ID = "<< Measure_ID << endl;
        // cout << ((uint8_t)buffer[i+10] & 0b00000001) << endl;
        int laser_id = 0;

        for(int j = 12; j < 396; j += 12){ 
            uint32_t byte_range = (buffer[i + j] | (buffer[i + j + 1] << 8) | (buffer[i + j + 2] << 16) & 0b00000111); // 19비트만 사용
            double Distance = byte_range; //unit mm
            // cout << "Distance = " << Distance  << endl;
            uint8_t byte_reflec = buffer[i + j + 4];
            // cout << "Reflec = " <<byte_reflec << endl; 
            // arr.push_back(cal_point_Ouster(Distance, Azimuth, byte_reflec, laser_id));
            if(1)
            {
                arr.push_back(cal_point_Ouster(Distance, byte_reflec, laser_id, Measure_ID));
            }
            else
            {
                // cout << "Distance : 0" << endl;
            }
            //                             double    uint8_t      int       uint16_t 

            laser_id++;
        } 
    }
    Parse_rawdata();
    arr.clear();
    arr2.clear();
}


void Parse_rawdata(){//by YJ
    for(int i = 0; i < arr.size(); i++){
        azimuth_now = arr[i].azimuth;
        pcl::PointXYZI tmp;
        tmp.x = arr[i].x;
        tmp.y = arr[i].y;
        tmp.z = arr[i].z;
        tmp.intensity = arr[i].reflec;
        rawdata.push_back(tmp);//rawdata 넣기 
    } 

    if(azimuth_b4 < 180.0f && azimuth_now >= 180.0f) {
    // if(azimuth_b4 > 180.0f && azimuth_now <= 180.0f) {
    // if(k ==200){
        printf("\033[38;2;139;232;229mState\033[0m : \033[1;37m%d\033[0m\n", lidar_state);
        if(switch_UDP_communication) RT::end_cal("UDP_communication");
        else printf("\033[33mUDP_communication runtime\033[0m : \033[1;35mOFF\033[0m\n");
        RT::start();
        sensor_msgs::PointCloud2 output;
        // cout << "rawdata size = " << rawdata.size() << endl; 
        pub_process(rawdata, output);

        pub_rawdata.publish(output);
        rawdata.clear();
    }
    azimuth_b4 = azimuth_now;
}

void GE_OusterRaw(const sensor_msgs::PointCloud2ConstPtr& scan){ //ROS 통신 - 지면제거
    PCXYZI::Ptr origin(new PCXYZI());
    PCXYZI::Ptr cloud(new PCXYZI());
    PCXYZI::Ptr cloud_upper(new PCXYZI());
    pcl::fromROSMsg(*scan, *origin);
    for(int i =0;i<origin->points.size();i++){
        if(origin->points[i].z < GE_Z){ //GE_Z아래인 점들에 대해 랜삭 돌리기 위함
            (*cloud).push_back(origin->points[i]);
        }
        else{
            (*cloud_upper).push_back(origin->points[i]);
        }
    }
    PCXYZI::Ptr cloud_new (new PCXYZI()),inlierPoints (new PCXYZI());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); // point x,y,z, vector x,y,z
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    seg.setOptimizeCoefficients (true);       
    seg.setModelType (pcl::SACMODEL_PLANE);    
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);               //최대 실행 수
    seg.setDistanceThreshold (0.3);//평면으로 부터 0.3m떨어진 점까지 inlier로 간주
        
    if(cloud->points.size() > 2){    
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        
        pcl::copyPointCloud<pcl::PointXYZI>(*cloud, *inliers, *inlierPoints);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers); 
        extract.setNegative(true);
        extract.filter(*cloud_new);
    }
    ground = *inlierPoints;
    others = *cloud_new + *cloud_upper;
    
    sensor_msgs::PointCloud2 output;
    pub_process(others,output);
    pub_others.publish(output);
    pub_process(ground,output);
    pub_ground.publish(output);

    others.clear();
    ground.clear();
}

void Ground_Extraction(const sensor_msgs::PointCloud2ConstPtr& scan){//VLP32
    PCXYZI p;
    pcl::fromROSMsg(*scan, p);
    for( int i = 0; i < p.size(); i+=32){
        PCXYZI arr3;

        for( int j =i; j < i+32; j += 2){//0,2,4~~30, 1,3,5~31
            arr3.push_back(p[j]);
        }
        for(int j = i+1; j < i+32; j+= 2){
            arr3.push_back(p[j]);
        }
        //
        int object = 0;
        //
        for(int k =0 ; k < arr3.size(); k++){
            double d_xy_1 = cal_dist_21(arr3[k].x, arr3[k-1].x, arr3[k].y, arr3[k-1].y);
            double d_z_1 = abs(arr3[k].z - arr3[k-1].z);
            double slope1 = atan2(d_z_1, d_xy_1);
           // cout << "slope 1 = " << slope1 << endl;
            double d_xy_2 = cal_dist_21(arr3[k+1].x, arr3[k].x, arr3[k+1].y, arr3[k].y);
            double d_z_2 = abs(arr3[k+1].z - arr3[k].z);
            double slope2 = atan2(d_z_2, d_xy_2);
            //cout << "slope 2 = " << slope2 << endl;
            pcl::PointXYZI tmp;
            tmp.x = arr3[k].x;
            tmp.y = arr3[k].y;
            tmp.z = arr3[k].z;
            tmp.intensity = arr3[k].intensity;
            if (k == 0)
            {
                if (arr3[k].z <= GE_Z)
                {
                    ground.push_back(tmp);
                    object = 0;
                }
                else if (arr3[k].z > GE_Z)
                {
                    if(slope2 < GE_slope)
                    {
                        ground.push_back(tmp);
                        object = 0;
                    }
                    else
                    {
                        others.push_back(tmp);
                        object = 1;
                    }
                }
            }
            else if (k != 0)
            {
                if (arr3[k].z <= GE_Z)
                { // lidar z position 0.5
                    ground.push_back(tmp);
                    object = 0;
                }

                else if (arr3[k].z > GE_Z)
                {
                    if(abs(slope2 - slope1) <= GE_delta_slope && slope2 < GE_slope)
                    {
                        ground.push_back(tmp);
                    }
                    else
                    {
                        others.push_back(tmp);
                    }
                }
            }
        }
    }
    sensor_msgs::PointCloud2 output;
    pub_process(others,output);
    pub_others.publish(output);
    pub_process(ground,output);
    pub_ground.publish(output);

    others.clear();
    ground.clear();
}
void Ground_Extraction2(const sensor_msgs::PointCloud2ConstPtr& scan){//VLP32 _RANSAC
    PCXYZI::Ptr origin(new PCXYZI());
    PCXYZI::Ptr cloud(new PCXYZI());
    PCXYZI::Ptr cloud_upper(new PCXYZI());
    pcl::fromROSMsg(*scan, *origin);
    for(int i =0;i<origin->points.size();i++){
        if(origin->points[i].z < GE_Z){
            (*cloud).push_back(origin->points[i]);
        }
        else{
            (*cloud_upper).push_back(origin->points[i]);
        }
    }
    PCXYZI::Ptr cloud_new (new PCXYZI()),inlierPoints (new PCXYZI());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); // point x,y,z, vector x,y,z
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    seg.setOptimizeCoefficients (true);       
    seg.setModelType (pcl::SACMODEL_PLANE);    
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);               //최대 실행 수
    seg.setDistanceThreshold (0.3);//평면으로 부터 0.3m떨어진 점까지 inlier로 간주
        
    if(cloud->points.size() > 2){    
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        
        pcl::copyPointCloud<pcl::PointXYZI>(*cloud, *inliers, *inlierPoints);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers); 
        extract.setNegative(true);
        extract.filter(*cloud_new);
    }
    ground = *inlierPoints;
    others = *cloud_new + *cloud_upper;
    
    sensor_msgs::PointCloud2 output;
    pub_process(others,output);
    pub_others.publish(output);
    pub_process(ground,output);
    pub_ground.publish(output);

    others.clear();
    ground.clear();

}

void Ground_Extraction_H(const sensor_msgs::PointCloud2ConstPtr& scan){//Hesai & Ouster
    PCXYZI p;
    pcl::fromROSMsg(*scan,p);
    // cout << p.size() << endl;
    for (int i = 0; i < p.size(); i+=32){
        for(int k = i; k < i+32; k++){
            double d_xy_1, d_xy_2;
            double d_z_1, d_z_2;
            double slope1, slope2;
            if(k != 0){
                d_xy_1 = cal_dist_21(p[k].x, p[k-1].x, p[k].y, p[k-1].y);
                d_z_1 = abs(p[k].z - p[k-1].z);
                slope1 = atan2(d_z_1, d_xy_1);
            }
            if(k != 31){
                d_xy_2 = cal_dist_21(p[k + 1].x, p[k].x, p[k + 1].y, p[k].y);
                d_z_2 = abs(p[k + 1].z - p[k].z);
                slope2 = atan2(d_z_2, d_xy_2);
            }
            pcl::PointXYZI tmp;
            tmp.x = p[k].x;
            tmp.y = p[k].y;
            tmp.z = p[k].z;
            tmp.intensity = p[k].intensity;
            
            if(k == 0){
                if(p[k].z <= -0.7 || slope2 < 0.5){
                    ground.push_back(tmp);
                }
                else {
                    others.push_back(tmp);
                }
            }
            else if(k != 0 && k != 31){
                if(p[k].z <= -0.7){
                    ground.push_back(tmp);
                }
                else if(p[k].z > -0.7){
                    if(abs(slope2 - slope1) <= 0.3 && slope2 < 0.5){
                        ground.push_back(tmp);
                    }
                    else{
                        others.push_back(tmp);
                    }
                }
            }
            else{
                if(slope1 > 0.5 && d_xy_1 < 0.5){
                    others.push_back(tmp);
                }
                else{
                    ground.push_back(tmp);
                }
            }

        }
    }
    sensor_msgs::PointCloud2 output;
    pub_process(others,output);
    pub_others.publish(output);
    pub_process(ground,output);
    pub_ground.publish(output);

    others.clear();
    ground.clear();
}

void Ground_Extraction_V16(const sensor_msgs::PointCloud2ConstPtr& scan) {//for 16 channel
    PCXYZI p;
    pcl::fromROSMsg(*scan,p);
    for( int i = 0; i < p.size(); i+=16){
        PCXYZI arr3;

        for( int j =i; j < i+16; j += 2){//0,2,4~~30, 1,3,5~31
            arr3.push_back(p[j]);
        }
        for(int j = i+1; j < i+16; j+= 2){
            arr3.push_back(p[j]);
        }
        //
        int object = 0;
        //
        for(int k =0 ; k < arr3.size(); k++){
            double d_xy_1 = cal_dist_21(arr3[k].x, arr3[k-1].x, arr3[k].y, arr3[k-1].y);
            double d_z_1 = abs(arr3[k].z - arr3[k-1].z);
            double slope1 = atan2(d_z_1, d_xy_1);
           // cout << "slope 1 = " << slope1 << endl;
            double d_xy_2 = cal_dist_21(arr3[k+1].x, arr3[k].x, arr3[k+1].y, arr3[k].y);
            double d_z_2 = abs(arr3[k+1].z - arr3[k].z);
            double slope2 = atan2(d_z_2, d_xy_2);
            //cout << "slope 2 = " << slope2 << endl;
            pcl::PointXYZI tmp;
            tmp.x = arr3[k].x;
            tmp.y = arr3[k].y;
            tmp.z = arr3[k].z;
            tmp.intensity = arr3[k].intensity;
            if (k == 0)
            {
                if (arr3[k].z <= GE_Z)
                {
                    ground.push_back(tmp);
                    object = 0;
                }
                else if (arr3[k].z > GE_Z)
                {
                    if(slope2 < GE_slope)
                    {
                        ground.push_back(tmp);
                        object = 0;
                    }
                    else
                    {
                        others.push_back(tmp);
                        object = 1;
                    }
                }
            }
            else if (k != 0)
            {
                if (arr3[k].z <= GE_Z)
                { // lidar z position 0.5
                    ground.push_back(tmp);
                    object = 0;
                }

                else if (arr3[k].z > GE_Z)
                {
                    if(abs(slope2 - slope1) <= GE_delta_slope && slope2 < GE_slope)
                    {
                        ground.push_back(tmp);
                    }
                    else
                    {
                        others.push_back(tmp);
                    }
                }
            }
        }
    }
    sensor_msgs::PointCloud2 output;
    pub_process(others,output);
    pub_others.publish(output);
    pub_process(ground,output);
    pub_ground.publish(output);

    others.clear();
    ground.clear();
}
