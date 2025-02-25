#include <LiDAR/Lidar_declare.h>

using namespace std;

string send_msg_minmax(float xMin,float xMax, float yMin, float yMax){
    string tmp_xMax = (xMax < 0) ? to_string(((int)(xMax * -100) / 2) * 2 + 1) : to_string(((int)(xMax * 100) / 2) * 2);
    string tmp_xMin = (xMin < 0) ? to_string(((int)(xMin * -100) / 2) * 2 + 1) : to_string(((int)(xMin * 100) / 2) * 2);
    string tmp_yMax = (yMax < 0) ? to_string(((int)(yMax * -100) / 2) * 2 + 1) : to_string(((int)(yMax * 100) / 2) * 2);
    string tmp_yMin = (yMin < 0) ? to_string(((int)(yMin * -100) / 2) * 2 + 1) : to_string(((int)(yMin * 100) / 2) * 2);
    string tmp, st;

    for(int i = 4; i > tmp_xMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = tmp + tmp_xMin;
    tmp.clear();
    for(int i = 4; i > tmp_xMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_xMax;
    tmp.clear();
    for(int i = 4; i > tmp_yMin.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMin;
    tmp.clear();
    for(int i = 4; i > tmp_yMax.size(); i--){
        tmp = "0" + tmp;
    }
    st = st + tmp + tmp_yMax;
    return st;
}

bool comparedistance(const LiDAR::object_msg& a, const LiDAR::object_msg& b) {
    float dist1 = cal_dist(a.x, a.y);
    float dist2 = cal_dist(b.x, b.y);
    return dist1 < dist2;
}
bool comparedistance2(const LiDAR::objInfo& a, const LiDAR::objInfo& b) {
    float dist1 = cal_dist(a.posX, a.posY);
    float dist2 = cal_dist(b.posX, b.posY);
    return dist1 < dist2;
}
void object_msg_process(const vector<struct objectInfo>& objs, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){//for camera, pointInfo 추가
    LiDAR::object_msg_arr msg;
    LiDAR::object_msg msgCpnt;
    vector<LiDAR::object_msg> msgConvertVector;//for camera
    msg.objc = objs.size();
    for (objectInfo obj : objs){
        msgCpnt.x = obj.x;
        msgCpnt.y = obj.y;
        msgCpnt.z = obj.z;
        msgCpnt.xMin = obj.xMin;
        msgCpnt.yMin = obj.yMin;
        msgCpnt.zMin = obj.zMin;
        msgCpnt.xMax = obj.xMax;
        msgCpnt.yMax = obj.yMax;
        msgCpnt.zMax = obj.zMax;
        msgCpnt.classes = obj.classes;
        msgCpnt.idx = obj.idx;
        // for camera
        
        LiDAR::raw4cam p;
        // objPoints는 PointIndices로, 이를 통해 실제 포인트에 접근
        for (int i = 0; i < obj.objPoints->indices.size(); i++) {
            
            // 인덱스를 통해 실제 포인트를 가져옵니다
            pcl::PointXYZI point = cloud->points[obj.objPoints->indices[i]]; // cloud는 실제 포인트 클라우드
            
            // 조건에 맞는 포인트 처리
            if (point.y == obj.yMin) {
     
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                msgCpnt.atYmin = p;
            }
            else if (point.y == obj.yMax) {

                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                msgCpnt.atYmax = p;
            }
            else if (point.z == obj.zMin) {

                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                msgCpnt.atZmin = p;
            }
            else if (point.z == obj.zMax) {
   
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                msgCpnt.atZmax = p;
            }
        }
        //
        msgConvertVector.push_back(msgCpnt);
    }
    sort(msgConvertVector.begin(), msgConvertVector.end(),comparedistance);//dist arrange YJ
    for(int i = 0; i < msgConvertVector.size();i++){ //idx reload
         msgConvertVector[i].idx = i;
    }
    msg.object_msg_arr = msgConvertVector;
    pub_obj.publish(msg);
    // msg.object_msg_arr = msgConvertVector; //SM sort
    // sort(msg.object_msg_arr.begin(), msg.object_msg_arr.end(),
    //       [](const LiDAR::object_msg& obj1, const LiDAR::object_msg& obj2) {
    //         float dist_obj1 = cal_dist(obj1.x, obj1.y);
    //         float dist_obj2 = cal_dist(obj2.x, obj2.y);
    //           return dist_obj1 < dist_obj2;
    //       });
    // pub_obj.publish(msg);

    LiDAR::objsInfo msg3;
    LiDAR::objInfo msgCpnt3;
    LiDAR::coeff coeff_tmp;

    vector<LiDAR::objInfo> msgConvertVector3;//for 판단제어
    msg3.objNum = objs.size();
    int i = 1;
    for (objectInfo obj : objs){
        msgCpnt3.classes = i;
        msgCpnt3.posX = obj.x;
        msgCpnt3.posY = obj.y;
        msgCpnt3.posZ = obj.z;
        msgCpnt3.size = (obj.xMax-obj.xMin)*(obj.yMax-obj.yMin);// rectangular
        msgCpnt3.distance = cal_dist(obj.x, obj.y);
        msgCpnt3.x_max = obj.xMax;
        msgCpnt3.x_min = obj.xMin;
        msgCpnt3.y_max = obj.yMax;
        msgCpnt3.y_min = obj.yMin;
        msgCpnt3.z_max = obj.zMax;
        msgConvertVector3.push_back(msgCpnt3);
        i+=1;
    }
    for(int i = 0; i < ransac_coeff.size(); i++) {
        msg3.coeff.push_back(ransac_coeff[i]);
    }
    for(int i= 0; i < lane_coeff.size(); i++){
        msg3.lane.push_back(lane_coeff[i]);
    }
    for(int i= 0; i < top.size(); i++){
        msg3.tunneloutpoint.push_back(top[i]);
    }
    msg3.data = msgConvertVector3;
    sort(msg3.data.begin(),msg3.data.end(),comparedistance2);
    for(int i = 0; i < msg3.data.size(); i++){
        msg3.data[i].classes=i;
    }

    // Ensure the data size does not exceed 50
    if (msg3.data.size() > 50) {
        msg3.data.resize(50);  // Keep only the first 50 elements after class re-assigning
    }

    pub_LC.publish(msg3);
}