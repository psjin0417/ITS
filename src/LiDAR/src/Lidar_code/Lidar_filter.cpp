#include <LiDAR/Lidar_declare.h>

using namespace std;

void Filter::Cone_filter(vector<objectInfo>& objs, bool flag){
    if(!flag) return;
    
    for (vector<objectInfo>::iterator it = objs.begin(); it != objs.end();) {   
        if ( it->zMax > 0.2 
            || (abs(it->zMax-it->zMin) > 0.75)
            // || (abs(it->zMax-it->zMin) < 0.1) 
            || (abs(it->xMax-it->xMin) > 0.45 || abs(it->xMax-it->xMin) < 0.00) || (abs(it->yMax-it->yMin) > 0.45 || abs(it->yMax-it->yMin) < 0.00)){
            it = objs.erase(it); //edit object vector
        }
        else it++;
    }
}

void Filter::Surround_filter(vector<objectInfo>& objs, bool flag){
    if(!flag) return;

    for (vector<objectInfo>::iterator it = objs.begin(); it != objs.end();){
        if (it->zMax < Surround_Z_ROI){
        // if (abs(it->yMax-it->yMin) > 0.7 || abs(it->xMax-it->xMin) > 0.6 || abs(it->yMax-it->yMin) < 0.3 ){
            it = objs.erase(it); //edit object vector
        }
        else it++;
    }
}
void Filter::Delievery_filter(vector<objectInfo>& objs, bool flag){
    if(!flag) return;

    for (vector<objectInfo>::iterator it = objs.begin(); it != objs.end();){

        if (abs(it->yMax-it->yMin) > 0.8 || abs(it->xMax-it->xMin) > 0.6 || abs(it->yMax-it->yMin) < 0.2 
        // || it->zMax < 0.3 
        ){
            it = objs.erase(it); //edit object vector
        }
        else it++;
    }
}

void Filter::Drum_filter(vector<objectInfo>& objs, bool flag){
    if(!flag) return;

    for (vector<objectInfo>::iterator it = objs.begin(); it != objs.end();){

        if (it->zMax > 0.2 || it->zMax < -0.5 ){
            it = objs.erase(it); //edit object vector
        }
        else it++;
    }

}


void Filter::jiwon_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag) {
    if(!flag) return;
    vector<pair<PXYZI,string>>::iterator it = sorted_OBJ.begin();
    for (int i = 0; i < sorted_OBJ.size(); i++) {
        it = sorted_OBJ.begin();
        for (int k = 0; k < i + 1; k++) {
            it++;
        }
        for (int j = i + 1; j < sorted_OBJ.size(); j++) {
            if (check_in(sorted_OBJ[i].first, sorted_OBJ[j].first)) {
                it = sorted_OBJ.erase(it);
                it--;
                j--;
            }
            it++;
        }
    }
}

void Filter::jiwon_filter(std::vector<objectInfo>& objs, pcl::PointCloud<PXYZI>::Ptr inputCloud, bool flag) {//using code
    if (!flag) return;

    const float DISTANCE_THRESHOLD = 3.0; // x * x

    bool merged;
    do {
        merged = false; // 초기 상태를 false로 설정

        for (std::vector<objectInfo>::iterator iit = objs.begin(), jit; iit != objs.end(); ++iit) {
            for (jit = iit + 1; jit != objs.end(); ) {
                bool shouldMerge = false;

                // 먼저 두 객체의 중심점 간의 거리를 계산
                float dx = (*iit).x - (*jit).x;
                float dy = (*iit).y - (*jit).y;
                float distance = sqrt(dx * dx + dy * dy);

                if (distance <= DISTANCE_THRESHOLD) {
                    const auto& i_indices = (*iit).objPoints->indices;
                    const auto& j_indices = (*jit).objPoints->indices;
                    const auto& points = inputCloud->points; // inputCloud->points에 대한 접근 캐싱

                    for (int i_idx : i_indices) {
                        const PXYZI& PIT = points[i_idx]; // 참조를 사용하여 포인트에 접근

                        for (int j_idx : j_indices) {
                            const PXYZI& PJT = points[j_idx]; // 참조를 사용하여 포인트에 접근

                            float pdx = PIT.x - PJT.x;
                            float pdy = PIT.y - PJT.y;
                            if ((pdx * pdx + pdy * pdy) <= (REMOVE_FACTOR * REMOVE_FACTOR)) {
                                shouldMerge = true;
                                break;
                            }
                        }

                        if (shouldMerge) break;
                    }
                }
                if (shouldMerge) {
                    // 물체 정보를 업데이트
                    (*iit).xMin = std::min((*iit).xMin, (*jit).xMin);
                    (*iit).yMin = std::min((*iit).yMin, (*jit).yMin);
                    (*iit).zMin = std::min((*iit).zMin, (*jit).zMin);
                    (*iit).xMax = std::max((*iit).xMax, (*jit).xMax);
                    (*iit).yMax = std::max((*iit).yMax, (*jit).yMax);
                    (*iit).zMax = std::max((*iit).zMax, (*jit).zMax);
                    (*iit).x = ((*iit).xMin + (*iit).xMax) / 2;
                    (*iit).y = ((*iit).yMin + (*iit).yMax) / 2;
                    (*iit).z = ((*iit).zMin + (*iit).zMax) / 2;

                    // 포인트 인덱스 병합
                    (*iit).objPoints->indices.insert((*iit).objPoints->indices.end(), 
                                                     (*jit).objPoints->indices.begin(), 
                                                     (*jit).objPoints->indices.end());

                    jit = objs.erase(jit); // 벡터에서 삭제 후 다음 클러스터로
                    merged = true; // 병합이 일어났음을 표시
                } else {
                    ++jit;
                }
            }
        }
    } while (merged); // 더 이상 병합이 일어나지 않을 때까지 반복
}



void Filter::jiwon_filter(vector<objectInfo>& objs, bool flag) {
    if(!flag) return;

    for (vector<objectInfo>::iterator iit = objs.begin(), jit; iit != objs.end(); iit++){

        jit = iit + 1;

        for(jit; jit != objs.end();){

            if(((abs(iit->x - jit->x) <= REMOVE_FACTOR) && (abs(iit->y - jit->y) <= REMOVE_FACTOR))){
            // if( ( abs(iit->xMin - jit->xMin) <= REMOVE_FACTOR || abs(iit->xMax - jit->xMax) <= REMOVE_FACTOR || abs(iit->xMin - jit->xMax) <= REMOVE_FACTOR || abs(iit->xMax - jit->xMin) <= REMOVE_FACTOR ) &&
            //     ( abs(iit->yMin - jit->yMin) <= REMOVE_FACTOR || abs(iit->yMax - jit->yMax) <= REMOVE_FACTOR || abs(iit->yMin - jit->yMax) <= REMOVE_FACTOR || abs(iit->yMax - jit->yMin) <= REMOVE_FACTOR ) ){    
                //edit obj info
                iit->xMin = (iit->xMin < jit->xMin) ? iit->xMin : jit->xMin;
                iit->yMin = (iit->yMin < jit->yMin) ? iit->yMin : jit->yMin;
                iit->zMin = (iit->zMin < jit->zMin) ? iit->zMin : jit->zMin;
                iit->xMax = (iit->xMax > jit->xMax) ? iit->xMax : jit->xMax;
                iit->yMax = (iit->yMax > jit->yMax) ? iit->yMax : jit->yMax;
                iit->zMax = (iit->zMax > jit->zMax) ? iit->zMax : jit->zMax;
                iit->x = (iit->xMin + iit->xMax) / 2;
                iit->y = (iit->yMin + iit->yMax) / 2;
                iit->z = (iit->zMin + iit->zMax) / 2;

                //merge point indices
                iit->objPoints->indices.insert(iit->objPoints->indices.end(), jit->objPoints->indices.begin(), jit->objPoints->indices.end());

                jit = objs.erase(jit); //edit object vector
            }
            else jit++;
        }

    }
}
bool compareY(const PXYZI& a, const PXYZI& b){
    return a.y < b.y;
}
void Filter::find_Rectangle(std::vector<objectInfo>& objs, pcl::PointCloud<PXYZI>::Ptr inputCloud, bool flag) {
    if (!flag) return;
    const double tan1deg = std::tan(M_PI / 180.0);  // 1도의 탄젠트 값 (rad)

    for (auto& obj : objs) {

        if (obj.zMax <= 0.3) continue;

        // float z_interval = obj.x * tan1deg;
        float z_interval = 0.1;
        const auto& i_indices = obj.objPoints->indices;
        const auto& points = inputCloud->points;

        if (i_indices.empty() || points.empty()) continue;

        pcl::PointIndices::Ptr filtered_indices(new pcl::PointIndices);

        for (float z_start = obj.zMin; z_start < obj.zMax; z_start += z_interval) {

            float z_end = z_start + z_interval;
            std::vector<int> valid_indices;

            for (int i_idx : i_indices) {

                if (i_idx < 0 || i_idx >= points.size()) continue; // 인덱스 범위 검사

                const PXYZI& PIT = points[i_idx];
                if (PIT.z >= z_start && PIT.z < z_end) {
                    valid_indices.push_back(i_idx);
                }
            }

            if (valid_indices.empty()) continue;

            std::sort(valid_indices.begin(), valid_indices.end(), [&points](int a, int b) {
                return points[a].y < points[b].y;
            });

            float y_min = points[valid_indices.front()].y;
            float y_max = points[valid_indices.back()].y;

            if (std::abs(y_max - y_min) >= 0.15) {
                filtered_indices->indices.insert(filtered_indices->indices.end(), valid_indices.begin(), valid_indices.end());
            }
        }

        // 필터링된 인덱스를 objPoints에 갱신
        obj.objPoints->indices = std::move(filtered_indices->indices);
        // minmax 업데이트를 위한 초기화
        obj.xMin = obj.yMin = obj.zMin = std::numeric_limits<float>::max();
        obj.xMax = obj.yMax = obj.zMax = std::numeric_limits<float>::lowest();

        // 필터링된 포인트들을 바탕으로 min/max 업데이트
        for (int i_idx : obj.objPoints->indices) {
            const PXYZI& PIT = points[i_idx];
            if (PIT.x < obj.xMin) obj.xMin = PIT.x;
            if (PIT.y < obj.yMin) obj.yMin = PIT.y;
            if (PIT.z < obj.zMin) obj.zMin = PIT.z;

            if (PIT.x > obj.xMax) obj.xMax = PIT.x;
            if (PIT.y > obj.yMax) obj.yMax = PIT.y;
            if (PIT.z > obj.zMax) obj.zMax = PIT.z;
        }
    }
}



void Filter::generate_return_PointCloud(PCXYZI::Ptr inputCloud, PCXYZI& returnCloud, vector<objectInfo>& objs){
    short intensity = 0; //color PointCloud to seperate each cluster
    returnCloud.clear();
    for (const objectInfo& obj : objs){
        for(const int& idx : obj.objPoints->indices){
            PXYZI tmpInput = inputCloud->points[idx];
            tmpInput.intensity = (intensity) % 10;
            returnCloud.push_back(tmpInput);
        }
        intensity++;
    }
}