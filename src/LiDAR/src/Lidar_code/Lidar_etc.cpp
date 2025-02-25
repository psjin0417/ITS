#include <LiDAR/Lidar_declare.h>

using namespace std;

Fps::Fps(): m_count(0), prev_clock(1650000000){
    // cout << "\033[1;42mprocessing node pid = " << getpid() << "\033[0m" << endl;
};

void Fps::update(){
    double cur_clock = ros::Time::now().toSec();
    interval = cur_clock - prev_clock;
    prev_clock = cur_clock;
    m_fps = 1 / interval;
    m_count++;
    // if (interval < 0.1)  {
        // printf("\033[38;2;139;232;229mInterval\033[0m : \033[38;2;22;225;0m%.1f\033[0m ms", interval * 1000);
        // printf("\033[38;2;139;232;229m FPS\033[0m : \033[38s;2;22;225;0m%.1f\033[0m frame/sec", m_fps);
        cout << setprecision(2) << "\033[33mClustering runtime\033[0m : \033[1;35m" << m_fps << "\033[0m Hz" << endl;
        // printf("\tLoop %zu\n", m_count);
        // printf("\033[38;2;22;225;0m[SAFE] normal speed\033[0m\n"); 
    // }
    // else {   
        // printf("\033[38;2;139;232;229mInterval\033[0m : \033[1;93m%.1f\033[0m ms", interval * 1000);
        // printf("\033[38;2;139;232;229m FPS\033[0m : \033[1;93m%.1f\033[0m frame/sec", m_fps);
        // printf("\tLoop %zu\n", m_count);
        // printf("\033[1;93m[WARN] low speed warning\033[0m\n"); 
    // }
    // printf("\033[2m────────────────────────────");
    // printf("────────────────────────────\033[0m\n");
}

RT::RT(){}

void RT::start(){
    prev_clock = ros::Time::now().toSec();
}

void RT::end_cal(const char* nodeName){
    cur_clock = ros::Time::now().toSec();
    interval = cur_clock - prev_clock;
    double m_fps = 1 / interval;
    // cout << setprecision(2) << "\033[33m" << nodeName << " runtime\033[0m : \033[1;35m" << interval * 1000 << "\033[0m ms" << endl;
    cout << setprecision(2) << "\033[33m" << nodeName << " runtime\033[0m : \033[1;35m" << m_fps << "\033[0m Hz" << endl;
    // printf("\033[38;2;139;232;229m%s runtime\033[0m : \033[1;37m%.1f\033[0m ms\n", nodeName, interval * 1000);
}