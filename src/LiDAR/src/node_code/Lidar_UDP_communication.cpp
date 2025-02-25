#include <LiDAR/Lidar_declare.h>

ros::Publisher pub_ref;

void udp_process(const LiDAR::totalInfoConstPtr& signal) {
    lidar_state = signal->State;
    switch_UDP_communication = (signal->State == 1 || signal->State == 2 || signal->State == 3 || signal->State == 4 || signal->State == 8 || signal->State == 9 || signal->State == -1) ? true : false;

    // if(signal->State == 4) {
    //     GE_Z = GE_Z_4;
    // }
    // else {
    //     GE_Z = GE_Z_re;
    // }
}

void* dataReceptionThread(void* arg) {
    int socket_fd = *(int*)arg;
    struct sockaddr_in client_address;

    // BUFFER_SIZE에 맞는 동적 메모리 할당
    char* buffer = new char[BUFFER_SIZE];

    while (ros::ok() && !shouldExit) {
        socklen_t addr_len = sizeof(client_address);
        ssize_t num_bytes = recvfrom(socket_fd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&client_address, &addr_len);
        // cout<<"Received byte = " << num_bytes<< endl;
        if (num_bytes == -1) {
            cerr << "recvfrom failed: " << strerror(errno) << endl;
            continue;
        }
        
        if (num_bytes == BUFFER_SIZE) {
            std::vector<char> data(buffer, buffer + num_bytes);
            std::lock_guard<std::mutex> lock(queueMutex);
            dataQueue.push(data);
            //대기중인 파싱쓰레드에 새로운 데이터가 큐에 들어왔음 알림
            queueCondVar.notify_one();
        }
    }
    delete[] buffer;
    return nullptr;
}


// 데이터 파싱 스레드 함수
void* dataParsingThread(void* arg) {
    // cout <<"--" << endl;
    while (ros::ok() && !shouldExit) {
        std::vector<char> data;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            //notify_one이 호출될때까지 대기
            queueCondVar.wait(lock, [] { return !dataQueue.empty() || shouldExit; });
            if (shouldExit) break;
            //dataQue의 첫번째 데이터를 꺼내고 dataQue에서 뺀 데이터는 삭제
            data = dataQueue.front();
            dataQueue.pop();
        }
        if(BUFFER_SIZE == 1080){
            Parsing_Hesai(data.data());
        }
        else if(BUFFER_SIZE == 6400){
            Parsing_Ouster(data.data());
        }
        else{
            Parsing_lidar_data(data.data());
        }
    }
    return nullptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "UDP_communication");
    ros::NodeHandle nh;

    nh.getParam("/UDP_node/UDP_IP", UDP_IP);
    nh.getParam("/UDP_node/Port_Num", Port_Num);
    nh.getParam("/UDP_node/BUFFER_SIZE", BUFFER_SIZE);
    nh.getParam("/UDP_node/VLP16", VLP16);
    int socket_fd;
    struct sockaddr_in server_address;
    const char* IP = UDP_IP.c_str();

    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd == -1) {
        cerr << "Failed to create socket." << endl;
        return 1;
    }
    int opt = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        cerr << "Failed to set socket options." << endl;
        return 1;
    }

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = inet_addr(IP);//pc address
    server_address.sin_port = htons(Port_Num);


    if (bind(socket_fd, (struct sockaddr*)&server_address, sizeof(server_address)) == -1) {
        cerr << "Failed to bind socket." << endl;
        close(socket_fd);
        return 1;
    }

    //sub/pub이 쓰레드에서 사용되기 전에 반드시 초기화가 완료된 상태여야함
    ros::Subscriber sub = nh.subscribe<LiDAR::totalInfo>("/Totalcom", 1, udp_process);
    pub_rawdata = nh.advertise<sensor_msgs::PointCloud2>("/0_0_UDP_rawdata", 1);

    pthread_t receptionThread, parsingThread;
    // cout << 1 << endl;
    pthread_create(&receptionThread, nullptr, dataReceptionThread, &socket_fd);
    // cout << 3 << endl;
    pthread_create(&parsingThread, nullptr, dataParsingThread, nullptr);
    // ros::Subscriber sub = nh.subscribe<LiDAR::totalInfo>("/Totalcom", 1, udp_process);
    // pub_rawdata = nh.advertise<sensor_msgs::PointCloud2>("/0_0_UDP_rawdata", 1);

    ros::spin();    

    shouldExit = true;
    //대기 중인 모든 쓰레드를 깨움(여기서는 파싱 쓰레드만 해당)
    //shouldExit이 true이므로 파싱쓰레드는 종료
    queueCondVar.notify_all();
    //각 쓰레드의 종료를 기다림
    pthread_join(receptionThread, nullptr);
    //recpetion thread가 종료되면 다음 코드로 넘어감
    pthread_join(parsingThread, nullptr);
    //parsing thread가 종료되면 다음코드로 넘어감

    close(socket_fd);

    return 0;
}