#include <LiDAR/Lidar_declare.h>

ros::Publisher pub_signal_test;

int st = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sg_test");
    ros::NodeHandle nh;
    ros::Time tm = ros::Time::now();

    pub_signal_test = nh.advertise<LiDAR::totalInfo>("/Totalcom", 1);
    ros::Rate rate(30);

    while (ros::ok())
    {
        ros::Time curtm = ros::Time::now();
        ros::Duration dt = curtm - tm;
        double dt_sec = dt.toSec();
        // if(dt_sec > 1) {
        //     st++;
        //     if(st > 9) st = -1;
        //     else if(st == -1) st = 0;
        //     tm = ros::Time::now();
        // 
        st = 4;
        LiDAR::totalInfo test_msg;
        test_msg.State = st;
        pub_signal_test.publish(test_msg);
        rate.sleep();
    }
    ros::spin();
}