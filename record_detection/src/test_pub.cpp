#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char  *argv[])
{   

    setlocale(LC_ALL,"");

    ros::init(argc,argv,"talker");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);
    std_msgs::String msg;
    std::string msg_front = "Hello";
    int count = 0;

    ros::Rate r(1);

    while (ros::ok())
    {
        std::stringstream ss;
        ss << msg_front << count;
        msg.data = ss.str();
        pub.publish(msg);
        ROS_INFO("Message being sent:%s",msg.data.c_str());
        r.sleep();
        count++;
        ros::spinOnce();
    }


    return 0;
}
