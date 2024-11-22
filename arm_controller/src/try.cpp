#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
   ros::init(argc, argv, "joint_state_publisher");
   ros::NodeHandle nh;
   ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
   ros::Rate loop_rate(10);



   while(ros::ok()){
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(joint_state.name.size());
        joint_state.name = {"joint1", "joint2", "joint3", "joint4"};


        joint_state.position = {0,0 ,0 ,0 }; //in radians | teta1, teta2, teta3
        joint_state_pub.publish(joint_state);
        printf("joint state published\n");


        ros::spinOnce();
        loop_rate.sleep();
   }
   return 0;
}
