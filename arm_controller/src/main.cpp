#include "arm_controller/inverse_kinematic.hpp"
#include <custom_msg/degrees.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

class ArmController {
private:
    ros::NodeHandle nh_;
    ros::Publisher arm_pub_;
    ros::Subscriber sensor_sub_;
    custom_msg::degrees msg_degree_;
    
    float prev_distance_ = 0;
    float curr_distance_ = 0;
    int32_t current_rotation_ = -90;
    bool new_sensor_data_ = false;

    const double ROTATION_INCREMENT = 0.8;
    const double DISTANCE_THRESHOLD = 10.0;
    const double MINIMUM_DISTANCE = 25.0;
    const int PICKUP_POSITIONS = 3;

public:
    ArmController() : nh_() {
        arm_pub_ = nh_.advertise<custom_msg::degrees>("degree", 10);
        sensor_sub_ = nh_.subscribe("sensor_data", 10, &ArmController::sensorCallback, this);
    }

    void sensorCallback(const std_msgs::Float32::ConstPtr &msg) {
        ROS_INFO("Sensor callback triggered with data: %f", msg->data);
        prev_distance_ = curr_distance_;
        curr_distance_ = msg->data;
        new_sensor_data_ = true;
    }

    void moveIt_D1(float start, float end){
        for(float i = start; i<=end; i+=0.1){
            msg_degree_.teta1= i;
            publishAndLog();
            ros::Duration(0.05).sleep();
        }
    }

    void moveIt_D2(float start, float end){
        for(float i = start; i<=end; i+=0.1){
            msg_degree_.teta2= i;
            publishAndLog();
            ros::Duration(0.05).sleep();
        }
    }

    void moveIt_D3(float start, float end){
        for(float i = start; i<=end; i+=0.1){
            msg_degree_.teta3= i;
            publishAndLog();
            ros::Duration(0.05).sleep();
        }
    }

    void moveIt_D4(float start, float end){
        for(float i = start; i<=end; i+=0.2){
            msg_degree_.teta4= i;
            publishAndLog();
            ros::Duration(0.01).sleep();
        }
    }

    bool moveToScanPosition() {

        msg_degree_.teta1 = 90;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta0 = current_rotation_;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta4 = 50;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta3 = -66.4288;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta1 = 24.1963;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta2 = -90.0000;
        publishAndLog();
        ros::Duration(1.0).sleep();
   
        return true;
    }

    bool scanForObject() {
        ros::Rate rate(10); 
        new_sensor_data_ = false;

        while (ros::ok()) {
            ros::spinOnce();
            
            ROS_INFO("current location: %f", curr_distance_);

            if (std::abs(prev_distance_ - curr_distance_) >= DISTANCE_THRESHOLD || curr_distance_ > MINIMUM_DISTANCE) {
                
                ROS_INFO("Perbedaan %f", prev_distance_ - curr_distance_);

                if (msg_degree_.teta0 + ROTATION_INCREMENT >= 90) return false;  

                msg_degree_.teta0 += ROTATION_INCREMENT;
                publishAndLog();
                ros::Duration(0.06).sleep();
            } 
            else if (new_sensor_data_){
                
                current_rotation_ = msg_degree_.teta0 + 1;
                publishAndLog();
                ros::Duration(0.8).sleep();
                return true;  
            }

            rate.sleep();
        }
        return false;
    }

    bool pickupObject(int count) {

        float temp1 = inverse_teta1(curr_distance_, 6);
        float temp2 = inverse_teta2(curr_distance_, temp1, 6);
        float temp3 = inverse_teta3(temp1, temp2);
        
        msg_degree_.teta2 = temp2 * deg * -1;
        publishAndLog();
        ros::Duration(1.5).sleep();
        // moveIt_D2(msg_degree_.teta2, temp2 * deg * -1);

        msg_degree_.teta1 = temp1 * deg;
        publishAndLog();
        ros::Duration(1.5).sleep();
        // moveIt_D1(msg_degree_.teta1, temp1 * deg);

        msg_degree_.teta3 = temp3 * deg * -1;
        publishAndLog();
        ros::Duration(1.0).sleep();
        // moveIt_D3(msg_degree_.teta3, temp3 * deg * -1);

        msg_degree_.teta4 = 0;
        publishAndLog();
        ros::Duration(2.0).sleep();
        // moveIt_D4(msg_degree_.teta4, 0);

        return true;
    }

    bool upObject(){

        msg_degree_.teta1 = 70;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta2 = -20;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta3 = 50;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta0 = 90;
        publishAndLog();
        ros::Duration(1.0).sleep();    

        return true;    
    }

    bool placeObject(int count) {
        // Move to placement position
        float temp1 = inverse_teta1(7, 4.5*(count+1));
        float temp2 = inverse_teta2(7, temp1, 4.5*(count+1));
        float temp3 = inverse_teta3(temp1, temp2);

        msg_degree_.teta3 = temp3 * deg * -1;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta2 = temp2 * deg * -1;
        publishAndLog();
        ros::Duration(2.0).sleep();

        msg_degree_.teta1 = temp1 * deg;
        publishAndLog();
        ros::Duration(1.0).sleep();

        msg_degree_.teta4 = 50;
        publishAndLog();
        ros::Duration(1.0).sleep();

        return true;
    }

    void publishAndLog() {
        arm_pub_.publish(msg_degree_);
        ROS_INFO("Published angles: Teta0: %.2f, Teta1: %.2f, Teta2: %.2f, Teta3: %.2f, Teta4: %.2f", 
                 msg_degree_.teta0, msg_degree_.teta1, msg_degree_.teta2, msg_degree_.teta3, msg_degree_.teta4);
    }

    void run() {
        ros::Rate rate(10);

        // Wait for subscriber to connect
        ros::Duration(1.0).sleep();

        for (int count = 0; count < PICKUP_POSITIONS && ros::ok(); count++) {
            ROS_INFO("Starting pick and place sequence %d", count + 1);

            if (!moveToScanPosition()) {
                ROS_ERROR("Failed to move to scan position");
                continue;
            }
            // Can can lah

            if (!scanForObject()) {
                ROS_WARN("No object found in scan range");
                continue;
            }
            // Scan for object bisa
            // ros::Duration(3.0).sleep();

            if (!pickupObject(count)) {
                ROS_ERROR("Failed to pick up object");
                continue;
            }

            if(!upObject()){
                ROS_WARN("Cannot pick up object");
                continue;               
            }

            if (!placeObject(count)) {
                ROS_ERROR("Failed to place object");
                continue;
            }

            ros::Duration(1.0).sleep();

            ROS_INFO("Completed pick and place sequence %d", count + 1);
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller");
    
    try {
        ArmController controller;
        controller.run();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in arm controller: %s", e.what());
        return 1;
    }

    return 0;
}