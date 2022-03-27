/**
 * @file offb_node.cpp
 * @author RODRIGUES Nicolas (nico.r648@gmail.com)
 * @brief Control of the uav, set it in offboard mode ; based on 
 * Offboard control example node, written with MAVROS version 0.19.x, 
 * PX4 Pro Flight 
 * Stack and tested in Gazebo SITL
 * @version 0.1
 * @date 2022-03-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

mavros_msgs::State current_state;//current state of uav

/**
 * @brief Call by the subscription to the topic "mavros/global_position/global" to store the state
 * 
 * @param msg msg get by the subscriber
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;  
}

nav_msgs::Odometry current_local_pose;//current pose of the uav in uav space
/**
 * @brief Call by the subscription to the topic "mavros/global_position/local" to store the position
 * 
 * @param msg msg get by the subscriber
 */
void local_pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_local_pose = *msg;
}

geometry_msgs::PoseStamped pose_to_go;
/**
 * @brief Call by the subscription to the topic "drone_pose_to_go" to know the current pose to reach
 * 
 * @param msg msg get by the subscriber
 */
void pose_to_go_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_to_go = *msg;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    //Subscribe to "mavros/global_position/local" to get the local pose of the frame
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>
        ("mavros/global_position/local",10, local_pose_cb);
    //Subscribe to "mavros/state" to get data
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //Publish to "mavros/setpoint_position/local" to push data on controller
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //Ask the uav a service, here to arm the uav
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //Ask the uav a service, here to change the mode of the uav
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //Publish true if a the current pose to go is reach, otherwise false
    ros::Publisher send_next_pose_pub = nh.advertise<std_msgs::Bool>
        ("send_next_pose", 10);
    //Subscribe to "mavros/state" to get the pose to reach
    ros::Subscriber current_pos_to_go_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("drone_pose_to_go", 10, pose_to_go_cb);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //The pose the uav goes
    geometry_msgs::PoseStamped pose;
    
    //send a few setpoints before starting (needed before switching to offboard mode)
    for(int i = 100; ros::ok() && i > 0; --i){
       
            
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 5;
       
            
            
        
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //Set the msg to switch on the offboard mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //Set the msg to arm the uav
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        //Check if already in offboard mode
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            //Send a request to switch mode   
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            //Check if already armed
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                //If not arm the uav    
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armeds");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        pose.pose.position.x = pose_to_go.pose.position.x;
        pose.pose.position.y = pose_to_go.pose.position.y;
        pose.pose.position.z = pose_to_go.pose.position.z;
        
        if( current_state.mode == "OFFBOARD" && current_state.armed){
            //While the current pose to go is not reached send the pose
            while((current_local_pose.pose.pose.position.z < pose.pose.position.z - 1 || current_local_pose.pose.pose.position.z > pose.pose.position.z + 1)
            || (current_local_pose.pose.pose.position.y < pose.pose.position.y - 1 || current_local_pose.pose.pose.position.y > pose.pose.position.y + 1)
            || (current_local_pose.pose.pose.position.x < pose.pose.position.x - 1 || current_local_pose.pose.pose.position.x > pose.pose.position.x + 1))
            {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            } 
            //Ask for the next pose to go
            std_msgs::Bool send_next;
            send_next.data = true;
            send_next_pose_pub.publish(send_next);
            
            
            ros::spinOnce();
            rate.sleep();
            
            send_next.data = false;
            send_next_pose_pub.publish(send_next);
            ros::spinOnce();
            rate.sleep();
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}