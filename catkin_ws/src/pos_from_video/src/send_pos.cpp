/**
 * @file send_pos.cpp
 * @author RODRIGUES Nicolas (nico.r648@gmail.com)
 * @brief Just for test, a node which send pose to the uav.
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <geometry_msgs/PoseStamped.h>

bool send_next_pose = false;//To know if the next pose is needed
/**
 * @brief Call by the subscription to the topic "send_next_pose" to store the bool
 * 
 * @param msg msg get by the subscriber
 */
void next_pose_cb(const std_msgs::Bool::ConstPtr& msg){
    std_msgs::Bool next_pose = *msg;
    send_next_pose = next_pose.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_pos_node");
    ros::NodeHandle nh;
    //Publish to "drone_pose_to_go" to push the pose to go
    ros::Publisher current_pose_to_go = nh.advertise<geometry_msgs::PoseStamped>
        ("drone_pose_to_go", 10);
    //Subscribe to "send_next_pose" to know if the node has to send the next pose
    ros::Subscriber send_next_sub = nh.subscribe<std_msgs::Bool>
        ("send_next_pose", 10, next_pose_cb);
    //to match the publishing rate needed by the offb_node
    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped tab_pose_tmp[5];
    float x_max = 15;
    float y_max = 12;
    float z_max = 10;
    geometry_msgs::PoseStamped pose_tmp;
    //Generate random pose to test
    for(int i = 0; i < 5 ; i++){
        //generate x as 0 <= x <= x_max
        pose_tmp.pose.position.x = ((float)rand()/(float)(RAND_MAX)) * x_max;
        //generate y as 0 <= y <= y_max
        pose_tmp.pose.position.y = ((float)rand()/(float)(RAND_MAX)) * y_max;
        //generate z as 0 <= y <= z_max
        pose_tmp.pose.position.z = ((float)rand()/(float)(RAND_MAX)) * z_max;
        if(pose_tmp.pose.position.z == 0){
            //to ensure safety, z need to be > 1.5
            pose_tmp.pose.position.z = pose_tmp.pose.position.z + 2;
        }
        
        tab_pose_tmp[i] = pose_tmp;
    }
    int i = 0;
    while(ros::ok()){
        if(i == 5){
            i = 0;
        }
        current_pose_to_go.publish(tab_pose_tmp[i]);
        if(send_next_pose == true) {
            i++;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}
