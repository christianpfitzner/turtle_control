

/* Copyright (c), Prof. Dr. Christian Pfitzner, All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
 */


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


ros::Subscriber lidar_sub; 
ros::Publisher vel_pub;


// callback function for lidar 
void lidarCallback(const sensor_msgs::LaserScan msg)
{
    // calculate the closest point to the robot
    double min_dist = msg.range_max;
    int min_index = 0;
    for (int i = 0; i < msg.ranges.size(); i++)
    {
        if (msg.ranges[i] < min_dist && msg.ranges[i] != 0.0)
        {
            ROS_ERROR("Lidar: %f", msg.ranges[i]);
            min_dist = msg.ranges[i];
            min_index = i;
        }
    }


    // drive the robot with 0.3 m/s forward
    geometry_msgs::Twist vel; 
    vel.linear.x  = 1.0; 
    vel.angular.z = 0.0; 

    // if the closest point is less than 0.5 meters away, stop the robot
    if (min_dist < 0.5)
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }

    // publish the velocity command
    vel_pub.publish(vel);
}


 
 int main(int argc, char* argv[])
 {
    ros::init(argc, argv, "turtle_obstacle_stop");
    ros::NodeHandle nh;


    vel_pub    = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    lidar_sub  = nh.subscribe("scan", 1, lidarCallback);


    // this comand replaces the while(ros::ok())-loop and the sleep command
    // the complete computation in this node is done in one callback function
    ros::spin(); 

 }