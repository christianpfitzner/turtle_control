

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




 // a ros node which publishes the velocity for a turtlebot to move to a certain position based on differential drive kinematics parameters



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


// the differential drive kinematics parameters
double wheel_radius = 0.0;
double wheel_base = 0.0;
double max_velocity = 0.0;
double max_angular_velocity = 0.0;

// the current position and orientation of the robot
double x = 0.0;
double y = 0.0;
double theta = 0.0;

// the target position and orientation of the robot
double x_target = 0.0;
double y_target = 0.0;
// double theta_target = 0.0;

// the velocity command
geometry_msgs::Twist vel;
vel.linear.x = 0.0;
vel.linear.y = 0.0;


// callback function for odometry



void odometryCallback(const nav_msgs::Odometry msg)
{
    // update the current position and orientation
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    theta = tf::getYaw(msg.pose.pose.orientation);
}




// callback function for target position
void pointCallback(const nav_msgs::Point2D msg)
{
    // update the target position and orientation
    x_target = msg.x;
    y_target = msg.y;
    // theta_target = atan2(y_target - y, x_target - x);
}











int main(int argc, char* argv[])
{
    ros::init(argc, argv, "turtle_control_kinematics");
    ros::NodeHandle nh;


    ros::Subscriber odom_sub     = nh.subscribe("odom", 1, odometryCallback);
    ros::Subscriber target_sub   = nh.subscribe("target", 1, pointCallback);
    ros::Publisher vel_pub       = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);


    ros::Rate rate(10);
    while(ros::ok())
    {


        rate.sleep();
    }
    
}


