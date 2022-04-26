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
#include <turtlebot3_msgs/Sound.h>



ros::Publisher sound_pub;





void lidarCallback(const sensor_msgs::LaserScan msg)
{
    // calculate the closest point to the robot
    double min_dist = msg.range_max;
    int min_index = 0;
    for (int i = 0; i < msg.ranges.size(); i++)
    {
        if (msg.ranges[i] < min_dist && msg.ranges[i] != 0.0)
        {
            min_dist = msg.ranges[i];
            min_index = i;
        }
    }


    ROS_ERROR("Lidar: %f", min_dist);

    if(min_dist < 0.5)
    {
        turtlebot3_msgs::Sound sound;
        sound.value = 3;
        sound_pub.publish(sound);
    }
}




int main(int argc, char* argv[])
{
    ros::init(argc, argv, "turtle_obstacle_beep");
    ros::NodeHandle nh;


    sound_pub                = nh.advertise<turtlebot3_msgs::Sound>("sound", 1);
    ros::Subscriber lidar_sub = nh.subscribe("scan", 1, lidarCallback);
    
    

    ros::spin(); 
}