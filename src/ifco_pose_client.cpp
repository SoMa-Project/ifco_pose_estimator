// Copyright 2019 Ocado Innovation Limited

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "ifco_pose_estimator/ifco_pose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ifco_pose_client");
    ros::NodeHandle n;
    
    ros::ServiceClient client = n.serviceClient<ifco_pose_estimator::ifco_pose>("ifco_pose");
    
    ifco_pose_estimator::ifco_pose srv;
    tf::TransformBroadcaster br;

    n.setParam("/ifco/length", 0.57);
    n.setParam("/ifco/width", 0.37);
    n.setParam("/ifco/height", 0.17);
    
    srv.request.publish_ifco = false;

    if (client.call(srv))
    {
        ROS_INFO("Service was called");
        ROS_INFO("Fitness value was %f", srv.response.fitness);
        tf::Transform cam_to_ifco;
        tf::poseMsgToTF(srv.response.pose, cam_to_ifco); 
        while(true) br.sendTransform(tf::StampedTransform(cam_to_ifco, ros::Time::now(), "camera", "ifco_bottom_centre"));
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }


    return 0;
}