/******************************************************************************
*                                                                             *
* New BSD License                                                             *
*                                                                             *
* Copyright (c) 2021, Abhishek Karote, Technische Universität Chemnitz        *
* All rights reserved.                                                        *
*                                                                             *
* Redistribution and use in source and binary forms, with or without          *
* modification, are permitted provided that the following conditions are met: *
*     * Redistributions of source code must retain the above copyright        *
*       notice, this list of conditions and the following disclaimer.         *
*     * Redistributions in binary form must reproduce the above copyright     *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*     * Neither the name of the Technische Universität Chemnitz nor the       *
*       names of its contributors may be used to endorse or promote products  *
*       derived from this software without specific prior written permission. *
*                                                                             *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" *
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
* ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY      *
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR          *
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER  *
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT          *
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY   *
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH *
* DAMAGE.                                                                     *
*                                                                             *
*******************************************************************************
*                                                                             *
* github repository                                                           *
*   https://github.com/TUC-ProAut/                                            *
*                                                                             *
* Chair of Automation Technology, Technische Universität Chemnitz             *
*   https://www.tu-chemnitz.de/etit/proaut                                    *
*                                                                             *
*******************************************************************************
*                                                                             *
* emergency_stop_recover.cpp                                                  *
* ==========================                                                  *
*                                                                             *
******************************************************************************/

// dynamic reconfigure
#include <emergency_stop_recover_pa/regionConfig.h>

// ROS headers
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// msgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// PCL headers
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

// Std C++ headers
#include <vector>
#include <string>
#include <sstream>
#include <boost/lexical_cast.hpp>


namespace proaut {


/****************************[class definition]*******************************/
class EmergencyStop
{
  public:
    explicit EmergencyStop(ros::NodeHandle nh);
    ~EmergencyStop();
    // Std C++ variables
    std::string _pointcloud_topic, _base_link_frame, _velocity_topic, q_arr;
    int _inlier_thresh, _recover_factor_vel;
    double _recover_step_vel;
    bool is_inlier;
    // ROS variables
    ros::Subscriber sub;
    ros::Publisher pub_inlier, vel_pub, pub_e_stop, pub_e_stop_status;
    sensor_msgs::PointCloud2 inlier_cloud_ros;
    std_msgs::Bool e_stop;
    std_msgs::String msg;
    std::vector<double> polygon_list;
    geometry_msgs::Twist vel_msg;
    // PCL variables
    pcl::PointCloud<pcl::PointXYZ> inlier, polygon;

  protected:
    void check_inlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
    void dynam_CB(const emergency_stop_recover_pa::regionConfig &config, uint32_t level);
    dynamic_reconfigure::Server<emergency_stop_recover_pa::regionConfig> reconf_server;
    dynamic_reconfigure::Server<emergency_stop_recover_pa::regionConfig>::CallbackType reconf_cb;
};


/****************************[class instantiation]****************************/
EmergencyStop::EmergencyStop(ros::NodeHandle nh):
  is_inlier(false), q_arr("10000")
{
    reconf_cb = boost::bind(&EmergencyStop::dynam_CB, this, _1, _2);
    reconf_server.setCallback(reconf_cb);

    vel_pub = nh.advertise<geometry_msgs::Twist>(_velocity_topic, 1);
    pub_inlier = nh.advertise<sensor_msgs::PointCloud2>("inlier_cloud", 1);
    pub_e_stop = nh.advertise<std_msgs::Bool>("e_stop", 1);
    pub_e_stop_status = nh.advertise<std_msgs::String>("e_stop_status", 1);
    nh.getParam("pointcloud_topic", _pointcloud_topic);
    sub = nh.subscribe(_pointcloud_topic, 1, &EmergencyStop::cloud_cb, this);
    if (!nh.hasParam("safetyregion"))
    {
        ROS_INFO("No param named 'safetyregion'");
    }
    else
    {
        nh.getParam("safetyregion", polygon_list);
        for (unsigned i = 0; i < polygon_list.size(); i = i+3)
        {
            polygon.push_back(pcl::PointXYZ(polygon_list[i], polygon_list[i+1], polygon_list[i+2]));
        }
        std::stringstream str;
        str << "Safety Region:" << std::endl;
        for ( unsigned i = 0; i < polygon.size(); ++i )
        {
            str << "\t(" << polygon.points[i].x << ", " << polygon.points[i].y
                << ", " << polygon.points[i].z << ')' << std::endl;
        }
        ROS_INFO_STREAM(str.str());
    }
    ROS_INFO_STREAM("Pointcloud topic : " << _pointcloud_topic);
    ROS_INFO_STREAM("Velocity  topic : " << _velocity_topic);
    ROS_INFO_STREAM("base link frame : " << _base_link_frame);
}

EmergencyStop::~EmergencyStop()
{

}


/****************************[callbacks]**************************************/
void EmergencyStop::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *cloud);
    std::vector<int> removedNan;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, removedNan);
    if ( cloud->size() > 0 )
    {
        this->check_inlier(cloud);
    }
}

void EmergencyStop::dynam_CB(const emergency_stop_recover_pa::regionConfig &config, uint32_t level)
{
    _pointcloud_topic   = config.pointcloudTopic;
    _base_link_frame    = config.baseLinkFrame;
    _velocity_topic     = config.velocityTopic;
    _inlier_thresh      = config.inlierThresh;
    _recover_factor_vel = config.recover_factor_vel;
    _recover_step_vel   = config.recover_step_vel;
}


/****************************[pointcloud segmentation]************************/
void EmergencyStop::check_inlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    inlier.points.clear();
    int inlier_counter = 0;
    q_arr = "10000";
    sensor_msgs::PointCloud2Modifier pcd_modifier(inlier_cloud_ros);

    for ( int i=0; i < cloud->points.size() ; i++ )
    {
        if ( pcl::isXYPointIn2DXYPolygon( cloud->points[i], polygon) )
        {
            // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
            // flagging the respective quadrants
            // if atleast 1 point lies in a respective quadrant
            //         x +ve
            //         2 | 1
            // y -ve ----|----  y +ve
            //         3 | 4
            //         x -ve
            if ( cloud->points[i].x > 0 && cloud->points[i].y > 0 )
                {q_arr.at(1) = '1';}
            if ( cloud->points[i].x > 0 && cloud->points[i].y < 0 )
                {q_arr.at(2) = '1';}
            if ( cloud->points[i].x < 0 && cloud->points[i].y < 0 )
                {q_arr.at(3) = '1';}
            if ( cloud->points[i].x < 0 && cloud->points[i].y > 0 )
                {q_arr.at(4) = '1';}

            inlier.push_back(cloud->points[i]);
            inlier_counter++;
        }
    }

    if ( inlier_counter > _inlier_thresh )
    {
        is_inlier = true;
        e_stop.data = true;
        pub_e_stop.publish(e_stop);

        // recovery behaviour
        // printf (" \t Inliers: %c%c%c%c\n",q_arr[1],q_arr[2],q_arr[3],q_arr[4]);
        int number = boost::lexical_cast < int > (q_arr);
        // x is red, y green and z is blue
        // sending velocties to youbot
        //         x +ve
        //         2 | 1
        // y +ve ----|----  y -ve
        //         3 | 4
        //          x -ve

        switch ( number )
        {
            case 11000 :  // points only in q1 -- move to q3
                vel_msg.linear.x = -_recover_step_vel;
                vel_msg.linear.y = _recover_step_vel;
                // ss << "hello there";
                msg.data = "Object(s) detected in Front Left Corner --> Moving to Back Right Corner";
                break;
            case 10100 :  // points only in q2 -- move to q4
                vel_msg.linear.x = -_recover_step_vel;
                vel_msg.linear.y = -_recover_step_vel;
                msg.data = "Object(s) detected in Front Right Corner --> Moving to Back Left Corner";
                break;
            case 10010 :  // points only in q3 -- move to q1
                vel_msg.linear.x = _recover_step_vel;
                vel_msg.linear.y = -_recover_step_vel;
                msg.data = "Object(s) detected in Back Right Corner --> Moving to Front Left Corner";
                break;
            case 10001 :  // points only in q4 -- move to q2
                vel_msg.linear.x = _recover_step_vel;
                vel_msg.linear.y = _recover_step_vel;
                msg.data = "Object(s) detected in Back Left Corner --> Moving to Front Right Corner";
                break;
            case 11100 :  // points only in q1 & q2 --  roll back
                vel_msg.linear.x = -_recover_step_vel;
                vel_msg.linear.y = 0;
                msg.data = "Object(s) detected ahead --> Rolling backwards";
                break;
            case 10110 :  // points only in q2 & q3 --  slide right
                vel_msg.linear.x = 0;
                vel_msg.linear.y = -_recover_step_vel;
                msg.data = "Object(s) detected on Right --> Sliding towards Left";
                break;
            case 10011 :  // points only in q3 & q4 -- roll forward
                vel_msg.linear.x = _recover_step_vel;
                vel_msg.linear.y = 0;
                msg.data = "Object(s) detected behind --> Moving forward";
                break;
            case 11001 :  // points only in q1 & q4 --  slide left
                vel_msg.linear.x = 0;
                vel_msg.linear.y = _recover_step_vel;
                msg.data = "Object(s) detected on Left --> Sliding towards Right";
                break;
            default   :  // zero velocity for others -- call help
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                msg.data = "Too many Object(s)s detected in vicinity --> Please move robot manually";
                break;
            }

            // Update the Twist message
            vel_msg.linear.x = _recover_factor_vel * vel_msg.linear.x;
            vel_msg.linear.y = _recover_factor_vel * vel_msg.linear.y;
            vel_msg.linear.z = 0;
            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
            // printf ("\t(%f , %f)\n", vel_msg.linear.x, vel_msg.linear.y);
            pub_e_stop_status.publish(msg);
        }
        else
        {
            is_inlier = false;
            e_stop.data = false;
            pub_e_stop.publish(e_stop);
        }

        if ( inlier.size() != 0 )
        {
            pcl::toROSMsg(inlier, inlier_cloud_ros);
        }
        else
        {
            pcd_modifier.clear();
        }

        // publish ros messages
        inlier_cloud_ros.header.frame_id = cloud->header.frame_id;
        inlier_cloud_ros.header.stamp = ros::Time::now();
        pub_inlier.publish(inlier_cloud_ros);
    }

}  // namespace proaut


/****************************[main]*******************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_stop_recover_pa");
    ros::NodeHandle nh("~");
    proaut::EmergencyStop emergency_stop(nh);
    ros::spin();
    return 0;
}
