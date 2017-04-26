/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"/home/doom/odom_aid/ORB_SLAM2/include/System.h"

#include "/home/doom/odom_aid/ORB_SLAM2/include/Odom/odomdata.h"
#include "/home/doom/odom_aid/ORB_SLAM2/include/Odom/configparam.h"

#include "MsgSync/MsgSynchronizer.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Odom");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Odom path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ORB_SLAM2::ConfigParam config(argv[2]);

    /**
     * @brief added data sync
     */
    double imageMsgDelaySec = config.GetImageDelayToOdom();
    ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);
    ros::NodeHandle nh;
    ros::Subscriber limagesub = nh.subscribe("/camera/left/image_raw", 200, &ORBVIO::MsgSynchronizer::leftImageCallback, &msgsync);
    ros::Subscriber rimagesub = nh.subscribe("/camera/right/image_raw", 200, &ORBVIO::MsgSynchronizer::rightImageCallback, &msgsync);
    ros::Subscriber odomsub = nh.subscribe("/base_odometry/odom", 200, &ORBVIO::MsgSynchronizer::odomCallback, &msgsync);

    sensor_msgs::ImageConstPtr limageMsg;
    sensor_msgs::ImageConstPtr rimageMsg;
    //std::vector<sensor_msgs::ImuConstPtr> vimuMsg;
    std::vector<OdomConstPtr> vodomMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    //const double g3dm = 9.80665;
    //const bool bAccMultiply98 = config.GetAccMultiply9p8();


    //std::string bagfile = config._bagfile;
    //rosbag::Bag bag;
    //bag.open(bagfile,rosbag::bagmode::Read);

    //std::vector<std::string> topics;
    //std::string imutopic = config._imuTopic;
    //std::string odomtopic = config._odomTopic;
    //std::string limagetopic = config._limageTopic;
    //std::string rimagetopic = config._rimageTopic;
    //topics.push_back(limagetopic);
    //topics.push_back(rimagetopic);
    //topics.push_back(imutopic);
    //topics.push_back(odomtopic);

    //rosbag::View view(bag, rosbag::TopicQuery(topics));

    //ros::Rate r(1000);
    while(ros::ok())
    //BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        //sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
        //nav_msgs::OdomConstPtr sodom = m.instantiate<nav_msgs::Odometry>();
        //if(simu!=NULL)
        //    msgsync.imuCallback(simu);
	//if(sodom!=NULL)
	//      msgsync.odomCallback(sodom);
        //sensor_msgs::ImageConstPtr slimage = m.instantiate<sensor_msgs::Image>();
        //sensor_msgs::ImageConstPtr srimage = m.instantiate<sensor_msgs::Image>();
        //if(slimage!=NULL)
        //    msgsync.leftImageCallback(simage);
	//if(slimage!=NULL)
        //    msgsync.leftImageCallback(simage);
        bool bdata = msgsync.getRecentMsgs(limageMsg, rimageMsg, vodomMsg);

        if(bdata)
        {
            std::vector<ORB_SLAM2::OdomData> vodomData;
            //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
            for(unsigned int i=0;i<vodomMsg.size();i++)
            {
                OdomConstPtr odomMsg = vodomMsg[i];
                double vx = odomMsg->twist.twist.linear.x;
                double vy = odomMsg->twist.twist.linear.y;
                double vz = odomMsg->twist.twist.linear.z;

		double wx = odomMsg->twist.twist.angular.x; 
		double wy = odomMsg->twist.twist.angular.y;
		double wz = odomMsg->twist.twist.angular.z;

                ORB_SLAM2::OdomData odomdata(vx,vy,vz,
                                	     wx,wy,wz,
					     odomMsg->header.stamp.toSec());
                vodomData.push_back(odomdata);
                //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
            }

            // Copy the ros image message to cv::Mat.
            cv_bridge::CvImageConstPtr cv_ptr_l, cv_ptr_r;
            try
            {
                cv_ptr_l = cv_bridge::toCvShare(limageMsg);
		cv_ptr_r = cv_bridge::toCvShare(rimageMsg);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }

            // Consider delay of image message
            //SLAM.TrackMonocular(cv_ptr->image, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
            cv::Mat imLeft = cv_ptr_l->image.clone();
	    cv::Mat imRight = cv_ptr_r->image.clone();
            SLAM.TrackStereoOdom(imLeft, imRight, vodomData, limageMsg->header.stamp.toSec() - imageMsgDelaySec);
            //SLAM.TrackMonoVI(cv_ptr->image, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
            //cv::imshow("image",cv_ptr->image);

            // Wait local mapping end.
            bool bstop = false;
            while(!SLAM.bLocalMapAcceptKF())
            {
                if(!ros::ok())
                {
                    bstop=true;
                }
            };
            if(bstop)
                break;

        }

        //cv::waitKey(1);

        ros::spinOnce();
        //r.sleep();
        if(!ros::ok())
            break;
    }



//    ImageGrabber igb(&SLAM);

//    ros::NodeHandle nodeHandler;
//    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

//    ros::spin();


    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");

    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

//void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
//{
//    // Copy the ros image message to cv::Mat.
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvShare(msg);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

//    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
//}


