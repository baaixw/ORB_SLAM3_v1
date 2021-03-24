/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include <queue>
#include <map>
#include <queue>
#include <mutex>
#include <thread>

#include"../../../include/System.h"

using namespace std;
std::queue<sensor_msgs::ImageConstPtr> img0_buf;// reserve image

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void pubImage()
    { 
        ros::Time t = ros::Time::now();
       int image_index = 1;
        while(1 && image_index<147)
        {
            std::string img_file_path;
            img_file_path = "/home/wws/ORB_SLAM3/Examples/Monocular/NewTsukuba/"; 
            if(image_index>=0 && image_index<10)
            {
                img_file_path = img_file_path +"image000"+std::to_string(image_index) + ".jpg";//
            }
            else if(image_index>=10 && image_index<100)
            {
                img_file_path = img_file_path +"image00"+std::to_string(image_index) + ".jpg";//
            }
            else if(image_index>=100 && image_index<1000)
            {
                img_file_path = img_file_path +"image0"+std::to_string(image_index) + ".jpg";//
            }
            cout << "img_file_path: " << img_file_path << endl;
            
            cv::Mat im = cv::imread(img_file_path,CV_LOAD_IMAGE_COLOR);
            cv_bridge::CvImage cvImage;
            cvImage.image = im;
            cvImage.encoding = sensor_msgs::image_encodings::RGB8;
            cvImage.header.stamp = t;

            sensor_msgs::ImagePtr imgTrackMsg = cvImage.toImageMsg();
            // // bag_out.write("/camera/image_raw",ros::Time(t),cvImage.toImageMsg());
            img0_buf.push(imgTrackMsg);
            cout << "img0_buf.size(): " << img0_buf.size() << endl;

            std::chrono::milliseconds dura(20); // this thread sleep for 10 ms
            std::this_thread::sleep_for(dura);
            image_index++;
        }
        
    }

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/cam1/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    
    ros::Publisher pub = nodeHandler.advertise<sensor_msgs::Image>("/cam1/image_raw", 100); // 

    /* thread for factor graph optimization */
    std::thread optimizationThread = std::thread(&ImageGrabber::pubImage, &igb);

    ros::Duration(10.0).sleep();  // Sleep for one second

    ros::Rate loop_rate(2);
    while (ros::ok() && img0_buf.size()>0)
    {
        ros::spinOnce();
        cout << "Remaining img0_buf.size(): " << img0_buf.size() << endl;
        
        pub.publish(img0_buf.front());
        img0_buf.pop();
        loop_rate.sleep();
    }
    cout<< "all image proceessed"<<endl;

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


