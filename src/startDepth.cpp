/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list 
 	of conditions and the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this 
 	list of conditions and the following disclaimer in the documentation and/or other 
 	materials provided with the 	distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors 
 	may be used to endorse or promote products derived from this software without 
 	specific prior written 	permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include "k2_client/k2_client.h"

// this alternate resolution for an aligned depth image
//int imageSize = 639392;
int imageSize = 1920 * 1080 * 6;
int streamSize = imageSize;
std::string cameraName = "depth";
std::string imageTopicSubName = "image_depth";
std::string cameraFrame = "";

int main(int argC,char **argV)
{
    ros::init(argC,argV,"startDepth");
    ros::NodeHandle n(cameraName);
    image_transport::ImageTransport imT(n);
    std::string serverAddress;
    n.getParam("/serverNameOrIP",serverAddress);
    n.getParam(ros::this_node::getNamespace().substr(1,std::string::npos) +
            "/depth_frame", cameraFrame);
    Socket mySocket(serverAddress.c_str(), "18000", streamSize);
    image_transport::Publisher imagePublisher = imT.advertise(imageTopicSubName, 1);
    cv::Mat frame;
    cv_bridge::CvImage cvImage;
    sensor_msgs::Image rosImage;
    while(ros::ok())
    {
        mySocket.readData();
//      for(int i=0; i < 6; i++) {
//          printf("%02x", (unsigned char)(mySocket.mBuffer[(1920 * 1079 + 1000) * 6 + i]));
//      }
//      printf("\n");
        frame = cv::Mat(cv::Size(1920,1080), CV_16SC3,mySocket.mBuffer);
		cv::flip(frame,frame,0);
        cv::Vec3s test_vec = frame.at<cv::Vec3s>(500, 500);
        cvImage.header.frame_id = cameraFrame.c_str();
		cvImage.encoding = "16SC3";
		cvImage.image = frame;
		cvImage.toImageMsg(rosImage);
        imagePublisher.publish(rosImage);
		ros::spinOnce();
    }
    return 0;
}
