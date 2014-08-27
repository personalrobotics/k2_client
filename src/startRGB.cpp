#include "k2_client.h"

int imageSize = 8294400;
int streamSize = imageSize + sizeof(double);
std::string cameraName = "head/kinect2/rgb";
std::string imageTopicSubName = "/image_color";
std::string cameraInfoSubName = "/camera_info";

int main(int argC,char **argV)
{
	ros::init(argC,argV,"startRGB");
	ros::NodeHandle n;
	image_transport::ImageTransport imT(n);
	image_transport::Publisher imagePublisher = imT.advertise(cameraName+imageTopicSubName,1);
	ros::Publisher cameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(cameraName+cameraInfoSubName,1);
	camera_info_manager::CameraInfoManager camInfoMgr(n,cameraName);
	camInfoMgr.loadCameraInfo("");
	std::string serverAddress;
	n.getParam("/serverNameOrIP",serverAddress);
	Socket mySocket(serverAddress.c_str(),"9000",streamSize);
	cv::Mat frame;
	cv_bridge::CvImage cvImage;
	sensor_msgs::Image rosImage;
	while(ros::ok())
	{
		mySocket.readData();
		frame = cv::Mat(cv::Size(1920,1080),CV_8UC4,mySocket.mBuffer);
		cv::flip(frame,frame,1);
		double utcTime;
		memcpy(&utcTime,&mySocket.mBuffer[imageSize],sizeof(double));
		cvImage.header.stamp = ros::Time(utcTime);
		cvImage.header.frame_id = "/head/kinect2/colorFrame";
		cvImage.encoding = "bgra8";
		cvImage.image = frame;
		cvImage.toImageMsg(rosImage);
		sensor_msgs::CameraInfo camInfo = camInfoMgr.getCameraInfo();
		camInfo.header.stamp = cvImage.header.stamp;
		camInfo.header.frame_id = cvImage.header.frame_id;
		cameraInfoPub.publish(camInfo);
		imagePublisher.publish(rosImage);
		ros::spinOnce();
	}
	return 0;
}
