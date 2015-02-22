#include "k2_client.h"

int imageSize = 512*424*8;
int streamSize = imageSize;
std::string cameraName = "mapping";
std::string imageTopicName = "depth_mapping";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "startMapping");
    ros::NodeHandle n(cameraName);
    image_transport::ImageTransport imT(n);
    std::string serverAddress;
    n.getParam("/serverNameOrIP",serverAddress);
    Socket mySocket(serverAddress.c_str(),"9005",streamSize);
    image_transport::Publisher imagePublisher = imT.advertise(
            imageTopicName, 1);
    cv::Mat frame;
    cv_bridge::CvImage cvImage;
    sensor_msgs::Image rosImage;
    while(ros::ok())
    {
        mySocket.readData();
        frame = cv::Mat(cv::Size(512,424), CV_16UC4, mySocket.mBuffer);
        cv::flip(frame, frame, 1);
        double utcTime;
        memcpy(&utcTime,&mySocket.mBuffer[imageSize],sizeof(double));
        cvImage.encoding = "16UC4";
        cvImage.image = frame;
        cvImage.toImageMsg(rosImage);
        imagePublisher.publish(rosImage);
        ros::spinOnce();
    }
}
