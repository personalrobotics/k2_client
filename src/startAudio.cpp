#include "k2_client.h"

std::string topicName = "/head/kinect2/audio";
int twiceStreamSize = 8200;
int streamSize = 4100;

int main(int argC,char **argV)
{
	ros::init(argC,argV,"startAudio");
	ros::NodeHandle n;
	ros::Publisher audioPub = n.advertise<k2_client::Audio>(topicName,1);
	Socket mySocket("172.19.179.53","9004",twiceStreamSize);
	while(ros::ok())
	{
		mySocket.readData();
		std::string jsonString;
		for(int i=0;i<twiceStreamSize;i+=2)
		{
			jsonString += mySocket.mBuffer[i];
		}
		Json::Value jsonObject;
		Json::Reader jsonReader;
		bool parsingSuccessful = jsonReader.parse(jsonString,jsonObject,false);
		k2_client::Audio audio;
		audio.header.stamp = ros::Time(jsonObject["utcTime"].asDouble());
		audio.header.frame_id = "/head/kinect2/microphone_frame";
		audio.beamAngle = jsonObject["beamAngle"].asDouble();
		audio.beamAngleConfidence = jsonObject["beamAngleConfidence"].asDouble();
		for(int i=0;i<256;i++)
		{
			audio.audioStream.push_back(jsonObject["audioStream"][i].asFloat());
		}
		audio.numBytesPerSample = jsonObject["numBytesPerSample"].asUInt();
		audio.numSamplesPerFrame = jsonObject["numSamplesPerFrame"].asUInt();
		audio.frameLifeTime = jsonObject["frameLifeTime"].asDouble();
		audio.samplingFrequency = jsonObject["samplingFrequency"].asUInt();
		audioPub.publish(audio);
	}
	return 0;
}