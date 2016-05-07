#include "k2_client/k2_client.h"
#include "k2_client/FaceArray.h"
#include <iconv.h>
#include <cstdio>

std::string topicName = "faceArray";
size_t streamSize = 60000;
size_t readSkipSize = 60000;
size_t stringSize = 30000;

int main(int argC,char **argV)
{
    ros::init(argC,argV,"startFace");
    ros::NodeHandle n;
    std::string serverAddress;
    n.getParam("/serverNameOrIP",serverAddress);
    Socket mySocket(serverAddress.c_str(),"9006",streamSize);
    ros::Publisher facePub = n.advertise<k2_client::FaceArray>(topicName,1);
    while(ros::ok())
    {
        mySocket.readData();
        std::string jsonString(mySocket.mBuffer);
        Json::Value jsonObject;
        Json::Reader jsonReader;
        bool parsingSuccessful = jsonReader.parse(jsonString,jsonObject,false);
        if(!parsingSuccessful)
        {
            ROS_ERROR("Failure to parse");
            continue;
        }
        k2_client::FaceArray faceArray;
        try
        {
            for(int i=0;i<6;i++)
            {
                k2_client::Face face;
                face.header.stamp = ros::Time::now();
                face.header.frame_id =  ros::this_node::getNamespace().substr(1,std::string::npos) + "/depthFrame";
                
                face.appearance.wearingGlasses = jsonObject[i]["FaceProperties"]["WearingGlasses"].asBool();
                face.activities.eyeLeftClosed = jsonObject[i]["FaceProperties"]["EyeLeftClosed"].asBool();
                face.activities.eyeRightClosed = jsonObject[i]["FaceProperties"]["EyeRightClosed"].asBool();
                face.activities.mouthOpen = jsonObject[i]["FaceProperties"]["MouthOpen"].asBool();
                face.activities.mouthMoved = jsonObject[i]["FaceProperties"]["MouthMoved"].asBool();
                face.activities.lookingAway = jsonObject[i]["FaceProperties"]["LookingAway"].asBool();
                face.expressions.neutral = jsonObject[i]["FaceProperties"]["Neutral"].asBool();
                face.expressions.neutral = jsonObject[i]["FaceProperties"]["Happy"].asBool();
                
                face.facePointsInInfraredSpace.eyeLeftX = jsonObject[i]["FacePointsInInfraredSpace"]["EyeLeft"]["X"];
                face.facePointsInInfraredSpace.eyeLeftY = jsonObject[i]["FacePointsInInfraredSpace"]["EyeLeft"]["Y"];
                face.facePointsInInfraredSpace.eyeRightX = jsonObject[i]["FacePointsInInfraredSpace"]["EyeRight"]["X"];
                face.facePointsInInfraredSpace.eyeRightY = jsonObject[i]["FacePointsInInfraredSpace"]["EyeRight"]["Y"];
                face.facePointsInInfraredSpace.noseX = jsonObject[i]["FacePointsInInfraredSpace"]["EyeRight"]["X"];
                face.facePointsInInfraredSpace.noseY = jsonObject[i]["FacePointsInInfraredSpace"]["EyeRight"]["Y"];
                face.facePointsInInfraredSpace.mouthCornerLeftX = jsonObject[i]["FacePointsInInfraredSpace"]["MouthCornerLeft"]["X"];
                face.facePointsInInfraredSpace.mouthCornerLeftY = jsonObject[i]["FacePointsInInfraredSpace"]["MouthCornerLeft"]["Y"];
                face.facePointsInInfraredSpace.mouthCornerRightX = jsonObject[i]["FacePointsInInfraredSpace"]["MouthCornerRight"]["X"];
                face.facePointsInInfraredSpace.mouthCornerRightY = jsonObject[i]["FacePointsInInfraredSpace"]["MouthCornerRight"]["Y"];
                
                face.facePointsInColorSpace.eyeLeftX = jsonObject[i]["FacePointsInColorSpace"]["EyeLeft"]["X"];
                face.facePointsInColorSpace.eyeLeftY = jsonObject[i]["FacePointsInColorSpace"]["EyeLeft"]["Y"];
                face.facePointsInColorSpace.eyeRightX = jsonObject[i]["FacePointsInColorSpace"]["EyeRight"]["X"];
                face.facePointsInColorSpace.eyeRightY = jsonObject[i]["FacePointsInColorSpace"]["EyeRight"]["Y"];
                face.facePointsInColorSpace.noseX = jsonObject[i]["FacePointsInColorSpace"]["EyeRight"]["X"];
                face.facePointsInColorSpace.noseY = jsonObject[i]["FacePointsInColorSpace"]["EyeRight"]["Y"];
                face.facePointsInColorSpace.mouthCornerLeftX = jsonObject[i]["FacePointsInColorSpace"]["MouthCornerLeft"]["X"];
                face.facePointsInColorSpace.mouthCornerLeftY = jsonObject[i]["FacePointsInColorSpace"]["MouthCornerLeft"]["Y"];
                face.facePointsInColorSpace.mouthCornerRightX = jsonObject[i]["FacePointsInColorSpace"]["MouthCornerRight"]["X"];
                face.facePointsInColorSpace.mouthCornerRightY = jsonObject[i]["FacePointsInColorSpace"]["MouthCornerRight"]["Y"];
                
                face.faceBoundingBoxInInfraredSpace.left   = jsonObject[i]["FaceBoundingBoxInInfraredSpace"]["Left"];
                face.faceBoundingBoxInInfraredSpace.top    = jsonObject[i]["FaceBoundingBoxInInfraredSpace"]["Top"];
                face.faceBoundingBoxInInfraredSpace.right  = jsonObject[i]["FaceBoundingBoxInInfraredSpace"]["Right"];
                face.faceBoundingBoxInInfraredSpace.bottom = jsonObject[i]["FaceBoundingBoxInInfraredSpace"]["Bottom"];
                
                face.FaceBoundingBoxInColorSpace.left   = jsonObject[i]["FaceBoundingBoxInColorSpace"]["Left"];
                face.FaceBoundingBoxInColorSpace.top    = jsonObject[i]["FaceBoundingBoxInColorSpace"]["Top"];
                face.FaceBoundingBoxInColorSpace.right  = jsonObject[i]["FaceBoundingBoxInColorSpace"]["Right"];
                face.FaceBoundingBoxInColorSpace.bottom = jsonObject[i]["FaceBoundingBoxInColorSpace"]["Bottom"];
                
                face.faceRotationQuaternion.X = jsonObject[i]["FaceRotationQuaternion"]["X"];
                face.faceRotationQuaternion.Y = jsonObject[i]["FaceRotationQuaternion"]["Y"];
                face.faceRotationQuaternion.Z = jsonObject[i]["FaceRotationQuaternion"]["Z"];
                face.faceRotationQuaternion.W = jsonObject[i]["FaceRotationQuaternion"]["W"];
                
                face.trackingId = jsonObject[i]["TrackingId"];
                face.faceFrameFeatures = jsonObject[i]["FaceFrameFeatures"];
                
                faceArray.faces.push_back(face);
            }
        }
        catch (...)
        {
            ROS_ERROR("An exception occured");
            continue;
        }
        if (faceArray.faces.size() > 0) 
            facePub.publish(faceArray);
    }
    return 0;
}
